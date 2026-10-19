// TcmKbd.efi - resident UEFI boot-service driver for the OnePlus 11 (SM8550):
//   * brings up the S3908 touch controller exactly like TcmTouch.efi / SeProbe did
//     (RPMh L4B -> reset -> QUP1 SE4 clocks + gpio44/45 -> polled GENI I2C),
//   * draws an on-screen keyboard at the bottom of the GOP framebuffer,
//   * injects the keys into the firmware console by patching the function pointers of the
//     existing ConIn / ConInEx protocols in place, so GRUB / UEFI shell / any app sees them.
//
// Load it from GRUB BEFORE the menu is shown:
//     chainloader /boot/TcmKbd.efi
//     boot
// It is linked as a boot-service driver: if efi_main returns EFI_SUCCESS the image stays
// resident (there is deliberately no Unload handler, so UnloadImage refuses to remove it).
//
// Build: see build.sh   (note: /subsystem:efi_boot_service_driver, not efi_application)
//
// Design notes
//  - A 10 ms periodic timer event (TPL_CALLBACK) polls the controller when INT (gpio25) is low,
//    turns reports into per-finger down/up edges, hit-tests the key table and queues keys.
//  - GOP is only drawn when no ConOut call is in flight (ConOut is wrapped with a busy counter),
//    and the keyboard is redrawn when the text console may have overwritten it.
//  - ExitBootServices: DISABLE_REPORT + restore clock votes / pin mux, so the OS driver
//    starts from the same state the first probe found.
#include <stdarg.h>

typedef unsigned char UINT8;
typedef unsigned short UINT16;
typedef unsigned int UINT32;
typedef unsigned long long UINT64;
typedef signed int INT32;
typedef UINT64 UINTN;
typedef UINT16 CHAR16;
typedef void VOID;
typedef UINT64 EFI_STATUS;
typedef VOID *EFI_HANDLE;

typedef struct { UINT32 a; UINT16 b, c; UINT8 d[8]; } GUID;

#define EFIERR(n) (0x8000000000000000ULL | (n))
#define EFI_SUCCESS 0ULL
#define EFI_UNSUPPORTED EFIERR(3)
#define EFI_NOT_READY EFIERR(6)
#define EFI_DEVICE_ERROR EFIERR(7)
#define EFI_OUT_OF_RESOURCES EFIERR(9)
#define EFI_NOT_FOUND EFIERR(14)

// ---- user tunables --------------------------------------------------------
#define KB_START_VISIBLE 1
// Raw touch range. Measured: x 277..5719, y 320..12720 -> looks like 4x of 1440x3216.
// Verify by touching the four corners (see the log in TcmTouch) and adjust.
#define TOUCH_RAW_W 5760
#define TOUCH_RAW_H 12864
#define TOUCH_SWAP_XY 0     // 1 if the GOP is landscape while the panel reports portrait
#define TOUCH_INVERT_X 0
#define TOUCH_INVERT_Y 0
#define MSG_RD_LEN 256      // bytes per I2C read (same as Linux / the working probe)
#define TICK_100NS 100000   // 10 ms
#define REPEAT_DELAY_MS 450
#define REPEAT_RATE_MS 70
#define SLOT_TIMEOUT_MS 150 // finger counts as lifted if its slot is silent this long

// ---- minimal EFI tables ---------------------------------------------------
typedef struct { UINT16 ScanCode; CHAR16 Unicode; } KEY;
typedef struct { UINT16 ScanCode; CHAR16 Unicode; UINT32 ShiftState; UINT8 ToggleState; UINT8 pad[3]; } KEYDATA;

typedef struct {
  EFI_STATUS (*Reset)(VOID *, UINT8);
  EFI_STATUS (*ReadKey)(VOID *, KEY *);
  VOID *WaitForKey;
} CONIN;

typedef struct {
  EFI_STATUS (*Reset)(VOID *, UINT8);
  EFI_STATUS (*ReadKeyEx)(VOID *, KEYDATA *);
  VOID *WaitForKeyEx;
  EFI_STATUS (*SetState)(VOID *, UINT8 *);
  EFI_STATUS (*RegNotify)(VOID *, VOID *, VOID *, VOID **);
  EFI_STATUS (*UnregNotify)(VOID *, VOID *);
} CONINEX;

typedef struct { INT32 MaxMode, Mode, Attribute, CursorColumn, CursorRow; UINT8 CursorVisible; } CONOUT_MODE;
typedef struct {
  EFI_STATUS (*Reset)(VOID *, UINT8);
  EFI_STATUS (*OutputString)(VOID *, CHAR16 *);
  EFI_STATUS (*TestString)(VOID *, CHAR16 *);
  EFI_STATUS (*QueryMode)(VOID *, UINTN, UINTN *, UINTN *);
  EFI_STATUS (*SetMode)(VOID *, UINTN);
  EFI_STATUS (*SetAttribute)(VOID *, UINTN);
  EFI_STATUS (*ClearScreen)(VOID *);
  EFI_STATUS (*SetCursorPosition)(VOID *, UINTN, UINTN);
  EFI_STATUS (*EnableCursor)(VOID *, UINT8);
  CONOUT_MODE *Mode;
} CONOUT;

typedef struct {
  char Hdr[24];
  VOID *FirmwareVendor;
  UINT32 FirmwareRevision, pad0;
  VOID *ConInHandle;
  CONIN *ConIn;
  VOID *ConOutHandle;
  CONOUT *ConOut;
  VOID *StdErrHandle, *StdErr, *Runtime;
  char *BS;   // boot services (accessed via slots below)
} SYSTAB;

// Boot services slots (index after the 24-byte table header)
#define BS_SLOT(n) (((VOID **)(ST->BS + 24))[n])
#define BS_RAISE_TPL(t) ((UINTN(*)(UINTN))BS_SLOT(0))(t)
#define BS_RESTORE_TPL(t) ((void (*)(UINTN))BS_SLOT(1))(t)
#define BS_ALLOC_POOL(type, sz, pp) ((EFI_STATUS(*)(UINT32, UINTN, VOID **))BS_SLOT(5))(type, sz, pp)
#define BS_CREATE_EVENT(type, tpl, fn, ctx, ev) \
  ((EFI_STATUS(*)(UINT32, UINTN, VOID *, VOID *, VOID **))BS_SLOT(7))(type, tpl, (VOID *)(fn), ctx, ev)
#define BS_SET_TIMER(ev, type, t) ((EFI_STATUS(*)(VOID *, UINT32, UINT64))BS_SLOT(8))(ev, type, t)
#define BS_SIGNAL_EVENT(ev) ((EFI_STATUS(*)(VOID *))BS_SLOT(10))(ev)
#define BS_CHECK_EVENT(ev) ((EFI_STATUS(*)(VOID *))BS_SLOT(12))(ev)
#define BS_HANDLE_PROTOCOL(h, g, i) ((EFI_STATUS(*)(VOID *, GUID *, VOID **))BS_SLOT(16))(h, g, i)
#define BS_STALL(us) ((EFI_STATUS(*)(UINTN))BS_SLOT(28))(us)
#define BS_WATCHDOG(a, b, c, d) ((EFI_STATUS(*)(UINTN, UINT64, UINTN, VOID *))BS_SLOT(29))(a, b, c, d)
#define BS_LOCATE_PROTOCOL(g, r, i) ((EFI_STATUS(*)(GUID *, VOID *, VOID **))BS_SLOT(37))(g, r, i)

#define TPL_CALLBACK 8
#define TPL_NOTIFY 16
#define EVT_TIMER 0x80000000u
#define EVT_NOTIFY_WAIT 0x00000100u
#define EVT_NOTIFY_SIGNAL 0x00000200u
#define EVT_SIGNAL_EXIT_BOOT_SERVICES 0x00000201u
#define EfiBootServicesData 4

typedef struct {
  UINT64 Revision;
  EFI_STATUS (*Open)(VOID *, VOID **, CHAR16 *, UINT64, UINT64);
  EFI_STATUS (*Close)(VOID *);
  EFI_STATUS (*Delete)(VOID *);
  EFI_STATUS (*Read)(VOID *, UINTN *, VOID *);
  EFI_STATUS (*Write)(VOID *, UINTN *, VOID *);
  EFI_STATUS (*GetPos)(VOID *, UINT64 *);
  EFI_STATUS (*SetPos)(VOID *, UINT64);
  EFI_STATUS (*GetInfo)(VOID *, VOID *, UINTN *, VOID *);
  EFI_STATUS (*SetInfo)(VOID *, VOID *, UINTN, VOID *);
  EFI_STATUS (*Flush)(VOID *);
} FILEP;
typedef struct {
  UINT64 Revision;
  EFI_STATUS (*OpenVolume)(VOID *, FILEP **);
} SFS;

// Graphics output protocol
typedef struct { UINT32 Version, Hres, Vres, PixelFormat; UINT32 PixInfo[4]; UINT32 PixelsPerScanLine; } GOP_INFO;
typedef struct { UINT32 MaxMode, Mode; GOP_INFO *Info; UINTN SizeOfInfo; UINT64 FbBase; UINTN FbSize; } GOP_MODE;
typedef struct {
  VOID *QueryMode;
  VOID *SetMode;
  EFI_STATUS (*Blt)(VOID *, VOID *, UINTN, UINTN, UINTN, UINTN, UINTN, UINTN, UINTN, UINTN);
  GOP_MODE *Mode;
} GOP;
#define BLT_VIDEO_FILL 0
#define BLT_BUFFER_TO_VIDEO 2

static GUID gLoadedImage = {0x5B1B31A1, 0x9562, 0x11D2, {0x8E, 0x3F, 0x00, 0xA0, 0xC9, 0x69, 0x72, 0x3B}};
static GUID gSfs = {0x964E5B22, 0x6459, 0x11D2, {0x8E, 0x39, 0x00, 0xA0, 0xC9, 0x69, 0x72, 0x3B}};
static GUID gGopGuid = {0x9042A9DE, 0x23DC, 0x4A38, {0x96, 0xFB, 0x7A, 0xDE, 0xD0, 0x80, 0x51, 0x6A}};
static GUID gTextInEx = {0xDD9E7534, 0x7762, 0x4698, {0x8C, 0x14, 0xF5, 0x85, 0x17, 0xA6, 0x25, 0xAA}};
static GUID gPmicLdo = {0x22D38D3D, 0xE8B6, 0x4F8F, {0x9C, 0x26, 0xBC, 0xEB, 0x07, 0xD6, 0xCB, 0x68}};
static GUID gCmdDb = {0x1C34F691, 0xD33D, 0x4E14, {0xB8, 0xF8, 0x32, 0xC6, 0xC0, 0x29, 0xE9, 0x5B}};
static GUID gRpmh = {0xE1D0B573, 0xF29F, 0x495D, {0x93, 0x52, 0xCA, 0x0D, 0xD4, 0x2A, 0x0A, 0x74}};

// Stock PmicDxe LDO protocol (revision 0x10005), layout recovered from PmicDxe.efi.
typedef struct {
  UINT64 Revision;
  EFI_STATUS (*SwEnable)(UINT32 Pmic, UINT32 Ldo, UINT8 Enable);
  EFI_STATUS (*SetMv)(UINT32 Pmic, UINT32 Ldo, UINT32 Mv);
  EFI_STATUS (*GetLevel)(UINT32 Pmic, UINT32 Ldo, UINT32 *Level);
  EFI_STATUS (*Fn3)(UINT32 Pmic, UINT32 Ldo, UINT32 X);
  EFI_STATUS (*Unsupported)(VOID);
  EFI_STATUS (*GetStatus)(UINT32 Pmic, UINT32 Ldo, VOID *Out);
  EFI_STATUS (*SetUv)(UINT32 Pmic, UINT32 Ldo, UINT32 Uv);
} LDOP;

// CmdDbDxe protocol: Revision + 7 functions, each takes This first. fn[1] = QueryAddr(This, ResId[8], &Addr).
typedef struct { UINT64 Revision; VOID *fn[7]; } CMDDB;
// RpmhDxe protocol: Revision + 24 functions that do NOT take This.
//   fn[5] CreateHandle(DrvId, Name, &Handle)   fn[14] IssueCommandSet(Handle, Set, &Req)   fn[22] Wait(Handle, Req)
typedef struct { UINT64 Revision; VOID *fn[24]; } RPMH;
typedef struct { UINT32 Addr, Data, Wait; } RCMD;
typedef struct { UINT32 State, Count, Result; RCMD Cmd[16]; } RSET;

// ---- tiny runtime ---------------------------------------------------------
VOID *memset(VOID *d, int c, UINTN n) { UINT8 *p = d; while (n--) *p++ = (UINT8)c; return d; }
VOID *memcpy(VOID *d, const VOID *s, UINTN n) { UINT8 *p = d; const UINT8 *q = s; while (n--) *p++ = *q++; return d; }

static SYSTAB *ST;
static FILEP *gLog;
static int g_quiet;   // set once the driver goes resident: no more logging / console output

static void out_raw(const char *s, UINTN n) {
  if (gLog) {
    UINTN len = n;
    gLog->Write(gLog, &len, (VOID *)s);
    gLog->Flush(gLog);
  }
  CHAR16 w[2 * 128 + 4];
  UINTN i = 0, j;
  while (i < n) {
    for (j = 0; i < n && j < 2 * 128; i++) {
      if (s[i] == '\n') w[j++] = '\r';
      w[j++] = (CHAR16)(UINT8)s[i];
    }
    w[j] = 0;
    ST->ConOut->OutputString(ST->ConOut, w);
  }
}

static void pr(const char *fmt, ...) {
  if (g_quiet) return;
  char b[256];
  UINTN n = 0;
  va_list ap;
  va_start(ap, fmt);
  for (; *fmt && n < sizeof(b) - 24; fmt++) {
    if (*fmt != '%') { b[n++] = *fmt; continue; }
    fmt++;
    int zero = 0, width = 0, lng = 0;
    if (*fmt == '0') { zero = 1; fmt++; }
    while (*fmt >= '0' && *fmt <= '9') width = width * 10 + (*fmt++ - '0');
    if (*fmt == 'l') { lng = 1; fmt++; }
    if (*fmt == 's') {
      const char *s = va_arg(ap, const char *);
      while (*s && n < sizeof(b) - 24) b[n++] = *s++;
    } else if (*fmt == 'c') {
      b[n++] = (char)va_arg(ap, int);
    } else if (*fmt == 'x' || *fmt == 'd') {
      UINT64 v;
      int neg = 0;
      if (*fmt == 'd') {
        long long sv = lng ? va_arg(ap, long long) : va_arg(ap, int);
        if (sv < 0) { neg = 1; sv = -sv; }
        v = (UINT64)sv;
      } else {
        v = lng ? va_arg(ap, UINT64) : va_arg(ap, UINT32);
      }
      char t[24];
      int k = 0, base = (*fmt == 'x') ? 16 : 10;
      do { int d = (int)(v % base); t[k++] = (char)(d < 10 ? '0' + d : 'a' + d - 10); v /= base; } while (v);
      if (neg) b[n++] = '-';
      for (int p = k; p < width; p++) b[n++] = zero ? '0' : ' ';
      while (k) b[n++] = t[--k];
    } else {
      b[n++] = *fmt;
    }
  }
  va_end(ap);
  out_raw(b, n);
}

static void ms(UINTN n) { while (n--) BS_STALL(1000); }

static UINT64 g_freq;
static UINT64 now_ms(void) {
  UINT64 v;
  __asm__ volatile("isb\n\tmrs %0, cntvct_el0" : "=r"(v));
  return v / (g_freq / 1000);
}

// ---- TLMM (raw MMIO, layout from pinctrl-sm8550.c) ------------------------
#define TLMM_BASE 0x0F100000ULL
#define GPIO_CFG(n) (TLMM_BASE + 0x1000ULL * (n))
#define GPIO_IO(n) (TLMM_BASE + 0x1000ULL * (n) + 4)
static UINT32 rd(UINT64 a) { return *(volatile UINT32 *)a; }
static void wr(UINT64 a, UINT32 v) { *(volatile UINT32 *)a = v; }

static void gpio_dump(int n, const char *name) {
  UINT32 c = rd(GPIO_CFG(n)), io = rd(GPIO_IO(n));
  pr("  gpio%d %s cfg=%08x io=%08x func=%d pull=%d drv=%d oe=%d in=%d out=%d\n", n, name, c, io,
     (c >> 2) & 0xF, c & 3, (c >> 6) & 7, (c >> 9) & 1, io & 1, (io >> 1) & 1);
}
static void gpio_out(int n, int val) {
  wr(GPIO_IO(n), val ? 2 : 0);
  UINT32 c = rd(GPIO_CFG(n));
  c &= ~(0xFu << 2);
  c |= (1u << 9);
  wr(GPIO_CFG(n), c);
}
static void gpio_in_pullup(int n) {
  UINT32 c = rd(GPIO_CFG(n));
  c &= ~((0xFu << 2) | (1u << 9) | 3u);
  c |= 3u;
  wr(GPIO_CFG(n), c);
}
static int gpio_get(int n) { return rd(GPIO_IO(n)) & 1; }

// ---- GCC / QUP ------------------------------------------------------------
#define GCC_BASE 0x00100000ULL
#define GCC(off) (GCC_BASE + (off))
#define VOTE_REG 0x52008
#define SE4_BASE 0x00A90000ULL
#define QUP1_BASE 0x00AC0000ULL
#define TCM_ADDR 0x4B

static UINT32 se_rd(UINT32 off) { return rd(SE4_BASE + off); }
static void se_wr(UINT32 off, UINT32 v) { wr(SE4_BASE + off, v); }

typedef struct { const char *name; UINT64 halt; UINT32 bit; } BR;
static const BR brs[] = {
  {"core   ", 0x23284, 19}, {"core_2x", 0x23294, 18}, {"m_ahb  ", 0x2327c, 20},
  {"s_ahb  ", 0x23280, 21}, {"s4 clk ", 0x184e4, 26},
};

static int clocks_ok(void) {
  int ok = 1;
  for (int i = 0; i < 5; i++) {
    int t;
    for (t = 0; t < 200 && (rd(GCC(brs[i].halt)) & 0x80000000u); t++) {
      if (rd(GCC(brs[i].halt)) & 2) break;   // hw-gated branch: will not leave OFF while idle
      BS_STALL(100);
    }
    UINT32 h = rd(GCC(brs[i].halt));
    int hw = (h & 2) != 0;
    pr("  %s halt=%08x %s\n", brs[i].name, h,
       (h & 0x80000000u) ? (hw ? "off but hw-gated (wakes on access, same as Linux)" : "STILL OFF") : "on");
    if ((h & 0x80000000u) && !hw) ok = 0;
  }
  return ok;
}

static void se_irq_clear(void) {
  se_wr(0xe18, 0);
  se_wr(0x618, 0xFFFFFFFFu);
  se_wr(0x648, 0xFFFFFFFFu);
  se_wr(0xc44, 0xFFFFFFFFu);
  se_wr(0xd44, 0xFFFFFFFFu);
  se_wr(0xe1c, 0xFFFFFFFFu);
}

static void se_setup(UINT32 tx_depth) {
  se_irq_clear();
  se_wr(0x28, se_rd(0x28) | 0x7F);
  se_wr(0xe30, se_rd(0xe30) | 0xF);
  se_wr(0x24, 0x7F);
  se_wr(0x20, 1);
  se_wr(0xe1c, se_rd(0xe1c) | 0xF);
  se_wr(0x258, se_rd(0x258) & ~1u);
  se_wr(0xe18, 0);
  se_wr(0x810, tx_depth - 1);
  se_wr(0x814, tx_depth);
  se_wr(0x614, se_rd(0x614) | 0x33C0007Eu);
  se_wr(0x260, 0x7F8FE); se_wr(0x264, 0xFFEFE);
  se_wr(0x284, 0x7F8FE); se_wr(0x288, 0xFFEFE);
  se_wr(0x254, 0);
  se_wr(0x7c, 0);
  se_wr(0x48, (2u << 4) | 1u);
  se_wr(0x278, (5u << 20) | (11u << 10) | 22u);   // 400 kHz
}

static int i2c_read(UINT8 addr, UINT8 *buf, UINT32 len, UINT32 *seen) {
  UINT32 g = se_rd(0x40);
  if (g & 1) { pr("  GENI cmd already active (status=%08x), not starting\n", g); return -1; }
  se_irq_clear();
  se_wr(0x614, se_rd(0x614) | 1u | (1u << 30) | (1u << 26) | (1u << 27));
  se_wr(0x258, se_rd(0x258) & ~1u);
  se_wr(0x270, len);
  se_wr(0x600, (2u << 27) | ((UINT32)addr << 9));
  UINT32 got = 0, all = 0;
  int done = 0, t;
  for (t = 0; t < 1500 && !done; t++) {          // ~30 ms worst case, a 256 B read needs ~6 ms
    UINT32 m = se_rd(0x610);
    if (m & ((1u << 26) | (1u << 27))) {
      UINT32 rs = se_rd(0x804);
      UINT32 words = rs & 0x1FFFFFF;
      UINT32 valid = (rs >> 28) & 7;
      for (UINT32 w = 0; w < words; w++) {
        UINT32 v = se_rd(0x780);
        UINT32 nb = 4;
        if ((rs >> 31) && w == words - 1 && valid) nb = valid;
        for (UINT32 b = 0; b < nb && got < len; b++) buf[got++] = (UINT8)(v >> (8 * b));
      }
    }
    if (m) se_wr(0x618, m);
    all |= m;
    if (m & 1u) done = 1;
    if (m & ((1u << 1) | (1u << 2) | (1u << 3) | (1u << 10) | (1u << 12) | (1u << 13))) done = 1;
    if (!done) BS_STALL(20);
  }
  if (!done) {
    se_wr(0x604, 1u << 2);
    for (t = 0; t < 100 && !(se_rd(0x610) & (1u << 4)); t++) BS_STALL(100);
    if (!(se_rd(0x610) & (1u << 4))) { se_wr(0x604, 1u << 1); BS_STALL(5000); }
    se_wr(0x618, 0xFFFFFFFFu);
  }
  *seen = all;
  pr("  m_irq_status seen=%08x bytes=%d geni_status=%08x ios=%08x\n", all, got, se_rd(0x40), se_rd(0x908));
  if (all & (1u << 10)) return -2;   // NACK
  if (!done) return -3;
  if (all & ((1u << 1) | (1u << 2) | (1u << 3) | (1u << 12) | (1u << 13))) return -4;
  return (got == len) ? 0 : -5;
}

static int i2c_write(UINT8 addr, const UINT8 *data, UINT32 len) {
  if (se_rd(0x40) & 1) { pr("  GENI cmd already active, not writing\n"); return -1; }
  se_irq_clear();
  se_wr(0x614, se_rd(0x614) | 1u | (1u << 30) | (1u << 26) | (1u << 27));
  se_wr(0x258, se_rd(0x258) & ~1u);
  se_wr(0x26c, len);
  se_wr(0x600, (1u << 27) | ((UINT32)addr << 9));
  se_wr(0x80c, 1);
  UINT32 sent = 0, all = 0;
  int done = 0;
  for (int t = 0; t < 2000 && !done; t++) {
    UINT32 m = se_rd(0x610);
    if ((m & (1u << 30)) && sent < len) {
      while (sent < len) {
        UINT32 w = 0;
        for (UINT32 b = 0; b < 4 && sent + b < len; b++) w |= (UINT32)data[sent + b] << (8 * b);
        se_wr(0x700, w);
        sent += 4;
      }
      se_wr(0x80c, 0);
    }
    if (m) se_wr(0x618, m);
    all |= m;
    if (m & 1u) done = 1;
    if (m & ((1u << 1) | (1u << 2) | (1u << 3) | (1u << 10) | (1u << 12) | (1u << 13))) done = 1;
    if (!done) BS_STALL(20);
  }
  if (!done) { se_wr(0x604, 1u << 2); BS_STALL(5000); se_wr(0x618, 0xFFFFFFFFu); return -3; }
  if (all & (1u << 10)) return -2;
  if (all & ((1u << 1) | (1u << 2) | (1u << 3) | (1u << 12) | (1u << 13))) return -4;
  return 0;
}

static int wait_int(int ms_max) {
  for (int t = 0; t < ms_max; t++) {
    if (!gpio_get(25)) return 1;
    BS_STALL(1000);
  }
  return 0;
}

static int tcm_cmd(UINT8 cmd) { return i2c_write(TCM_ADDR, &cmd, 1); }

static UINT8 g_msg[256];
static int tcm_msg(UINT8 *buf) {   // one message: Linux reads 256 bytes per report
  UINT32 seen = 0;
  memset(buf, 0, MSG_RD_LEN);
  return i2c_read(TCM_ADDR, buf, MSG_RD_LEN, &seen);
}

// ---- hardware bring-up / teardown (same sequence as the working probe) ----
static UINT32 g_vote0, g_cfg44, g_cfg45, g_added;
static int g_hw_started;

static void hw_restore(void) {
  pr("\n[restore] clocks and pins\n");
  wr(GCC(VOTE_REG), rd(GCC(VOTE_REG)) & ~g_added);
  wr(GPIO_CFG(44), g_cfg44);
  wr(GPIO_CFG(45), g_cfg45);
  pr("  vote reg now %08x\n", rd(GCC(VOTE_REG)));
}

static int hw_bringup(void) {
  int rc;
  g_vote0 = rd(GCC(VOTE_REG));
  g_cfg44 = rd(GPIO_CFG(44));
  g_cfg45 = rd(GPIO_CFG(45));
  g_added = 0;

  pr("\n[0] vcc rail: vreg_l4b_1p8 on via RPMh\n");
  {
    CMDDB *cd = 0;
    RPMH *rp = 0;
    LDOP *ld = 0;
    BS_LOCATE_PROTOCOL(&gPmicLdo, 0, (VOID **)&ld);
    EFI_STATUS s1 = BS_LOCATE_PROTOCOL(&gCmdDb, 0, (VOID **)&cd);
    EFI_STATUS s2 = BS_LOCATE_PROTOCOL(&gRpmh, 0, (VOID **)&rp);
    UINT32 base = 0;
    if (!s1 && cd) {
      char idb[8] = {'l', 'd', 'o', 'b', '4', 0, 0, 0};
      EFI_STATUS rq = ((EFI_STATUS(*)(VOID *, const char *, UINT32 *))cd->fn[1])(cd, idb, &base);
      pr("  CmdDb ldob4 -> status=0x%lx addr=0x%x\n", rq, base);
      if (rq) base = 0;
    }
    if (ld) {
      UINT32 o[5];
      memset(o, 0, sizeof(o));
      if (!ld->GetStatus(1, 3, o)) pr("  L4B before: enabled=%d ready=%d\n", o[3], o[4] & 0xFF);
    }
    if (base && !s2 && rp) {
      UINT64 h = 0;
      EFI_STATUS r1 = ((EFI_STATUS(*)(UINT32, const char *, UINT64 *))rp->fn[5])(2, "tcm", &h);
      pr("  CreateHandle(drv 2) -> 0x%lx\n", r1);
      if (!r1 && h) {
        RSET set;
        memset(&set, 0, sizeof(set));
        set.Count = 2;
        set.Cmd[0].Addr = base;     set.Cmd[0].Data = 1800; set.Cmd[0].Wait = 1;
        set.Cmd[1].Addr = base + 4; set.Cmd[1].Data = 1;    set.Cmd[1].Wait = 1;
        UINT32 req = 0;
        r1 = ((EFI_STATUS(*)(UINT64, VOID *, UINT32 *))rp->fn[14])(h, &set, &req);
        pr("  IssueCommandSet -> 0x%lx req=%d result=%d\n", r1, req, set.Result);
        if (!r1 && req) ((EFI_STATUS(*)(UINT64, UINT32))rp->fn[22])(h, req);
      }
    } else {
      pr("  cannot send RPMh request (ldob4 address or protocol missing)\n");
    }
    ms(10);
    if (ld) {
      UINT32 o[5];
      memset(o, 0, sizeof(o));
      if (!ld->GetStatus(1, 3, o)) pr("  L4B after:  enabled=%d ready=%d\n", o[3], o[4] & 0xFF);
    }
  }

  pr("\n[1] touch power: VDD high, reset low->high\n");
  gpio_out(10, 1);
  ms(20);
  gpio_out(24, 0);
  ms(20);
  gpio_in_pullup(25);
  gpio_out(24, 1);
  int t;
  for (t = 0; t < 100 && gpio_get(25); t++) ms(10);
  pr("  INT after %d ms: %d (0 = controller has a message)\n", t * 10, gpio_get(25));
  ms(300);

  pr("\n[2] enable wrapper-1 / SE4 clocks (votes)\n");
  UINT32 want = (1u << 18) | (1u << 19) | (1u << 20) | (1u << 21) | (1u << 26);
  g_added = want & ~g_vote0;
  wr(GCC(VOTE_REG), g_vote0 | want);
  pr("  vote reg %08x -> %08x\n", g_vote0, rd(GCC(VOTE_REG)));
  if (!clocks_ok()) { pr("  clocks did not come up\n"); return -1; }

  pr("\n[3] pin mux gpio44/45 -> qup1_se4 (function 1), pull-up, 2 mA\n");
  wr(GPIO_CFG(44), (g_cfg44 & ~((0xFu << 2) | 3u | (7u << 6))) | (1u << 2) | 3u);
  wr(GPIO_CFG(45), (g_cfg45 & ~((0xFu << 2) | 3u | (7u << 6))) | (1u << 2) | 3u);
  gpio_dump(44, "sda");
  gpio_dump(45, "scl");

  pr("\n[4] engine state\n");
  UINT32 hwver = rd(QUP1_BASE + 4);
  UINT32 fw = se_rd(0x68), ifd = se_rd(0x64), hp0 = se_rd(0xe24);
  UINT32 maj = hwver >> 28, mnr = (hwver >> 16) & 0xFFF;
  UINT32 depth = (hp0 >> 16) & ((maj == 3 && mnr >= 10) || maj > 3 ? 0xFFu : 0x3Fu);
  pr("  QUPV3_HW_VER=%08x (%d.%d)  FW_REVISION=%08x  IF_DISABLE=%08x  HW_PARAM_0=%08x\n", hwver, maj, mnr, fw, ifd, hp0);
  pr("  protocol=%d (3=I2C) fifo_if_disabled=%d tx_fifo_depth=%d\n", (fw >> 8) & 0xFF, ifd & 1, depth);
  if (((fw >> 8) & 0xFF) != 3) { pr("  engine has no I2C firmware loaded\n"); return -2; }
  if (ifd & 1) { pr("  FIFO interface disabled (GPI-only)\n"); return -3; }
  if (!depth) { pr("  zero FIFO depth\n"); return -4; }

  pr("\n[5] configure engine, read the pending IDENTIFY message\n");
  se_setup(depth);
  rc = tcm_msg(g_msg);
  pr("  read rc=%d: marker=%02x code=%02x len=%d mode=%d part=%c%c%c%c%c\n", rc, g_msg[0], g_msg[1],
     g_msg[2] | (g_msg[3] << 8), g_msg[5], g_msg[6], g_msg[7], g_msg[8], g_msg[9], g_msg[10]);
  if (rc || g_msg[0] != 0xA5) { pr("  no valid IDENTIFY\n"); return -5; }

  pr("\n[6] GET_APPLICATION_INFO (0x20)\n");
  rc = tcm_cmd(0x20);
  pr("  send rc=%d, INT low: %d\n", rc, wait_int(500));
  rc = tcm_msg(g_msg);
  pr("  read rc=%d code=%02x len=%d\n", rc, g_msg[1], g_msg[2] | (g_msg[3] << 8));

  pr("\n[7] ENABLE_REPORT (0x05)\n");
  rc = tcm_cmd(0x05);
  pr("  send rc=%d\n", rc);
  g_hw_started = (rc == 0);
  return rc ? -6 : 0;
}

static void hw_stop(void) {
  if (g_hw_started) { tcm_cmd(0x06); BS_STALL(5000); g_hw_started = 0; }   // DISABLE_REPORT
  hw_restore();
}

// ---- font (5x7, columns, bit0 = top row; ASCII 0x20..0x5F, lowercase is shown as upper) ----
static const UINT8 font5x7[64][5] = {
  {0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x5F, 0x00, 0x00},
  {0x00, 0x07, 0x00, 0x07, 0x00},
  {0x14, 0x7F, 0x14, 0x7F, 0x14},
  {0x24, 0x2A, 0x7F, 0x2A, 0x12},
  {0x23, 0x13, 0x08, 0x64, 0x62},
  {0x36, 0x49, 0x56, 0x20, 0x50},
  {0x00, 0x08, 0x07, 0x03, 0x00},
  {0x00, 0x1C, 0x22, 0x41, 0x00},
  {0x00, 0x41, 0x22, 0x1C, 0x00},
  {0x2A, 0x1C, 0x7F, 0x1C, 0x2A},
  {0x08, 0x08, 0x3E, 0x08, 0x08},
  {0x00, 0x80, 0x70, 0x30, 0x00},
  {0x08, 0x08, 0x08, 0x08, 0x08},
  {0x00, 0x00, 0x60, 0x60, 0x00},
  {0x20, 0x10, 0x08, 0x04, 0x02},
  {0x3E, 0x51, 0x49, 0x45, 0x3E},
  {0x00, 0x42, 0x7F, 0x40, 0x00},
  {0x72, 0x49, 0x49, 0x49, 0x46},
  {0x21, 0x41, 0x49, 0x4D, 0x33},
  {0x18, 0x14, 0x12, 0x7F, 0x10},
  {0x27, 0x45, 0x45, 0x45, 0x39},
  {0x3C, 0x4A, 0x49, 0x49, 0x31},
  {0x41, 0x21, 0x11, 0x09, 0x07},
  {0x36, 0x49, 0x49, 0x49, 0x36},
  {0x46, 0x49, 0x49, 0x29, 0x1E},
  {0x00, 0x00, 0x14, 0x00, 0x00},
  {0x00, 0x40, 0x34, 0x00, 0x00},
  {0x00, 0x08, 0x14, 0x22, 0x41},
  {0x14, 0x14, 0x14, 0x14, 0x14},
  {0x00, 0x41, 0x22, 0x14, 0x08},
  {0x02, 0x01, 0x59, 0x09, 0x06},
  {0x3E, 0x41, 0x5D, 0x59, 0x4E},
  {0x7C, 0x12, 0x11, 0x12, 0x7C},
  {0x7F, 0x49, 0x49, 0x49, 0x36},
  {0x3E, 0x41, 0x41, 0x41, 0x22},
  {0x7F, 0x41, 0x41, 0x22, 0x1C},
  {0x7F, 0x49, 0x49, 0x49, 0x41},
  {0x7F, 0x09, 0x09, 0x09, 0x01},
  {0x3E, 0x41, 0x41, 0x51, 0x73},
  {0x7F, 0x08, 0x08, 0x08, 0x7F},
  {0x00, 0x41, 0x7F, 0x41, 0x00},
  {0x20, 0x40, 0x41, 0x3F, 0x01},
  {0x7F, 0x08, 0x14, 0x22, 0x41},
  {0x7F, 0x40, 0x40, 0x40, 0x40},
  {0x7F, 0x02, 0x1C, 0x02, 0x7F},
  {0x7F, 0x04, 0x08, 0x10, 0x7F},
  {0x3E, 0x41, 0x41, 0x41, 0x3E},
  {0x7F, 0x09, 0x09, 0x09, 0x06},
  {0x3E, 0x41, 0x51, 0x21, 0x5E},
  {0x7F, 0x09, 0x19, 0x29, 0x46},
  {0x26, 0x49, 0x49, 0x49, 0x32},
  {0x03, 0x01, 0x7F, 0x01, 0x03},
  {0x3F, 0x40, 0x40, 0x40, 0x3F},
  {0x1F, 0x20, 0x40, 0x20, 0x1F},
  {0x3F, 0x40, 0x38, 0x40, 0x3F},
  {0x63, 0x14, 0x08, 0x14, 0x63},
  {0x03, 0x04, 0x78, 0x04, 0x03},
  {0x61, 0x59, 0x49, 0x4D, 0x43},
  {0x00, 0x7F, 0x41, 0x41, 0x00},
  {0x02, 0x04, 0x08, 0x10, 0x20},
  {0x00, 0x41, 0x41, 0x7F, 0x00},
  {0x04, 0x02, 0x01, 0x02, 0x04},
  {0x40, 0x40, 0x40, 0x40, 0x40}
};

// ---- keyboard model -------------------------------------------------------
enum { K_CHAR, K_SCAN, K_UNI, K_SHIFT, K_CTRL, K_HIDE };
typedef struct { const char *lbl; UINT8 type; UINT8 rep; UINT16 val; UINT8 w2; } KEYDEF;   // w2 = width in half units
#define C(c) {0, K_CHAR, 0, (c), 2}

static const KEYDEF row0[] = {
  {"ESC", K_SCAN, 0, 0x17, 2}, {"TAB", K_UNI, 0, 9, 2}, {"CTRL", K_CTRL, 0, 0, 2}, {"DEL", K_SCAN, 1, 8, 2},
  {"@L", K_SCAN, 1, 4, 2}, {"@U", K_SCAN, 1, 1, 2}, {"@D", K_SCAN, 1, 2, 2}, {"@R", K_SCAN, 1, 3, 2},
  {"HOME", K_SCAN, 0, 5, 2}, {"HIDE", K_HIDE, 0, 0, 2}};
static const KEYDEF row1[] = {C('1'), C('2'), C('3'), C('4'), C('5'), C('6'), C('7'), C('8'), C('9'), C('0')};
static const KEYDEF row2[] = {C('q'), C('w'), C('e'), C('r'), C('t'), C('y'), C('u'), C('i'), C('o'), C('p')};
static const KEYDEF row3[] = {C('a'), C('s'), C('d'), C('f'), C('g'), C('h'), C('j'), C('k'), C('l'),
                              {"ENT", K_UNI, 0, 0x0D, 2}};
static const KEYDEF row4[] = {{"SHIFT", K_SHIFT, 0, 0, 3}, C('z'), C('x'), C('c'), C('v'), C('b'), C('n'), C('m'),
                              {"BKSP", K_UNI, 1, 0x08, 3}};
static const KEYDEF row5[] = {C('-'), C('='), C(';'), C('\''), {"SPACE", K_UNI, 0, ' ', 6}, C(','), C('.'), C('/')};

#define NROWS 6
static const KEYDEF *const rows[NROWS] = {row0, row1, row2, row3, row4, row5};
static const UINT8 rowlen[NROWS] = {10, 10, 10, 10, 9, 8};
#define MAXK 64

typedef struct { INT32 x, y, w, h; } KRECT;
static const KEYDEF *g_kd[MAXK];
static KRECT g_r[MAXK];
static int g_nk;
static KRECT g_tog;   // "KB" button shown while the keyboard is hidden

static UINT16 shift_of(UINT16 c) {
  if (c >= 'a' && c <= 'z') return c - 32;
  switch (c) {
    case '1': return '!'; case '2': return '@'; case '3': return '#'; case '4': return '$'; case '5': return '%';
    case '6': return '^'; case '7': return '&'; case '8': return '*'; case '9': return '('; case '0': return ')';
    case '-': return '_'; case '=': return '+'; case ';': return ':'; case '\'': return '"';
    case ',': return '<'; case '.': return '>'; case '/': return '?';
  }
  return c;
}

// ---- key queue (producer: timer tick at TPL_CALLBACK, consumer: hooks, under TPL_NOTIFY) ----
typedef struct { UINT16 scan, uni; UINT32 shift; } QK;
#define QN 32
static QK g_q[QN];
static volatile UINT32 g_qh, g_qt;

static void q_push(UINT16 scan, UINT16 uni, UINT32 shift) {
  if (g_qh - g_qt >= QN) return;
  QK *k = &g_q[g_qh % QN];
  k->scan = scan; k->uni = uni; k->shift = shift;
  g_qh++;
}
static int q_pop(QK *o) {
  UINTN old = BS_RAISE_TPL(TPL_NOTIFY);
  int r = 0;
  if (g_qh != g_qt) { *o = g_q[g_qt % QN]; g_qt++; r = 1; }
  BS_RESTORE_TPL(old);
  return r;
}

// ---- state shared between tick, hooks and exit handler ----
static volatile int g_in_tick, g_stopped, g_busy, g_dead;
static int g_err;
static int g_vis = KB_START_VISIBLE;
static int g_shift, g_ctrl;
static UINT64 g_pressed, g_dirty_keys;
static int g_dirty_all = 1, g_need_clear, g_con_dirty;
static UINT64 g_last_full;

// ---- GOP / render buffer ----
static GOP *g_gop;
static UINT32 g_W, g_H, g_kbh, g_y0;
static UINT32 *g_kbuf;   // g_W x g_kbh, BGRA

#define COL_BG 0x00101010u
#define COL_KEY 0x002E2E32u
#define COL_SPECIAL 0x0024304Au
#define COL_ON 0x004A7DFFu
#define COL_LATCH 0x00D08A00u
#define COL_TEXT 0x00F0F0F0u
#define GAP 4

static void fill(INT32 x, INT32 y, INT32 w, INT32 h, UINT32 c) {
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > (INT32)g_W) w = g_W - x;
  if (y + h > (INT32)g_kbh) h = g_kbh - y;
  for (INT32 j = 0; j < h; j++) {
    UINT32 *p = g_kbuf + (UINT64)(y + j) * g_W + x;
    for (INT32 i = 0; i < w; i++) p[i] = c;
  }
}

static const UINT8 *glyph(int c) {
  if (c >= 'a' && c <= 'z') c -= 32;
  if (c < 32 || c > 95) c = 32;
  return font5x7[c - 32];
}

static void draw_text(const char *s, const KRECT *r, UINT32 col) {
  int len = 0;
  while (s[len]) len++;
  int sc = (r->h * 4 / 10) / 7;
  while (sc > 1 && (len * 6 - 1) * sc > r->w * 8 / 10) sc--;
  if (sc < 1) sc = 1;
  int tw = (len * 6 - 1) * sc, th = 7 * sc;
  int x0 = r->x + (r->w - tw) / 2, y0 = r->y + (r->h - th) / 2;
  for (int ci = 0; ci < len; ci++) {
    const UINT8 *g = glyph(s[ci]);
    for (int cx = 0; cx < 5; cx++)
      for (int cy = 0; cy < 8; cy++)   // 8th row is the descender of , ; and friends
        if ((g[cx] >> cy) & 1) fill(x0 + (ci * 6 + cx) * sc, y0 + cy * sc, sc, sc, col);
  }
}

static INT32 iabs(INT32 v) { return v < 0 ? -v : v; }

static void draw_arrow(char dir, const KRECT *r, UINT32 col) {
  INT32 s = (r->w < r->h ? r->w : r->h) / 5;
  INT32 cx = r->x + r->w / 2, cy = r->y + r->h / 2;
  for (INT32 b = -s; b <= s; b++)
    for (INT32 a = -s; a <= s; a++) {
      int in = 0;
      switch (dir) {
        case 'L': in = iabs(b) <= (a + s) / 2; break;
        case 'R': in = iabs(b) <= (s - a) / 2; break;
        case 'U': in = iabs(a) <= (b + s) / 2; break;
        case 'D': in = iabs(a) <= (s - b) / 2; break;
      }
      if (in) fill(cx + a, cy + b, 1, 1, col);
    }
}

static void draw_keycap(const KRECT *r, UINT32 bg, const char *lbl) {
  fill(r->x + GAP, r->y + GAP, r->w - 2 * GAP, r->h - 2 * GAP, bg);
  if (lbl[0] == '@') draw_arrow(lbl[1], r, COL_TEXT);
  else draw_text(lbl, r, COL_TEXT);
}

static void draw_key(int i) {
  const KEYDEF *k = g_kd[i];
  UINT32 bg = (k->type == K_CHAR) ? COL_KEY : COL_SPECIAL;
  if (g_pressed & (1ULL << i)) bg = COL_ON;
  if ((k->type == K_SHIFT && g_shift) || (k->type == K_CTRL && g_ctrl)) bg = COL_LATCH;
  char tmp[2] = {0, 0};
  const char *lbl = k->lbl;
  if (k->type == K_CHAR) { tmp[0] = (char)(g_shift ? shift_of(k->val) : k->val); lbl = tmp; }
  draw_keycap(&g_r[i], bg, lbl);
}

static void draw_toggle(void) { draw_keycap(&g_tog, COL_SPECIAL, "KB"); }

static void blit(INT32 x, INT32 y, INT32 w, INT32 h) {
  g_gop->Blt(g_gop, g_kbuf, BLT_BUFFER_TO_VIDEO, x, y, x, g_y0 + y, w, h, (UINTN)g_W * 4);
}

static void service_draw(UINT64 now) {
  if (g_busy) return;   // a ConOut call is in flight; never Blt concurrently with the text console
  if (g_con_dirty && now - g_last_full >= 50) { g_dirty_all = 1; g_con_dirty = 0; g_last_full = now; }
  if (g_need_clear) {
    UINT32 black = 0;
    g_gop->Blt(g_gop, &black, BLT_VIDEO_FILL, 0, 0, 0, g_y0, g_W, g_kbh, 0);
    g_need_clear = 0;
    g_dirty_all = 1;
  }
  if (g_vis) {
    if (g_dirty_all) {
      fill(0, 0, g_W, g_kbh, COL_BG);
      for (int i = 0; i < g_nk; i++) draw_key(i);
      blit(0, 0, g_W, g_kbh);
      g_dirty_all = 0;
      g_dirty_keys = 0;
    } else if (g_dirty_keys) {
      UINT64 m = g_dirty_keys;
      g_dirty_keys = 0;
      for (int i = 0; i < g_nk; i++)
        if ((m >> i) & 1) { draw_key(i); blit(g_r[i].x, g_r[i].y, g_r[i].w, g_r[i].h); }
    }
  } else if (g_dirty_all) {
    draw_toggle();
    blit(g_tog.x, g_tog.y, g_tog.w, g_tog.h);
    g_dirty_all = 0;
  }
}

static void kb_hide(void) { g_vis = 0; g_need_clear = 1; g_dirty_all = 1; }
static void kb_show(void) { g_vis = 1; g_dirty_all = 1; }

// ---- key actions ----
static void emit(UINT16 scan, UINT16 uni) {
  UINT32 sh = 0x80000000u;   // EFI_SHIFT_STATE_VALID
  if (g_shift) sh |= 0x02;   // LEFT_SHIFT_PRESSED
  if (g_ctrl) sh |= 0x08;    // LEFT_CONTROL_PRESSED
  q_push(scan, uni, sh);
}

static void key_press(int i) {
  const KEYDEF *k = g_kd[i];
  switch (k->type) {
    case K_SHIFT: g_shift ^= 1; g_dirty_all = 1; return;
    case K_CTRL: g_ctrl ^= 1; g_dirty_keys |= 1ULL << i; return;
    case K_HIDE: kb_hide(); return;
    case K_CHAR: {
      UINT16 c = k->val;
      if (g_shift) c = shift_of(c);
      if (g_ctrl && (c | 32) >= 'a' && (c | 32) <= 'z') c = (UINT16)((c | 32) - 'a' + 1);   // like UsbKbDxe
      emit(0, c);
      break;
    }
    case K_SCAN: emit(k->val, 0); break;
    case K_UNI: emit(0, k->val); break;
  }
  if (g_shift || g_ctrl) { g_shift = g_ctrl = 0; g_dirty_all = 1; }   // modifiers are one-shot
}

// ---- touch -> pixels -> keys ----
typedef struct { int active; int key; UINT64 last, next_rep; } SLOT;
#define NSLOT 16
static SLOT g_slot[NSLOT];

static void map_touch(UINT32 rx, UINT32 ry, INT32 *px, INT32 *py) {
  UINT64 a = rx, b = ry, aw = TOUCH_RAW_W, bw = TOUCH_RAW_H;
#if TOUCH_SWAP_XY
  a = ry; b = rx; aw = TOUCH_RAW_H; bw = TOUCH_RAW_W;
#endif
  if (a >= aw) a = aw - 1;
  if (b >= bw) b = bw - 1;
#if TOUCH_INVERT_X
  a = aw - 1 - a;
#endif
#if TOUCH_INVERT_Y
  b = bw - 1 - b;
#endif
  *px = (INT32)(a * g_W / aw);
  *py = (INT32)(b * g_H / bw);
}

static int in_rect(const KRECT *r, INT32 x, INT32 y) { return x >= r->x && x < r->x + r->w && y >= r->y && y < r->y + r->h; }

static void slot_down(int s, INT32 px, INT32 py, UINT64 now) {
  g_slot[s].key = -1;
  if ((UINT32)py < g_y0) return;
  py -= g_y0;
  if (!g_vis) {
    if (in_rect(&g_tog, px, py)) kb_show();
    return;
  }
  for (int i = 0; i < g_nk; i++) {
    if (!in_rect(&g_r[i], px, py)) continue;
    g_slot[s].key = i;
    g_pressed |= 1ULL << i;
    g_dirty_keys |= 1ULL << i;
    key_press(i);
    g_slot[s].next_rep = now + REPEAT_DELAY_MS;
    return;
  }
}

static void slot_up(int s) {
  if (g_slot[s].key >= 0) {
    g_pressed &= ~(1ULL << g_slot[s].key);
    g_dirty_keys |= 1ULL << g_slot[s].key;
  }
  g_slot[s].active = 0;
  g_slot[s].key = -1;
}

static void handle_msg(UINT64 now) {
  UINT8 *msg = g_msg;
  if (msg[0] != 0xA5 || msg[1] != 0x11) return;   // only touch reports
  UINT32 plen = msg[2] | (msg[3] << 8);
  UINT32 n = plen >= 7 ? (plen - 7) / 12 : 0;
  UINT32 seen = 0;
  for (UINT32 i = 0; i < n && i < 5; i++) {
    UINT8 *p = msg + 4 + 1 + 12 * i;
    UINT32 idx = p[6] & 0xF, x = p[7] | (p[8] << 8), y = p[9] | (p[10] << 8);
    INT32 px, py;
    map_touch(x, y, &px, &py);
    seen |= 1u << idx;
    g_slot[idx].last = now;
    if (!g_slot[idx].active) { g_slot[idx].active = 1; slot_down(idx, px, py, now); }
  }
  for (int s = 0; s < NSLOT; s++)
    if (g_slot[s].active && !((seen >> s) & 1)) slot_up(s);   // absent from the report = lifted
}

static void tick(VOID *ev, VOID *ctx) {
  (void)ev; (void)ctx;
  if (g_stopped || g_in_tick) return;
  g_in_tick = 1;
  UINT64 now = now_ms();
  if (!g_dead && !gpio_get(25)) {
    int rc = tcm_msg(g_msg);
    if (rc) { if (++g_err > 20) g_dead = 1; }
    else { g_err = 0; handle_msg(now); }
  }
  for (int s = 0; s < NSLOT; s++) {
    if (!g_slot[s].active) continue;
    if (now - g_slot[s].last > SLOT_TIMEOUT_MS) { slot_up(s); continue; }
    int k = g_slot[s].key;
    if (k >= 0 && g_kd[k]->rep && now >= g_slot[s].next_rep) {
      key_press(k);
      g_slot[s].next_rep = now + REPEAT_RATE_MS;
    }
  }
  service_draw(now);
  g_in_tick = 0;
}

// ---- console hooks (patched in place into the firmware's existing protocol structs) ----
static CONIN *CI;
static CONINEX *CX;
static CONOUT *CO;
static EFI_STATUS (*o_read)(VOID *, KEY *);
static EFI_STATUS (*o_read_ex)(VOID *, KEYDATA *);
static EFI_STATUS (*o_out)(VOID *, CHAR16 *);
static EFI_STATUS (*o_clear)(VOID *);
static EFI_STATUS (*o_setcur)(VOID *, UINTN, UINTN);
static EFI_STATUS (*o_encur)(VOID *, UINT8);
static EFI_STATUS (*o_setmode)(VOID *, UINTN);
static VOID *o_wait, *o_wait_ex;
static VOID *g_wait_ev, *g_wait_ex_ev, *g_timer_ev, *g_exit_ev;
static UINT32 g_cellh;
static EFI_STATUS (*o_query)(VOID *, UINTN, UINTN *, UINTN *);
static UINT32 g_glyph_h = 19;   // console cell height in pixels (calibrated at install)
static UINT32 g_rows;           // rows of the current text mode

// The text console centres its text block on the screen. Find the cell height from the
// largest text mode (it fills the screen), so we can work out where any mode draws.
static void calibrate_glyph(void) {
  UINTN best_rows = 0;
  for (INT32 m = 0; CO->Mode && m < CO->Mode->MaxMode; m++) {
    UINTN c = 0, r = 0;
    if (!o_query(CO, (UINTN)m, &c, &r) && r > best_rows) best_rows = r;
  }
  if (best_rows) {
    UINT32 gh = g_H / (UINT32)best_rows;
    if (gh >= 8 && gh <= 40) g_glyph_h = gh;
  }
}

// A text mode is usable if its (centred) block, plus one row of margin, ends above the keyboard.
static int mode_fits(UINTN rows) {
  UINT64 bh = (UINT64)rows * g_glyph_h;
  UINT64 bottom = ((UINT64)g_H + bh) / 2;
  return bottom + g_glyph_h <= g_y0;
}

// Largest usable text mode (by character count), or -1.
static INT32 best_mode(void) {
  INT32 best = -1;
  UINTN area = 0;
  for (INT32 m = 0; CO->Mode && m < CO->Mode->MaxMode; m++) {
    UINTN c = 0, r = 0;
    if (o_query(CO, (UINTN)m, &c, &r) || !mode_fits(r)) continue;
    if (c * r > area) { area = c * r; best = m; }
  }
  return best;
}

static void update_cell_height(void) {
  UINTN cols = 0, rws = 0;
  g_cellh = g_glyph_h;
  g_rows = 0;
  if (CO->Mode && !o_query(CO, (UINTN)CO->Mode->Mode, &cols, &rws)) g_rows = (UINT32)rws;
}

// Mark the keyboard for redraw if the console may have touched its area.
static void con_touched(void) {
  if (!g_rows || !CO->Mode) { g_con_dirty = 1; return; }
  UINT64 bh = (UINT64)g_rows * g_glyph_h;
  UINT64 top = (g_H > bh) ? (g_H - bh) / 2 : 0;
  UINT64 bottom = top + (UINT64)(CO->Mode->CursorRow + 2) * g_glyph_h;   // one extra row (scrolling keeps the cursor low)
  if (bottom >= g_y0) g_con_dirty = 1;
}

static EFI_STATUS h_out(VOID *t, CHAR16 *s) {
  g_busy++;
  EFI_STATUS r = o_out(t, s);
  con_touched();
  g_busy--;
  return r;
}
static EFI_STATUS h_clear(VOID *t) {
  g_busy++;
  EFI_STATUS r = o_clear(t);
  g_con_dirty = 1;
  g_busy--;
  return r;
}
static EFI_STATUS h_setcur(VOID *t, UINTN c, UINTN r) {
  g_busy++;
  EFI_STATUS s = o_setcur(t, c, r);
  con_touched();
  g_busy--;
  return s;
}
static EFI_STATUS h_encur(VOID *t, UINT8 v) {
  g_busy++;
  EFI_STATUS s = o_encur(t, v);
  con_touched();
  g_busy--;
  return s;
}
// Hide text modes whose block would run under the keyboard (the shell's "mode" command only
// lists modes that QueryMode accepts).
static EFI_STATUS h_query(VOID *t, UINTN m, UINTN *c, UINTN *r) {
  EFI_STATUS s = o_query(t, m, c, r);
  if (!s && r && !mode_fits(*r)) return EFI_UNSUPPORTED;
  return s;
}
static EFI_STATUS h_setmode(VOID *t, UINTN m) {
  g_busy++;
  UINTN c = 0, r = 0;
  if (!o_query(t, m, &c, &r) && !mode_fits(r)) {     // caller wants a mode that would overlap
    INT32 b = best_mode();
    if (b >= 0) m = (UINTN)b;
  }
  EFI_STATUS s = o_setmode(t, m);
  update_cell_height();
  g_con_dirty = 1;
  g_busy--;
  return s;
}

static EFI_STATUS h_read(VOID *t, KEY *k) {
  QK q;
  if (q_pop(&q)) { k->ScanCode = q.scan; k->Unicode = q.uni; return EFI_SUCCESS; }
  return o_read(t, k);
}
static EFI_STATUS h_read_ex(VOID *t, KEYDATA *kd) {
  QK q;
  if (q_pop(&q)) {
    kd->ScanCode = q.scan; kd->Unicode = q.uni; kd->ShiftState = q.shift; kd->ToggleState = 0x80;   // TOGGLE_STATE_VALID
    return EFI_SUCCESS;
  }
  return o_read_ex(t, kd);
}

// WaitForKey replacement: signalled when we have a virtual key, or when the original event is.
static void wait_notify(VOID *ev, VOID *ctx) {
  VOID *orig = *(VOID **)ctx;
  if (g_qh != g_qt) { BS_SIGNAL_EVENT(ev); return; }
  if (orig && BS_CHECK_EVENT(orig) == EFI_SUCCESS) BS_SIGNAL_EVENT(ev);   // key stays in the original queue
}

// ExitBootServices: leave the controller and clocks as we found them.
static void on_exit(VOID *ev, VOID *ctx) {
  (void)ev; (void)ctx;
  g_stopped = 1;
  if (g_in_tick) {                  // we preempted a tick: abort a possibly active GENI command first
    se_wr(0x604, 1u << 2);
    BS_STALL(1000);
    se_wr(0x618, 0xFFFFFFFFu);
  }
  hw_stop();
}

static int install(void) {
  EFI_STATUS s;
  CI = ST->ConIn;
  CO = ST->ConOut;
  CX = 0;
  BS_HANDLE_PROTOCOL(ST->ConInHandle, &gTextInEx, (VOID **)&CX);

  // GOP: prefer the one on the console-out handle
  g_gop = 0;
  s = BS_HANDLE_PROTOCOL(ST->ConOutHandle, &gGopGuid, (VOID **)&g_gop);
  if (s || !g_gop) { g_gop = 0; s = BS_LOCATE_PROTOCOL(&gGopGuid, 0, (VOID **)&g_gop); }
  if (s || !g_gop || !g_gop->Mode || !g_gop->Mode->Info) { pr("  no GOP\n"); return -1; }
  g_W = g_gop->Mode->Info->Hres;
  g_H = g_gop->Mode->Info->Vres;
  UINT32 hu = g_W / 20;
  UINT32 kh = hu * 2;
  if (kh * NROWS > g_H / 3) kh = (g_H / 3) / NROWS;
  g_kbh = kh * NROWS;
  g_y0 = g_H - g_kbh;
  pr("  GOP %dx%d, keyboard %dx%d at y=%d, key %dx%d\n", g_W, g_H, g_W, g_kbh, g_y0, hu * 2, kh);
  if (!hu || !kh) return -2;

  // layout
  UINT32 xoff = (g_W - hu * 20) / 2;
  g_nk = 0;
  for (int r = 0; r < NROWS; r++) {
    UINT32 x = xoff;
    for (int k = 0; k < rowlen[r]; k++) {
      const KEYDEF *kd = &rows[r][k];
      g_kd[g_nk] = kd;
      g_r[g_nk].x = x;
      g_r[g_nk].y = r * kh;
      g_r[g_nk].w = kd->w2 * hu;
      g_r[g_nk].h = kh;
      x += kd->w2 * hu;
      g_nk++;
    }
  }
  g_tog.w = 3 * hu; g_tog.h = kh; g_tog.x = g_W - xoff - g_tog.w; g_tog.y = g_kbh - kh;

  VOID *buf = 0;
  s = BS_ALLOC_POOL(EfiBootServicesData, (UINTN)g_W * g_kbh * 4, &buf);
  if (s || !buf) { pr("  AllocatePool failed\n"); return -3; }
  g_kbuf = buf;

  // events first, patches last (so a failure leaves the firmware untouched)
  o_query = CO->QueryMode;
  calibrate_glyph();
  update_cell_height();
  pr("  console: glyph height %d px, current mode %d (%d rows), keyboard starts at y=%d\n", g_glyph_h, CO->Mode ? CO->Mode->Mode : -1, g_rows, g_y0);
  if (BS_CREATE_EVENT(EVT_NOTIFY_WAIT, TPL_NOTIFY, wait_notify, &o_wait, &g_wait_ev)) return -4;
  if (CX && BS_CREATE_EVENT(EVT_NOTIFY_WAIT, TPL_NOTIFY, wait_notify, &o_wait_ex, &g_wait_ex_ev)) return -5;
  if (BS_CREATE_EVENT(EVT_SIGNAL_EXIT_BOOT_SERVICES, TPL_NOTIFY, on_exit, 0, &g_exit_ev)) return -6;
  if (BS_CREATE_EVENT(EVT_TIMER | EVT_NOTIFY_SIGNAL, TPL_CALLBACK, tick, 0, &g_timer_ev)) return -7;

  o_read = CI->ReadKey;       CI->ReadKey = h_read;
  o_wait = CI->WaitForKey;    CI->WaitForKey = g_wait_ev;
  if (CX) {
    o_read_ex = CX->ReadKeyEx;     CX->ReadKeyEx = h_read_ex;
    o_wait_ex = CX->WaitForKeyEx;  CX->WaitForKeyEx = g_wait_ex_ev;
  }
  o_out = CO->OutputString;          CO->OutputString = h_out;
  o_clear = CO->ClearScreen;         CO->ClearScreen = h_clear;
  o_setcur = CO->SetCursorPosition;  CO->SetCursorPosition = h_setcur;
  o_encur = CO->EnableCursor;        CO->EnableCursor = h_encur;
  o_setmode = CO->SetMode;           CO->SetMode = h_setmode;
  CO->QueryMode = h_query;
  // If the console is currently in a mode that reaches under the keyboard, move it to the best one that fits.
  if (CO->Mode && g_rows && !mode_fits(g_rows)) {
    INT32 bm = best_mode();
    pr("  current text mode overlaps the keyboard; switching to mode %d\n", bm);
    if (bm >= 0) CO->SetMode(CO, (UINTN)bm);
  }
  pr("  hooks installed: ConIn, ConInEx=%s, ConOut\n", CX ? "yes" : "no");

  BS_SET_TIMER(g_timer_ev, 1 /* TimerPeriodic */, TICK_100NS);
  return 0;
}

EFI_STATUS efi_main(EFI_HANDLE image, SYSTAB *st) {
  ST = st;
  BS_WATCHDOG(0, 0, 0, 0);
  __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(g_freq));
  if (!g_freq) g_freq = 19200000;

  VOID *li = 0;
  if (!BS_HANDLE_PROTOCOL(image, &gLoadedImage, &li)) {
    VOID *dev = *(VOID **)((UINT8 *)li + 0x18);
    SFS *sfs = 0;
    FILEP *root = 0, *f = 0;
    if (!BS_HANDLE_PROTOCOL(dev, &gSfs, (VOID **)&sfs) && !sfs->OpenVolume(sfs, &root) &&
        !root->Open(root, (VOID **)&f, (CHAR16 *)u"\\tcmkbd.log", 0x8000000000000003ULL, 0)) {
      f->SetPos(f, ~0ULL);
      gLog = f;
    }
  }
  pr("\n==== TcmKbd: S3908 touch + on-screen keyboard (resident driver) ====\n");
  pr("log file: %s\n", gLog ? "\\tcmkbd.log" : "NOT AVAILABLE");

  EFI_STATUS res = EFI_SUCCESS;
  int rc = hw_bringup();
  if (rc) {
    pr("\nTouch bring-up failed (%d), leaving the console untouched.\n", rc);
    hw_stop();
    res = EFI_DEVICE_ERROR;
  } else {
    pr("\n[8] keyboard + console hooks\n");
    rc = install();
    if (rc) {
      pr("  install failed (%d)\n", rc);
      hw_stop();
      res = EFI_DEVICE_ERROR;
    } else {
      pr("\nResident. Keyboard is %s.\n", g_vis ? "visible" : "hidden (tap KB at the bottom-right)");
    }
  }

  g_quiet = 1;
  if (gLog) { gLog->Close(gLog); gLog = 0; }
  return res;
}
