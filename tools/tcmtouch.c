// SeProbe.efi - powers the S3908, enables the clocks of QUP wrapper 1 / SE4 (i2c4),
// muxes gpio44/45, and does ONE polled GENI I2C read of 8 bytes at 0x4b.
// Every step is gated on the previous one; clocks and pin mux are restored at the end.
// Log: \seprobe.log on the ESP.
//
// Build: clang --target=aarch64-unknown-windows -ffreestanding -fno-builtin
//        -fshort-wchar -fno-stack-protector -O2 -c tcmprobe.c
//        lld-link /subsystem:efi_application /entry:efi_main /nodefaultlib
//        /machine:arm64 /dll /base:0 /align:4096 tcmprobe.obj /out:TcmProbe.efi
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

// ---- minimal EFI tables --------------------------------------------------
typedef struct { UINT16 ScanCode; CHAR16 Unicode; } KEY;
typedef struct {
  EFI_STATUS (*Reset)(VOID *, UINT8);
  EFI_STATUS (*ReadKey)(VOID *, KEY *);
  VOID *WaitForKey;
} CONIN;
typedef struct {
  EFI_STATUS (*Reset)(VOID *, UINT8);
  EFI_STATUS (*OutputString)(VOID *, CHAR16 *);
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

#define BS_SLOT(n) (((VOID **)(ST->BS + 24))[n])
#define BS_HANDLE_PROTOCOL(h, g, i) ((EFI_STATUS(*)(VOID *, GUID *, VOID **))BS_SLOT(16))(h, g, i)
#define BS_LOCATE_PROTOCOL(g, r, i) ((EFI_STATUS(*)(GUID *, VOID *, VOID **))BS_SLOT(37))(g, r, i)
#define BS_STALL(us) ((EFI_STATUS(*)(UINTN))BS_SLOT(28))(us)
#define BS_WATCHDOG(a, b, c, d) ((EFI_STATUS(*)(UINTN, UINT64, UINTN, VOID *))BS_SLOT(29))(a, b, c, d)

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

static GUID gLoadedImage = {0x5B1B31A1, 0x9562, 0x11D2, {0x8E, 0x3F, 0x00, 0xA0, 0xC9, 0x69, 0x72, 0x3B}};
static GUID gSfs = {0x964E5B22, 0x6459, 0x11D2, {0x8E, 0x39, 0x00, 0xA0, 0xC9, 0x69, 0x72, 0x3B}};
static GUID gQcomI2c = {0xB27AE8B1, 0x3E10, 0x4D07, {0xAB, 0x5C, 0xEB, 0x9A, 0x6D, 0xC6, 0xFA, 0x8F}};

// Stock Qualcomm I2C protocol (layout recovered from I2CDxe.efi).
// All functions return Qualcomm I2C status codes (0 = success, 1 = bad param,
// 3 = boot services already exited).
typedef struct {
  UINT64 Revision;
  INT32 (*Open)(UINT32 Instance, VOID **Handle);
  INT32 (*Read)(VOID *H, VOID *Cfg, UINT32 Off, UINT32 OffLen, UINT8 *Buf, UINT32 Len, UINT32 *Done);
  INT32 (*Write)(VOID *H, VOID *Cfg, UINT32 Off, UINT32 OffLen, UINT8 *Buf, UINT32 Len, UINT32 *Done);
  INT32 (*Batch)(VOID *H, VOID *Cfg, VOID *Xfers, UINT32 Count, VOID *Cb, VOID *Ctx, UINT32 Timeout, UINT32 *Done);
  INT32 (*Close)(VOID *H);
} QI2C;

typedef struct { UINT8 *Buf; UINT32 Len; UINT32 Flags; UINT32 Tail; UINT32 Pad; } XFER;  // 0x18 bytes
typedef struct { UINT32 FreqKhz; UINT32 Addr; UINT32 Mode; UINT32 StretchUs; UINT8 Rest[0x30]; } I2CCFG;

#define F_START 1
#define F_STOP 2
#define F_WRITE 4
#define F_READ 8

static GUID gPmicLdo = {0x22D38D3D, 0xE8B6, 0x4F8F, {0x9C, 0x26, 0xBC, 0xEB, 0x07, 0xD6, 0xCB, 0x68}};

// Stock PmicDxe LDO protocol (revision 0x10005), layout recovered from PmicDxe.efi.
// LDO indexes are 0-based (LDO4 = 3); PMIC index 1 = "b" (PM8550) in the driver's naming.
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

static GUID gCmdDb = {0x1C34F691, 0xD33D, 0x4E14, {0xB8, 0xF8, 0x32, 0xC6, 0xC0, 0x29, 0xE9, 0x5B}};
static GUID gRpmh = {0xE1D0B573, 0xF29F, 0x495D, {0x93, 0x52, 0xCA, 0x0D, 0xD4, 0x2A, 0x0A, 0x74}};

// CmdDbDxe protocol: Revision + 7 functions, each takes This first. fn[1] = QueryAddr(This, ResId[8], &Addr).
typedef struct { UINT64 Revision; VOID *fn[7]; } CMDDB;
// RpmhDxe protocol: Revision + 24 functions that do NOT take This.
//   fn[5]  (+0x30) CreateHandle(DrvId, Name, &Handle)
//   fn[14] (+0x78) IssueCommandSet(Handle, Set, &Req)
//   fn[22] (+0xb8) Wait(Handle, Req)
typedef struct { UINT64 Revision; VOID *fn[24]; } RPMH;
typedef struct { UINT32 Addr, Data, Wait; } RCMD;
typedef struct { UINT32 State, Count, Result; RCMD Cmd[16]; } RSET;

// ---- tiny runtime --------------------------------------------------------
VOID *memset(VOID *d, int c, UINTN n) { UINT8 *p = d; while (n--) *p++ = (UINT8)c; return d; }
VOID *memcpy(VOID *d, const VOID *s, UINTN n) { UINT8 *p = d; const UINT8 *q = s; while (n--) *p++ = *q++; return d; }

static SYSTAB *ST;
static FILEP *gLog;
static FILEP *gRoot;

static void out_raw(const char *s, UINTN n) {
  // log first, so a console problem can never lose it
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
    ST->ConOut->OutputString(ST->ConOut, w);   // This = protocol pointer, not the handle
  }
}

static void pr(const char *fmt, ...) {
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

// ---- TLMM (raw MMIO, layout from pinctrl-sm8550.c) -----------------------
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
  wr(GPIO_IO(n), val ? 2 : 0);                       // set level first
  UINT32 c = rd(GPIO_CFG(n));
  c &= ~(0xFu << 2);                                 // function 0 = GPIO
  c |= (1u << 9);                                    // output enable
  wr(GPIO_CFG(n), c);
}
static void gpio_in_pullup(int n) {
  UINT32 c = rd(GPIO_CFG(n));
  c &= ~((0xFu << 2) | (1u << 9) | 3u);
  c |= 3u;                                           // pull-up
  wr(GPIO_CFG(n), c);
}
static int gpio_get(int n) { return rd(GPIO_IO(n)) & 1; }


static void hexdump(const UINT8 *b, int n) { for (int i = 0; i < n; i++) pr("%02x ", b[i]); }

// ---- GCC / QUP ----------------------------------------------------------
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
    int hw = (h & 2) != 0;   // CBCR bit 1 = hardware clock gating: reads OFF while idle, wakes on access
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
  // geni_se_init
  se_irq_clear();
  se_wr(0x28, se_rd(0x28) | 0x7F);                    // CGC_CTRL
  se_wr(0xe30, se_rd(0xe30) | 0xF);                   // DMA_GENERAL_CFG
  se_wr(0x24, 0x7F);                                  // OUTPUT_CTRL
  se_wr(0x20, 1);                                     // FORCE_DEFAULT
  se_wr(0xe1c, se_rd(0xe1c) | 0xF);                   // SE_IRQ_EN (m, s, tx, rx)
  se_wr(0x258, se_rd(0x258) & ~1u);                   // FIFO mode (DMA_MODE_EN off)
  se_wr(0xe18, 0);
  se_wr(0x810, tx_depth - 1);                         // RX watermark
  se_wr(0x814, tx_depth);                             // RX RFR watermark
  se_wr(0x614, se_rd(0x614) | 0x33C0007Eu);           // M_COMMON_GENI_M_IRQ_EN
  // geni_se_config_packing(8 bits, 4 words, msb_to_lsb)
  se_wr(0x260, 0x7F8FE); se_wr(0x264, 0xFFEFE);
  se_wr(0x284, 0x7F8FE); se_wr(0x288, 0xFFEFE);
  se_wr(0x254, 0);
  // qcom_geni_i2c_conf: 19.2 MHz root (DFS level 0), 400 kHz
  se_wr(0x7c, 0);                                     // CLK_SEL = level 0
  se_wr(0x48, (2u << 4) | 1u);                        // clk_div 2, SER_CLK_EN
  se_wr(0x278, (5u << 20) | (11u << 10) | 22u);       // 400 kHz: t_high, t_low, t_cycle
}

static int i2c_read(UINT8 addr, UINT8 *buf, UINT32 len, UINT32 *seen) {
  UINT32 g = se_rd(0x40);
  if (g & 1) { pr("  GENI cmd already active (status=%08x), not starting\n", g); return -1; }
  se_irq_clear();
  se_wr(0x614, se_rd(0x614) | 1u | (1u << 30) | (1u << 26) | (1u << 27));
  se_wr(0x258, se_rd(0x258) & ~1u);
  se_wr(0x270, len);                                  // RX_TRANS_LEN
  se_wr(0x600, (2u << 27) | ((UINT32)addr << 9));     // M_CMD0 = I2C_READ
  UINT32 got = 0, all = 0;
  int done = 0, t;
  for (t = 0; t < 4000 && !done; t++) {
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
    if (!done) BS_STALL(100);
  }
  if (!done) {
    pr("  timeout, cancelling\n");
    se_wr(0x604, 1u << 2);
    for (t = 0; t < 100 && !(se_rd(0x610) & (1u << 4)); t++) BS_STALL(100);
    if (!(se_rd(0x610) & (1u << 4))) { se_wr(0x604, 1u << 1); BS_STALL(5000); }
    se_wr(0x618, 0xFFFFFFFFu);
  }
  *seen = all;
  pr("  m_irq_status seen=%08x bytes=%d geni_status=%08x ios=%08x\n", all, got, se_rd(0x40), se_rd(0x908));
  if (all & (1u << 10)) {
    UINT32 gp = se_rd(0x910);
    pr("  NACK: gp_length=%d -> %s\n", gp, gp ? "data NACK" : "ADDRESS NACK (nothing answered at 0x4b)");
    return -2;
  }
  if (!done) return -3;
  if (all & ((1u << 1) | (1u << 2) | (1u << 3) | (1u << 12) | (1u << 13))) return -4;
  return (got == len) ? 0 : -5;
}


static int i2c_write(UINT8 addr, const UINT8 *data, UINT32 len) {
  if (se_rd(0x40) & 1) { pr("  GENI cmd already active, not writing\n"); return -1; }
  se_irq_clear();
  se_wr(0x614, se_rd(0x614) | 1u | (1u << 30) | (1u << 26) | (1u << 27));
  se_wr(0x258, se_rd(0x258) & ~1u);
  se_wr(0x26c, len);                                  // TX_TRANS_LEN
  se_wr(0x600, (1u << 27) | ((UINT32)addr << 9));     // M_CMD0 = I2C_WRITE
  se_wr(0x80c, 1);                                    // TX watermark
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
    if (!done) BS_STALL(100);
  }
  if (!done) { se_wr(0x604, 1u << 2); BS_STALL(5000); se_wr(0x618, 0xFFFFFFFFu); return -3; }
  if (all & (1u << 10)) { pr("  write NACK (m_irq=%08x)\n", all); return -2; }
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

static int tcm_msg(UINT8 *buf) {   // one message: Linux reads 256 bytes per report
  UINT32 seen = 0;
  memset(buf, 0, 256);
  return i2c_read(TCM_ADDR, buf, 256, &seen);
}

EFI_STATUS efi_main(EFI_HANDLE image, SYSTAB *st) {
  ST = st;
  BS_WATCHDOG(0, 0, 0, 0);

  VOID *li = 0;
  if (!BS_HANDLE_PROTOCOL(image, &gLoadedImage, &li)) {
    VOID *dev = *(VOID **)((UINT8 *)li + 0x18);
    SFS *sfs = 0;
    FILEP *root = 0, *f = 0;
    if (!BS_HANDLE_PROTOCOL(dev, &gSfs, (VOID **)&sfs) && !sfs->OpenVolume(sfs, &root) &&
        !root->Open(root, (VOID **)&f, (CHAR16 *)u"\\tcmtouch.log", 0x8000000000000003ULL, 0)) {
      f->SetPos(f, ~0ULL);
      gLog = f;
    }
  }
  pr("\n==== SeProbe: S3908 via QUP1 SE4 (polled GENI I2C, addr 0x%02x) ====\n", TCM_ADDR);
  pr("log file: %s\n", gLog ? "\\tcmtouch.log" : "NOT AVAILABLE");

  UINT32 vote0 = rd(GCC(VOTE_REG));
  UINT32 cfg44 = rd(GPIO_CFG(44)), cfg45 = rd(GPIO_CFG(45));
  UINT32 added = 0;
  int rc = -100;

  pr("\n[0] vcc rail: vreg_l4b_1p8 on via RPMh (the PMIC does not allow direct writes)\n");
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
        set.Cmd[0].Addr = base;     set.Cmd[0].Data = 1800; set.Cmd[0].Wait = 1;   // voltage, mV
        set.Cmd[1].Addr = base + 4; set.Cmd[1].Data = 1;    set.Cmd[1].Wait = 1;   // enable
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
  added = want & ~vote0;
  wr(GCC(VOTE_REG), vote0 | want);
  pr("  vote reg %08x -> %08x\n", vote0, rd(GCC(VOTE_REG)));
  if (!clocks_ok()) { pr("  clocks did not come up, stopping before touching the engine\n"); goto restore; }

  pr("\n[3] pin mux gpio44/45 -> qup1_se4 (function 1), pull-up, 2 mA\n");
  wr(GPIO_CFG(44), (cfg44 & ~((0xFu << 2) | 3u | (7u << 6))) | (1u << 2) | 3u);
  wr(GPIO_CFG(45), (cfg45 & ~((0xFu << 2) | 3u | (7u << 6))) | (1u << 2) | 3u);
  gpio_dump(44, "sda");
  gpio_dump(45, "scl");

  pr("\n[4] engine state\n");
  UINT32 hwver = rd(QUP1_BASE + 4);
  UINT32 fw = se_rd(0x68), ifd = se_rd(0x64), hp0 = se_rd(0xe24);
  UINT32 maj = hwver >> 28, mnr = (hwver >> 16) & 0xFFF;
  UINT32 depth = (hp0 >> 16) & ((maj == 3 && mnr >= 10) || maj > 3 ? 0xFFu : 0x3Fu);
  pr("  QUPV3_HW_VER=%08x (%d.%d)  FW_REVISION=%08x  IF_DISABLE=%08x  HW_PARAM_0=%08x\n", hwver, maj, mnr, fw, ifd, hp0);
  pr("  GENI_STATUS=%08x CLK_CTRL_RO=%08x M_CLK_CFG=%08x CLK_SEL=%08x\n", se_rd(0x40), se_rd(0x60), se_rd(0x48), se_rd(0x7c));
  pr("  protocol=%d (3=I2C) fifo_if_disabled=%d tx_fifo_depth=%d\n", (fw >> 8) & 0xFF, ifd & 1, depth);
  if (((fw >> 8) & 0xFF) != 3) { pr("  engine has no I2C firmware loaded -> stopping\n"); goto restore; }
  if (ifd & 1) { pr("  FIFO interface disabled (GPI-only) -> stopping\n"); goto restore; }
  if (!depth) { pr("  zero FIFO depth -> stopping\n"); goto restore; }

  pr("\n[5] configure engine, read the pending IDENTIFY message\n");
  se_setup(depth);
  static UINT8 msg[256];
  rc = tcm_msg(msg);
  pr("  read rc=%d: marker=%02x code=%02x len=%d mode=%d part=%c%c%c%c%c\n", rc, msg[0], msg[1],
     msg[2] | (msg[3] << 8), msg[5], msg[6], msg[7], msg[8], msg[9], msg[10]);
  if (rc || msg[0] != 0xA5) { pr("  no valid IDENTIFY, stopping\n"); goto restore; }

  pr("\n[6] GET_APPLICATION_INFO (0x20)\n");
  rc = tcm_cmd(0x20);
  pr("  send rc=%d, INT low: %d\n", rc, wait_int(500));
  rc = tcm_msg(msg);
  pr("  read rc=%d code=%02x len=%d status=%d version=%d.%d\n", rc, msg[1], msg[2] | (msg[3] << 8),
     msg[6] | (msg[7] << 8), msg[4], msg[5]);
  UINT32 mx = msg[36] | (msg[37] << 8), my = msg[38] | (msg[39] << 8);
  pr("  max_x=%d max_y=%d max_objects=%d buttons=%d\n", mx, my, msg[40] | (msg[41] << 8), msg[42] | (msg[43] << 8));
  pr("  config id: ");
  for (int i = 20; i < 36; i++) pr("%c", (msg[i] >= 32 && msg[i] < 127) ? msg[i] : '.');
  pr("\n");

  pr("\n[7] ENABLE_REPORT (0x05), then touch the screen for ~30 s (any key stops)\n");
  rc = tcm_cmd(0x05);
  pr("  send rc=%d\n", rc);
  UINT32 reports = 0, touches = 0, shown = 0, minx = 99999, maxx = 0, miny = 99999, maxy = 0;
  for (int it = 0; it < 30000; it++) {
    if (it % 200 == 0) { KEY k; if (!st->ConIn->ReadKey(st->ConIn, &k)) break; }
    if (gpio_get(25)) { BS_STALL(1000); continue; }
    rc = tcm_msg(msg);
    if (rc) { pr("  read error %d\n", rc); break; }
    reports++;
    if (msg[0] != 0xA5) continue;
    if (msg[1] == 0x01) { pr("  REPORT_OK\n"); continue; }
    if (msg[1] != 0x11) { if (shown < 40) { pr("  report code %02x\n", msg[1]); shown++; } continue; }
    UINT32 plen = msg[2] | (msg[3] << 8);
    UINT32 n = plen >= 7 ? (plen - 7) / 12 : 0;
    if (n) touches++;
    for (UINT32 i = 0; i < n && i < 5; i++) {
      UINT8 *p = msg + 4 + 1 + 12 * i;
      UINT32 z = p[1] & 0xF, idx = p[6] & 0xF, x = p[7] | (p[8] << 8), y = p[9] | (p[10] << 8);
      if (x < minx) minx = x;
      if (x > maxx) maxx = x;
      if (y < miny) miny = y;
      if (y > maxy) maxy = y;
      if (shown < 120) { pr("  touch #%d obj %d x=%d y=%d z=%d (n=%d)\n", reports, idx, x, y, z, n); shown++; }
    }
  }
  pr("  reports=%d touch-reports=%d  x range %d..%d  y range %d..%d\n", reports, touches, minx, maxx, miny, maxy);
  tcm_cmd(0x06);   // DISABLE_REPORT
  BS_STALL(5000);

restore:
  pr("\n[6] restoring clocks and pins\n");
  wr(GCC(VOTE_REG), rd(GCC(VOTE_REG)) & ~added);
  wr(GPIO_CFG(44), cfg44);
  wr(GPIO_CFG(45), cfg45);
  pr("  vote reg now %08x\n", rd(GCC(VOTE_REG)));

  pr("\nDone. Press any key (Vol/Power) to exit.\n");
  for (int i = 0; i < 2400; i++) {
    KEY k;
    if (!st->ConIn->ReadKey(st->ConIn, &k)) break;
    ms(50);
  }
  if (gLog) gLog->Close(gLog);
  return 0;
}
