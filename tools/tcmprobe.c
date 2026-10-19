// TcmProbe.efi - standalone EFI app: powers/resets the Synaptics S3908 touch
// controller on OnePlus 11 (salami) and looks for it on the stock Qualcomm I2C
// protocol. Output goes to the screen AND to \tcmprobe.log on the ESP.
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

// ---- S3908 ---------------------------------------------------------------
#define GPIO_VDD 10
#define GPIO_RST 24
#define GPIO_INT 25
#define TCM_ADDR 0x4B

static void hexdump(const UINT8 *b, int n) {
  for (int i = 0; i < n; i++) pr("%02x ", b[i]);
}

static int probe_instance(QI2C *q, UINT32 inst, UINT8 *buf, int len) {
  VOID *h = 0;
  pr("  about to Open(%d) - if the device hangs here, this instance is unusable\n", inst);
  INT32 st = q->Open(inst, &h);
  if (st != 0) { pr("  inst %d: Open -> %d\n", inst, st); return 0; }
  I2CCFG cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.FreqKhz = 400;
  cfg.Addr = TCM_ADDR;
  cfg.Mode = 0;
  cfg.StretchUs = 500;
  memset(buf, 0, len);
  XFER x;
  memset(&x, 0, sizeof(x));
  x.Buf = buf;
  x.Len = len;
  x.Flags = F_START | F_STOP | F_READ;
  UINT32 done = 0;
  st = q->Batch(h, &cfg, &x, 1, 0, 0, 0, &done);
  pr("  inst %d: Open ok, read -> %d done=%d int=%d data: ", inst, st, done, gpio_get(GPIO_INT));
  hexdump(buf, len);
  pr("\n");
  q->Close(h);
  return st == 0 && buf[0] == 0xA5;   // TCM message marker
}

EFI_STATUS efi_main(EFI_HANDLE image, SYSTAB *st) {
  ST = st;
  BS_WATCHDOG(0, 0, 0, 0);

  // open log file on the volume we were loaded from
  VOID *li = 0;
  if (!BS_HANDLE_PROTOCOL(image, &gLoadedImage, &li)) {
    VOID *dev = *(VOID **)((UINT8 *)li + 0x18);
    SFS *sfs = 0;
    FILEP *root = 0, *f = 0;
    if (!BS_HANDLE_PROTOCOL(dev, &gSfs, (VOID **)&sfs) && !sfs->OpenVolume(sfs, &root) &&
        !root->Open(root, (VOID **)&f, (CHAR16 *)u"\\tcmprobe.log", 0x8000000000000003ULL, 0)) {
      f->SetPos(f, ~0ULL);   // append
      gLog = f;
      gRoot = root;
    }
  }
  pr("\n==== TcmProbe: S3908 on i2c4 (addr 0x%02x) ====\n", TCM_ADDR);
  pr("log file: %s\n", gLog ? "\\tcmprobe.log" : "NOT AVAILABLE");

  QI2C *q = 0;
  EFI_STATUS s = BS_LOCATE_PROTOCOL(&gQcomI2c, 0, (VOID **)&q);
  pr("Qualcomm I2C protocol: status=0x%lx ptr=0x%lx rev=0x%lx\n", s, (UINT64)q, q ? q->Revision : 0);
  if (s || !q) goto done;

  pr("\n[1] GPIO state before\n");
  gpio_dump(GPIO_VDD, "vdd ");
  gpio_dump(GPIO_RST, "rst ");
  gpio_dump(GPIO_INT, "int ");

  pr("\n[2] power on: VDD high, reset low->high\n");
  gpio_out(GPIO_VDD, 1);
  ms(20);
  gpio_out(GPIO_RST, 0);
  ms(20);
  gpio_in_pullup(GPIO_INT);
  gpio_out(GPIO_RST, 1);
  pr("  waiting for INT low (data ready)...\n");
  int t;
  for (t = 0; t < 100 && gpio_get(GPIO_INT); t++) ms(10);
  pr("  INT after %d ms: %d\n", t * 10, gpio_get(GPIO_INT));
  gpio_dump(GPIO_VDD, "vdd ");
  gpio_dump(GPIO_RST, "rst ");
  gpio_dump(GPIO_INT, "int ");

  pr("\n[3] I2C probe\n");
  int inst = -1;
  if (gRoot) {
    FILEP *f = 0;
    if (!gRoot->Open(gRoot, (VOID **)&f, (CHAR16 *)u"\\tcmprobe.inst", 1, 0)) {
      char b[16];
      UINTN n = 15;
      memset(b, 0, sizeof(b));
      if (!f->Read(f, &n, b)) {
        int v = 0, ok = 0;
        for (UINTN i = 0; i < n; i++) {
          if (b[i] >= '0' && b[i] <= '9') { v = v * 10 + (b[i] - '0'); ok = 1; }
          else if (ok) break;
        }
        if (ok) inst = v;
      }
      f->Close(f);
    }
  }
  if (inst < 0) {
    pr("  no \\tcmprobe.inst on the ESP -> skipping I2C (put the instance number in that file)\n");
  } else {
    UINT8 buf[8];
    if (probe_instance(q, (UINT32)inst, buf, 8))
      pr("\nRESULT: TCM controller answered on I2C instance %d (marker 0xA5 seen)\n", inst);
    else
      pr("\nRESULT: instance %d gave no TCM message. See status above.\n", inst);
  }

done:
  pr("\nPress any key (Vol/Power) to exit.\n");
  for (int i = 0; i < 2400; i++) {
    KEY k;
    if (!st->ConIn->ReadKey(st->ConIn, &k)) break;
    ms(50);
  }
  if (gLog) gLog->Close(gLog);
  return 0;
}
