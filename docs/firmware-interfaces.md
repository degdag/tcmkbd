# Firmware interfaces – stock Qualcomm DXEs on the OnePlus 11 (SM8550)

These protocols are installed by the stock UEFI (the PE images in `Binaries/salami/QcomPkg/Drivers/`).
There are no public headers for this firmware build. Everything here was recovered by disassembling
the images and then **exercised on the device** where marked.

Legend: **[Used]** exercised on hardware by TcmKbd or a probe. **[Decoded]** read from the
disassembly but not called. **[Unknown]** present, purpose not determined.

General rules observed in this firmware:

- AArch64 UEFI calling convention. `EFI_STATUS` is 64-bit; some functions return a 32-bit Qualcomm
  status in `w0` with undefined upper bits, so declare those as `INT32`.
- Protocol tables are a `UINT64 Revision` followed by function pointers. **Check the revision**
  before trusting a layout (values below).
- The Qualcomm protocols here **mostly do not take `This`**. CmdDb does; RPMh and the PMIC LDO
  protocol do not. Getting this wrong crashes or silently misbehaves.
- Some failure paths do **not** return: they loop forever (`ASSERT`). Hazards are listed per protocol.

| Protocol | GUID | Revision |
|---|---|---|
| CmdDb | `1C34F691-D33D-4E14-B8F8-32C6C029E95B` | `0x10000` |
| RPMh | `E1D0B573-F29F-495D-9352-CA0DD42A0A74` | `0x10001` |
| PMIC LDO | `22D38D3D-E8B6-4F8F-9C26-BCEB07D6CB68` | `0x10005` |
| Qualcomm I2C | `B27AE8B1-3E10-4D07-AB5C-EB9A6DC6FA8F` | `0x10000` |
| TLMM (A) | `AD9AEC18-7BF0-4809-9E96-3012309F3DF7` | `1` |
| TLMM (B) | `1A7405FC-0285-41F0-A535-018141F839E7` | `1` |

C form of the GUIDs used by TcmKbd:

```c
static GUID gCmdDb   = {0x1C34F691, 0xD33D, 0x4E14, {0xB8,0xF8,0x32,0xC6,0xC0,0x29,0xE9,0x5B}};
static GUID gRpmh    = {0xE1D0B573, 0xF29F, 0x495D, {0x93,0x52,0xCA,0x0D,0xD4,0x2A,0x0A,0x74}};
static GUID gPmicLdo = {0x22D38D3D, 0xE8B6, 0x4F8F, {0x9C,0x26,0xBC,0xEB,0x07,0xD6,0xCB,0x68}};
static GUID gQcomI2c = {0xB27AE8B1, 0x3E10, 0x4D07, {0xAB,0x5C,0xEB,0x9A,0x6D,0xC6,0xFA,0x8F}};
```

---

## 1. CmdDb (`CmdDbDxe`)

Command database: maps a resource name to the RPMh address of that resource. Seven functions at
offsets `+0x08 .. +0x38`; **every function takes `This` first**.

| Offset | Signature | Status |
|---|---|---|
| `+0x08` | `(This, id, a, b, UINT32 *out)` – looks like an aux-data read | [Decoded] |
| **`+0x10`** | **`QueryAddr(This, const char id[8], UINT32 *Addr)`** | **[Used]** |
| `+0x18` | `(This, UINT32 x, UINT32 y, *out)` | [Decoded] |
| `+0x20` | `(This, id, UINT32 x, *out)` | [Decoded] |
| `+0x28` | `(This, id, UINT16 *out)` (length / slave-id style value) | [Decoded] |
| `+0x30`, `+0x38` | same shape as `+0x28` | [Unknown] |

`id` is a **zero-padded 8-byte buffer** (e.g. `"ldob4\0\0\0"`); `Addr` is 0 / an error status when
the resource does not exist.

```c
char id[8] = {'l','d','o','b','4',0,0,0};
UINT32 base = 0;
((EFI_STATUS(*)(VOID*, const char*, UINT32*))cmddb->fn[1])(cmddb, id, &base);   // fn[1] = +0x10
```

Observed on this device: `ldob1 = 0x40700`, `ldob4 = 0x40800`, `ldob5 = 0x41100`.

## 2. RPMh (`RpmhDxe`)

Sends resource requests to the always-on processor (AOP), the same way Linux's RPMh does. Revision
`0x10001`, 24 functions at `+0x08 .. +0xC0`. **No `This`.**

| Offset | Signature | Status |
|---|---|---|
| `+0x20` | `CreateHandleEx(UINT32 DrvId, const char *Name, BOOLEAN Flag, UINT64 *Handle)` (explicit flag) | [Decoded] |
| `+0x28` | `CreateHandle(DrvId, Name, UINT64 *Handle)`, flag = 1 (allocates a small batch buffer) | [Decoded] |
| **`+0x30`** | **`CreateHandle(UINT32 DrvId, const char *Name, UINT64 *Handle)`**, flag = 0 | **[Used]** |
| **`+0x78`** | **`IssueCommandSet(UINT64 Handle, RPMH_SET *Set, UINT32 *Req)`** | **[Used]** |
| **`+0xB8`** | **`Wait(UINT64 Handle, UINT32 Req)`** | **[Used]** |
| `+0xC0` | another wait/flush style call taking `(Handle, Req)` (the stock PMIC driver uses it) | [Decoded] |
| others | getters/housekeeping (return globals, binary search over a DRV table) | [Unknown] |

All return `EFI_STATUS` (0 = success). The three used functions were checked against the way the stock
`PmicDxe` calls them (create a client handle once, build command sets, issue, wait).

### Command set

```c
typedef struct { UINT32 Addr, Data, Wait; } RCMD;       // 12 bytes
typedef struct {
  UINT32 State;     // 0 = active-only, 1 = sleep, 2 = wake-only
  UINT32 Count;     // <= 16 (checked against the DRV's TCS capacity)
  UINT32 Result;    // zeroed/filled by the driver
  RCMD   Cmd[16];   // commands start at byte offset 0x0C
} RPMH_SET;
```

### Regulator (VRM) resources
Each regulator resource has three registers at `base + 0`, `+4`, `+8` (same layout Linux's
`qcom-rpmh-regulator` uses): **voltage in mV**, **enable (0/1)**, **mode** (not sent by TcmKbd; the default
works). `base` comes from CmdDb.

```c
RPMH_SET set = {0};                                   // State 0 = active-only
set.Count = 2;
set.Cmd[0] = (RCMD){ base + 0, 1800, 1 };             // voltage 1800 mV
set.Cmd[1] = (RCMD){ base + 4, 1,    1 };             // enable
UINT64 h; UINT32 req;
create(2 /*DRV id*/, "name", &h);                     // +0x30
issue(h, &set, &req);                                 // +0x78
if (req) wait(h, req);                                // +0xB8
```

### Hazards
- **DRV id 2** is the one that works here. The driver validates the id against its table and
  **asserts (hangs) on an unknown id**, in `CreateHandle` and in `IssueCommandSet`.
- The implementation has a **"standalone mode, short circuiting"** path: if taken, requests are accepted
  but never reach the AOP (the rail would not change). The `PMIC LDO GetStatus` before/after check detects it.
- It logs debug strings (`rpmh_issue_command_set`, `resource_command`, ...) through the firmware debug path.

## 3. PMIC LDO (`PmicDxe`)

Revision `0x10005`. Seven functions, **no `This`**. Indexes are `(PmicIndex, LdoIndex)`, LDO index
**0-based** (LDO4 = 3).

| Offset | Signature | Status |
|---|---|---|
| `+0x08` | `SwEnable(UINT32 Pmic, UINT32 Ldo, UINT8 Enable)` | [Decoded] **do not call on RPMh-managed rails (crashes)** |
| `+0x10` | `SetMv(Pmic, Ldo, UINT32 mV)` (multiplies by 1000, calls `+0x38`) | [Decoded] **write: crashes on L4B** |
| **`+0x18`** | **`GetLevel(Pmic, Ldo, UINT32 *Level)`** returns **microvolts** (1,800,000 for L4B) | **[Used]** (read) |
| `+0x20` | `(Pmic, Ldo, UINT32 mode)` with mode <= 4 (software mode set) | [Decoded] |
| `+0x28` | always returns `EFI_UNSUPPORTED` | [Decoded] |
| **`+0x30`** | **`GetStatus(Pmic, Ldo, UINT32 out[5])`** | **[Used]** (read) |
| `+0x38` | `SetUv(Pmic, Ldo, UINT32 uV)` | [Decoded] write |

LDO indexes below `0x21` select the LDO block; indexes `0x22..0x35` select voltage switches (index
minus `0x22`). Index `0x21` is an invalid separator.

`GetStatus` output: four `UINT32` (`f0..f3`) followed by a byte `e` at offset `0x10`
(a `UINT32 out[5]` buffer is enough). The values come from reads of the LDO's registers:
rails that are on read `f3 = 1, e = 1`, rails that are off read `f3 = 0, e = 0`. L4B reads `f0=0 f1=3 f2=0 f3=0 e=0`
(off) at UEFI entry and `f3=1 e=1` after the RPMh request. The exact meaning of each field was not
decoded ([Inferred]: `f3` = enabled, `e` = output ready).

Return values: `0` success, `0x8000000000000007` (`EFI_DEVICE_ERROR`) when the PMIC index/LDO is not
present, `0x8000000000000003` (`EFI_UNSUPPORTED`) for the stub.

### Hazards
- **Reads are safe, writes to RPMh-managed rails crash** (observed twice: `SetMv` and `SwEnable`
  on L4B, both stopped the firmware at the call). Those rails are owned by the always-on processor.
- The set-level path contains assertion loops for invalid indexes (hang). The status path returns
  an error instead.
- PMIC index meaning (**[Inferred]**): 0 = PMK8550 (no LDOs), 1 = PM8550 ("b"), 2 and 3 = further PMICs
  with three LDOs each. LDO numbers match the DTS names (`ldob4` = LDO4 = index 3).

`PmicDxe` also registers per-resource "node" names (`/node/ldob5`, `ldob8`, `ldob9`, `ldob11`, `ldob12`,
`ldob13`, `ldob15`, `ldob17`, `ldoc3`, `ldod1`, `ldoe1`, `ldoe3`, `ldof3`, `ldog1`, `ldog3`). Each node
holds the 0-based LDO index (B5 -> 4). **`ldob4` is not one of them**, which fits "L4B is off at boot".

## 4. Qualcomm I2C (`I2CDxe`) – not used by TcmKbd

Revision `0x10000`; table order: `Open`, `Read`, `Write`, `BatchTransfer`, `Close` (no `This`).
Return codes are Qualcomm I2C results, not `EFI_STATUS`: `0` success, `1` invalid parameter, `3` called
after `ExitBootServices`, `28` (`0x1C`) instance has no configuration.

| Offset | Signature |
|---|---|
| `+0x08` | `Open(UINT32 Instance, VOID **Handle)` |
| `+0x10` | `Read(H, Cfg, UINT32 Offset, UINT32 OffsetLen, UINT8 *Buf, UINT32 Len, UINT32 *Done)` (writes the offset, then reads) |
| `+0x18` | `Write(...)` same shape (offset + data in one write) |
| `+0x20` | `BatchTransfer(H, Cfg, XFER *X, UINT32 Count, VOID *Cb, VOID *Ctx, UINT32 Timeout, UINT32 *Done)` – `Cb == NULL` = synchronous; `Done` = bytes transferred |
| `+0x28` | `Close(H)` |

```c
typedef struct { UINT8 *Buf; UINT32 Len; UINT32 Flags; UINT32 Tail; UINT32 Pad; } XFER;  // 0x18 bytes
// Flags: START=1  STOP=2  WRITE=4  READ=8   (plain write = 7, plain read = 0xB)
typedef struct { UINT32 FreqKhz /*100/400/1000*/; UINT32 Addr /*7-bit*/; UINT32 Mode /*0=I2C*/;
                 UINT32 StretchUs; UINT8 Rest[0x30]; } I2CCFG;   // offsets +0/+4/+8 verified; keep the rest 0
```

The driver looks each instance up in the firmware's device-config blob at
`/soc/i2c/i2c_device_config_<N>` (properties: `core_base_addr`, `common_base_addr`, `core_offset`,
`qupv3_instance`, `core_index`, `se_index`, `gpi_index`, `core_irq`, `polled_mode`, `i2c_hub`,
`min_data_length_for_dma`, `scl_encoding`, `sda_encoding`, `tcsr_*`, `se_clock`). Only instances **1, 3, 4, 19, 23**
exist (table in `hardware.md`). **Linux `i2c4` (`0xA90000`) is missing**, and opening a hub instance
(1, 3, 4) **hangs** because the hub clocks are not enabled. That is why TcmKbd programs the engine itself.

## 5. TLMM (`TLMMDxe`) – not used by TcmKbd

Two protocols. (A) `AD9AEC18-...`, revision 1, five functions: `ConfigGpio(Config, Enable)`,
`ConfigGpioGroup(Enable, Configs*, Count)`, `GpioIn(Config, *Value)`, `GpioOut(Config, Value)`, and one more
group output call **[Decoded]**. (B) `1A7405FC-...`, revision 1, seven functions **[Unknown]**.
The packed `Config` word (believed to be Qualcomm's `DAL_GPIO_CFG(gpio, func, dir, pull, drive)`) was
**not verified**, so the driver uses raw TLMM MMIO instead (`hardware.md`).

## 6. Present but not analysed
`PmicGlinkDxe`, `GLinkDxe`, `SmemDxe`, `ClockDxe` (incl. a patched variant in the tree), `GpiDxe`, `RngDxe`.

---

## 7. Device-config blob (where the I2C instances live)

`xbl_config` contains several devicetree blobs. To extract and list the I2C instances:

```python
import struct
d = open('xbl_config_a.img', 'rb').read(); i = n = 0
while True:
    i = d.find(b'\xd0\x0d\xfe\xed', i)
    if i < 0: break
    sz = struct.unpack('>I', d[i+4:i+8])[0]
    if 0x40 < sz < 0x400000:
        open('xblcfg_%d.dtb' % n, 'wb').write(d[i:i+sz]); n += 1
    i += 4
```

```sh
for f in xblcfg_*.dtb; do fdtget -l $f /soc/i2c 2>/dev/null && echo "^ $f"; done
dtc -I dtb -O dts xblcfg_1.dtb | grep -A14 i2c_device_config_
```

## 8. How the interfaces were recovered (to reproduce on another build)

Tools: `uefi-firmware-parser`, `pefile`, `capstone` (all `pip`).

1. **Get the PE images.** The shipped `Binaries/.../*.efi` are usable as-is. From a raw flash
   volume: `uefi-firmware-parser -b -e -o out <image>`, then take the `.pe` sections.
2. **Disassemble the `.text` section** with capstone (`CS_ARCH_ARM64`, `skipdata=True` because
   data and strings are mixed into `.text`). Image base is 0, so addresses are RVAs.
3. **Find where each protocol is installed.** Look for `ldr xN, [xM, #0x80]` (`InstallProtocolInterface`)
   and `#0x148` (`InstallMultipleProtocolInterfaces`) in boot-services calls, and resolve the
   preceding `adrp` + `add` pairs: those are the GUID address and the interface address
   (`InstallMultiple` takes GUID/interface pairs in `x1..x7`).
4. **Read the interface table.** Dump consecutive `UINT64` slots from the interface address and use
   the `.reloc` table to tell function pointers (relocated) from data. The first slot is the revision.
5. **Work out each function's signature** from its prologue: which of `x0..x7` it reads, whether the
   first argument is a `This`, which internal helper it calls, and how it builds the return status.
6. **Find call examples in another driver.** Locating a protocol GUID in a *consumer* (here `PmicDxe`
   references the CmdDb and RPMh GUIDs) shows real call sites, which is how the RPMh call sequence and command
   layout were confirmed.
7. **Confirm on hardware** with a read-only probe first (as done here: `GetStatus`, `QueryAddr`), then the
   smallest write, and log every line before the call.

Minimal table dump (what step 4 amounts to):

```python
import pefile, struct
pe = pefile.PE('X.efi'); img = bytes(pe.get_memory_mapped_image())
relocs = {r.rva for e in pe.DIRECTORY_ENTRY_BASERELOC for r in e.entries if r.rva}
q = lambda o: struct.unpack('<Q', img[o:o+8])[0]
tbl = 0x0000                                  # interface address from the install site
print('revision', hex(q(tbl)))
o = tbl + 8
while o in relocs:
    print(hex(o - tbl), hex(q(o))); o += 8
```

## 9. Porting checklist

Before using this on another firmware build or device, verify: GUIDs resolve with `LocateProtocol`,
the revision numbers above match, CmdDb returns a non-zero address for the target resource, the
RPMh DRV id exists, and read-only status calls (`GetStatus`) behave. Anything that differs means the
layouts above may have changed.
