# TcmKbd – touchscreen + on-screen keyboard for UEFI on the OnePlus 11 (SM8550, "salami")

`TcmKbd.efi` is a **resident UEFI boot-service driver** that

1. powers up and talks to the phone's **Synaptics S3908** touch controller from inside UEFI
   (no Linux, no Android, no stock touch driver involved),
2. draws a **touch keyboard** at the bottom of the GOP framebuffer, and
3. **injects the typed keys into the firmware console** (`ConIn` / `ConInEx`), so GRUB, the UEFI
   shell, and any UEFI application that reads the console sees them as normal keystrokes.

It is a single C file built with `clang` + `lld-link`, with no EDK2 build needed. It can be loaded
from the ESP (via GRUB `chainloader`) or embedded in the firmware volume as a DXE.

> **Status:** works on the device (GRUB and UEFI shell). Developed by probing the stock firmware
> and porting the register sequences from the Linux kernel. Read [Safety](#safety) before running
> it on a different device or firmware version.

---

## Table of contents

1. [Features](#features)
2. [How it works](#how-it-works)
3. [Hardware map](#hardware-map)
4. [Bring-up sequence](#bring-up-sequence)
5. [Firmware interfaces used (reverse engineered)](#firmware-interfaces-used)
6. [Touch controller protocol (TCM)](#touch-controller-protocol-tcm)
7. [Keyboard and console integration](#keyboard-and-console-integration)
8. [Build](#build)
9. [Usage](#usage)
10. [Configuration](#configuration)
11. [Log file and troubleshooting](#log-file-and-troubleshooting)
12. [Known limitations and ideas](#known-limitations-and-ideas)
13. [Safety](#safety)
14. [Development notes (what was tried and why)](#development-notes)
15. [Suggested repository layout](#suggested-repository-layout)
16. [Credits and licensing](#credits-and-licensing)

---

## Features

- Brings the touch panel up from a cold UEFI state: regulator, reset, clocks, pin mux, I2C engine.
- Polled I2C master for the Qualcomm GENI serial engine (QUP wrapper 1, SE4 = Linux `i2c4`).
- Multi-finger tracking (per-finger down/up edges, slot timeout for lost releases).
- 6-row on-screen keyboard: letters, digits, punctuation, `ESC TAB CTRL DEL`, arrows, `HOME`,
  `SHIFT` and `CTRL` latches, key repeat, `HIDE` button and a `KB` button to bring it back.
- Keys are delivered through the standard console protocols (`EFI_SIMPLE_TEXT_INPUT_PROTOCOL`
  and `EFI_SIMPLE_TEXT_INPUT_EX_PROTOCOL`) by patching the existing protocol instances in place.
- The text console is kept out of the keyboard's screen area (see
  [console fit](#console-fit)).
- Cleans up at `ExitBootServices` (disables touch reports, restores clock votes and pin mux) so the
  OS starts from the same state the firmware left.
- Writes a detailed bring-up log to `\tcmkbd.log` on the ESP.

## How it works

```
                 +---------------------------- TcmKbd.efi (resident) -----------------------------+
 touch panel --> | 10 ms timer (TPL_CALLBACK)                                                      |
 (S3908, I2C)    |   INT (gpio25) low? -> read one TCM message (polled GENI I2C) -> touch report   |
                 |   -> per-finger down/up edges -> key hit-test -> key queue                      |
                 |                                                                                 |
                 | patched in place:  ConIn / ConInEx : ReadKeyStroke(+Ex), WaitForKey(+Ex) events |
                 |                    ConOut          : OutputString, ClearScreen, SetMode,        |
                 |                                      QueryMode (busy counter + console fit)     |
                 | overlay:  keyboard drawn into the GOP framebuffer, redrawn when the console     |
                 |           may have touched it                                                    |
                 +---------------------------------------------------------------------------------+
```

Why patch instead of installing a new keyboard device? A new `SimpleTextInput` handle is only
picked up if a console splitter connects it; patching the *existing* protocol instances works for
every consumer that already holds the pointers (GRUB, shell, apps), regardless of connect order.

The image is linked as a **boot-service driver** (`/subsystem:efi_boot_service_driver`). If
`efi_main` returns `EFI_SUCCESS` the image stays in memory, and there is deliberately **no Unload
handler**, so `UnloadImage` refuses to remove it (GRUB's `chainloader` tries to unload on exit).

## Hardware map

| Item | Value | Source |
|---|---|---|
| Touch IC | Synaptics **S3908** (TCM protocol), I2C address **0x4B** | DTS `syna,s3908` |
| I2C bus | Linux `&i2c4`, QUP wrapper 1 SE4, registers at **0xA90000**, wrapper common block **0xAC0000** | `sm8550.dtsi` |
| Pins | **gpio44** = SDA, **gpio45** = SCL (function **1**, `qup1_se4`) | `pinctrl-sm8550.c` |
| Interrupt | **gpio25**, active low, level | DTS (`0x2008`, type bits `0x8`) |
| Reset | **gpio24**, active **low** (low 10 ms, then high) | DTS + working Linux driver sequence |
| VDD (2.8 V) | **gpio10**, active-high enable of a fixed regulator | DTS `enable2v8_gpio` |
| VCC (1.8 V) | `vreg_l4b_1p8` = PM8550 LDO4. In PMIC-driver numbering: **PMIC index 1, LDO index 3** (0-based) | DTS + firmware tables |
| TLMM base | `0x0F100000`; per-pin control register at `base + 0x1000*n` (+4 = in/out) | `sm8550.dtsi`, `pinctrl-sm8550.c` |
| GCC base | `0x00100000` | `sm8550.dtsi` |
| Touch raw range | ~`5760 x 12864` (about 4x the 1440x3216 panel) | measured; confirmed by GET_APPLICATION_INFO when it is read |
| GOP (this device) | **1080 x 2412**, 32 bpp | boot log |

TLMM pin register layout used: pull = bits `[1:0]` (3 = pull-up), function = `[5:2]`, drive =
`[8:6]`, output-enable = bit 9; I/O register: bit 0 = input level, bit 1 = output level.

## Bring-up sequence

The driver performs exactly this, in order, and **stops (leaving the console untouched)** if any
step fails:

0. **Enable the 1.8 V rail (L4B) through RPMh.**
   Look up `ldob4` in CmdDb (address `0x40800` on this device), create an RPMh handle (DRV id 2),
   send *voltage = 1800 mV* at `addr+0` and *enable = 1* at `addr+4`, then wait for completion.
   The PMIC driver's LDO status (`enabled`, `ready`) goes `0/0 -> 1/1`.
   (Direct PMIC register writes crash the firmware for this rail, see
   [Firmware interfaces](#firmware-interfaces-used).)
1. **Power the touch IC.** gpio10 high, 20 ms; gpio24 low 20 ms, gpio25 input with pull-up; gpio24
   high; INT goes low about 40 ms later (the controller's IDENTIFY message is waiting); wait 300 ms.
2. **Clocks.** Set vote bits 18, 19, 20, 21, 26 in `GCC+0x52008` (wrapper-1 core, core_2x, m_ahb,
   s_ahb, SE4 clock). Wait for each branch's halt bit to clear. `s_ahb` (halt reg `0x23280`) is a
   **hardware-gated** branch: it keeps reading "OFF" while idle (CBCR bit 1 set) and wakes on
   access, exactly as Linux treats it, so that case counts as OK.
3. **Pin mux.** gpio44/45 to function 1, pull-up, 2 mA.
4. **Engine checks** (all gates): protocol field of `GENI_FW_REVISION` is **3 (I2C)**, FIFO interface
   not disabled, FIFO depth non-zero. These are read from the already-loaded SE firmware, so the
   firmware must have loaded I2C protocol firmware for SE4 (it has on this device).
5. **Engine setup** (port of `geni_se_init` + `geni_i2c_init`): clear IRQs, enable clock gating
   controls, FIFO mode, watermarks (`depth-1`), 8-bit packing, **19.2 MHz root (DFS level 0), 400 kHz**:
   `M_CLK_CFG = 0x21`, `SCL_COUNTERS = 0x502C16`, `CLK_SEL = 0`.
6. **Read the pending IDENTIFY message** (256 bytes). Expect `A5 10 18 00 01 01 'S3908...'`:
   marker, IDENTIFY code, 24-byte payload, version 1, mode 1 = application firmware.
7. **GET_APPLICATION_INFO (0x20)** then **ENABLE_REPORT (0x05)**; touch reports start flowing.
8. Install the keyboard and the console hooks.

At `ExitBootServices` (and on any failure) the driver sends DISABLE_REPORT (0x06), clears the clock
votes it added and restores the original gpio44/45 configuration. The L4B rail and VDD are left on.

## Firmware interfaces used

All of these were recovered by disassembling the stock DXEs shipped in the firmware
(`Binaries/salami/QcomPkg/Drivers/*`). They are **not** public headers: treat the layouts as
observed behaviour for this firmware build.

### CmdDb (`CmdDbDxe`) — GUID `1C34F691-D33D-4E14-B8F8-32C6C029E95B`
Revision `0x10000`, seven functions, each taking `This` first.

| Index | Offset | Use |
|---|---|---|
| 1 | `+0x10` | `QueryAddr(This, char ResId[8], UINT32 *Addr)` — resource id is a zero-padded 8-byte string (`"ldob4"`) |

### RPMh (`RpmhDxe`) — GUID `E1D0B573-F29F-495D-9352-CA0DD42A0A74`
Revision `0x10001`, 24 functions. **These functions do not take `This`.**

| Offset | Signature | Notes |
|---|---|---|
| `+0x30` | `CreateHandle(UINT32 DrvId, const char *Name, UINT64 *Handle)` | DRV id **2**. An unknown DRV id **asserts and hangs** inside RpmhDxe. |
| `+0x78` | `IssueCommandSet(UINT64 Handle, RPMH_SET *Set, UINT32 *Req)` | |
| `+0xB8` | `Wait(UINT64 Handle, UINT32 Req)` | |

Command set layout:

```c
typedef struct { UINT32 Addr, Data, Wait; } RCMD;           // 12 bytes
typedef struct { UINT32 State;    // 0 = active-only, 1 = sleep, 2 = wake-only
                 UINT32 Count;    // <= 16
                 UINT32 Result;   // written by the driver
                 RCMD   Cmd[16]; } RPMH_SET;
```

Regulator (VRM) layout used: `base+0` = voltage in **mV**, `base+4` = enable (0/1), `base+8` = mode
(not sent; the default is fine for L4B).

### PMIC LDO (`PmicDxe`) — GUID `22D38D3D-E8B6-4F8F-9C26-BCEB07D6CB68`
Revision `0x10005`. Functions (also no `This`): `[0] SwEnable(Pmic, Ldo, En)`,
`[1] SetMv(Pmic, Ldo, mV)`, `[2] GetLevel(Pmic, Ldo, *uV)`, `[5] GetStatus(Pmic, Ldo, out)`.
LDO indexes are **0-based** (L4 = 3). PMIC index 0 is the LDO-less PMK8550, index 1 is "b"
(PM8550, nine+ LDOs). `GetStatus` fills `{f0,f1,f2,f3 (u32), e (u8)}`; LDOs that are on read
`f3=1,e=1`, off ones `0,0`.

> **Do not call `SwEnable` / `SetMv` for L4B.** Both are PMIC register *writes* and crash the
> firmware (read calls work). L4B is RPMh-managed, so use the RPMh path above. This driver only
> *reads* `GetStatus` to log the before/after state.

### Qualcomm I2C (`I2CDxe`) — GUID `B27AE8B1-3E10-4D07-AB5C-EB9A6DC6FA8F` (not used by TcmKbd)
Documented because it explains *why the driver programs the GENI engine directly*:

- Layout: `Revision, Open(Instance, &H), Read(...), Write(...), BatchTransfer(H, Cfg, Xfers, Count, Cb, Ctx, Timeout, &Done), Close(H)`.
- Transfer entry = 0x18 bytes `{Buf, Len, Flags}`, flags `START=1 STOP=2 WRITE=4 READ=8`.
- Instances come from the XBL device-config blob (`/soc/i2c/i2c_device_config_<N>`), which contains
  only instances **1, 3, 4** (I2C-hub engines at `0x980000/0x988000/0x98C000`) and **19, 23**
  (`0x880000`, `0x890000`). **Linux `i2c4` (`0xA90000`) is not in the list**, so the stock driver
  cannot reach the touch controller. Opening a hub instance hangs (its clocks are not enabled).

### Other
- `TLMMDxe` exposes `AD9AEC18-7BF0-4809-9E96-3012309F3DF7` (`ConfigGpio`, `GpioIn`, `GpioOut`...).
  Not used: raw TLMM MMIO is simpler and the packed config word was never verified.

## Touch controller protocol (TCM)

Wire format on I2C (address `0x4B`):

- **Read:** one transaction returns one message: `A5 <code> <len lo> <len hi> <payload...>`.
  Linux reads **256 bytes** per message; the driver does the same (`MSG_RD_LEN`).
- **Write:** a command with no payload is a **bare command byte**.
- Interrupt: INT (gpio25) is **level low** while a message is pending.

Commands used: `0x20` GET_APPLICATION_INFO, `0x05` ENABLE_REPORT, `0x06` DISABLE_REPORT.
(`RUN_APPLICATION_FIRMWARE` is not needed: after reset the controller is already in application
mode.) Note that other messages (e.g. code `0x1D`) can arrive *before* the REPORT_OK (`0x01`) that
answers a command, so read until the response code shows up.

IDENTIFY (`code 0x10`, payload 24 bytes): `version, mode, part_number[16], build_id[4], max_write_size[2]`.
`mode 1` = application firmware; the part number reads `s3908`.

APPLICATION_INFO offsets (from message start): `status @6`, `config id @20..35`, `max_x @36`,
`max_y @38`, `max_objects @40`, `buttons @42`.

**Touch report** (`code 0x11`): payload = `fps` (1 byte) + *N* x 12-byte points, with
`N = (len - 7) / 12`. Point layout:

| Byte | Field |
|---|---|
| 0 | unknown |
| 1 | `z` (low nibble; not reliable on this panel) |
| 2-3 / 4-5 | `width_x` / `width_y` |
| 6 | object index (low nibble) |
| 7-8 / 9-10 | `x` / `y` (little-endian, raw units) |
| 11 | reserved |

A finger that is absent from a report has lifted. Raw coordinates are scaled to the GOP size:
`px = x * GOP_W / TOUCH_RAW_W`.

## Keyboard and console integration

### Layout
```
ESC  TAB  CTRL  DEL  <  ^  v  >  HOME  HIDE
 1  2  3  4  5  6  7  8  9  0
 q  w  e  r  t  y  u  i  o  p
 a  s  d  f  g  h  j  k  l  ENT
SHIFT  z  x  c  v  b  n  m  BKSP
 -  =  ;  '  [ SPACE ]  ,  .  /
```
`SHIFT` and `CTRL` are latches (shown highlighted). `DEL`, `BKSP` and arrows auto-repeat
(`REPEAT_DELAY_MS`, `REPEAT_RATE_MS`). `HIDE` collapses the keyboard; a small `KB` button at the
bottom-right brings it back. A key fires the moment a finger lands, so several fingers can type.

### Key delivery
Keys are queued (32 deep) and returned by the patched `ReadKeyStroke` / `ReadKeyStrokeEx`.
`WaitForKey` / `WaitForKeyEx` are replaced by events that fire for either a virtual key or the
original source. Scan codes use the standard UEFI values (Up `0x01`, Down `0x02`, Right `0x03`,
Left `0x04`, Home `0x05`, Delete `0x08`, Esc `0x17`). Because the *existing* protocol instances are
patched, nothing needs to be reconnected.

### Drawing and `ConOut`
`ConOut` is wrapped with a **busy counter**: the overlay is only drawn when no console call is in
flight, and is redrawn after any console operation that may have touched its area
(`ClearScreen`, `SetMode`, or output near the bottom rows).

### Console fit
The stock `GraphicsConsole` centres its text block on the screen. In its largest text mode (126 rows
on this device) the block runs under the keyboard, and scrolling copies the keyboard's pixels upward.
So the driver keeps the console **above the keyboard**:

- `QueryMode` reports a text mode as unsupported if its centred block plus one row of margin would
  end below the keyboard top (so the shell's `mode` command lists only modes that fit),
- `SetMode` is clamped to the largest mode that fits,
- if the console is already in a mode that does not fit when the driver loads, it is switched.

The cell height is measured from the largest mode at install (19 px here). On this device the console
starts in mode 4 (126 rows) and is switched to mode 3. The install log prints one line with the cell
height, current mode, rows and keyboard top.

## Build

Requirements: `clang` and `lld` (`lld-link`) with the AArch64 target. No EDK2 needed.

```sh
clang --target=aarch64-unknown-windows \
      -ffreestanding -fno-builtin -fshort-wchar -fno-stack-protector -O2 -Wall \
      -c TcmKbd.c -o TcmKbd.obj

lld-link /subsystem:efi_boot_service_driver /entry:efi_main /nodefaultlib \
         /machine:arm64 /dll /base:0 TcmKbd.obj /out:TcmKbd.efi
```

Notes:
- It must be **`efi_boot_service_driver`**, not `efi_application`. An application image is freed when
  it returns, which would leave the timer and hooks pointing at freed memory.
- The code is position-independent and the image has no base relocations (`/base:0`).
- `memset` / `memcpy` are provided in the file (`-fno-builtin`, no C runtime).
- The `lld-link` warning about `/align` or `/dll` can be ignored.

## Usage

### From the ESP via GRUB (no firmware reflash)
Copy `TcmKbd.efi` to the ESP (e.g. `/boot/TcmKbd.efi`) and add, **before** the menu you want it in:

```
menuentry "Touch keyboard" {
    chainloader /boot/TcmKbd.efi
    boot
}
```

After it returns, the keyboard stays resident, so launch the next entry (shell, Linux, ...) normally.
The driver tolerates being started once; do not load it twice in the same boot.

### As a DXE in the firmware volume
Not done yet in this repo, but it is straightforward: package the `.efi` with a depex
`gRpmhProtocolGuid AND gCmdDbProtocolGuid` (the two firmware protocols it needs; GUIDs above) and
add it to the FDF like the other prebuilt `Binaries` entries. Keep the module type DXE driver
(boot-service driver subsystem). Make sure it dispatches after the display driver (it needs GOP
and the console protocols), or move the console/GOP lookups to a protocol-notify.

## Configuration

Compile-time `#define`s at the top of `TcmKbd.c`:

| Define | Default | Meaning |
|---|---|---|
| `KB_START_VISIBLE` | `1` | Start with the keyboard shown |
| `TOUCH_RAW_W` / `TOUCH_RAW_H` | `5760` / `12864` | Raw touch range used for scaling |
| `TOUCH_SWAP_XY` | `0` | Swap axes (if the GOP orientation differs from the panel) |
| `TOUCH_INVERT_X` / `TOUCH_INVERT_Y` | `0` | Mirror an axis |
| `MSG_RD_LEN` | `256` | Bytes read per I2C message |
| `TICK_100NS` | `100000` | Poll period (10 ms) |
| `REPEAT_DELAY_MS` / `REPEAT_RATE_MS` | `450` / `70` | Key auto-repeat |
| `SLOT_TIMEOUT_MS` | `150` | A finger counts as lifted if its slot is silent this long |

Colors (`COL_*`) and key sizes are also constants near the key tables.

## Log file and troubleshooting

The driver writes `\tcmkbd.log` to the volume it was loaded from. Each line is flushed as it is
written, so a partial log survives a hang. A healthy run shows `L4B before: enabled=0 ready=0` ->
`after: enabled=1 ready=1`, `INT after ~40 ms: 0`, `protocol=3`, an IDENTIFY with `part=s3908`, and
`hooks installed: ConIn, ConInEx=yes, ConOut`.

| Symptom | Likely cause |
|---|---|
| `cannot send RPMh request` | RPMh/CmdDb protocols missing (loaded too early, or firmware differs) |
| Hang right after `CreateHandle(drv 2)` | DRV id not valid on this firmware build |
| INT stays high, I2C reads NACK | 1.8 V rail not enabled, or reset timing |
| `clocks did not come up` | A branch other than `s_ahb` stayed halted |
| `engine has no I2C firmware` / `FIFO interface disabled` | SE4 not configured for I2C FIFO mode by earlier firmware |
| Crash/exception on a PMIC call | Direct PMIC write was attempted (do not) |
| Keys land on the wrong key | Adjust `TOUCH_RAW_*`, `TOUCH_SWAP_XY`, `TOUCH_INVERT_*` |
| Text overlaps the keyboard | Console mode did not fit; check the `console:` log line |

## Known limitations and ideas

- **Smaller shell output area.** Because text modes are discrete and the console centres its block,
  the largest mode that clears the keyboard (mode 3 here) does not start at the top of the screen.
  The clamp is currently applied even when the keyboard is hidden.
  Ideas: (a) relax the clamp while the keyboard is hidden, (b) hook GOP `Blt` to shift console
  drawing to the top of the screen, (c) a shorter keyboard.
- Only the keyboard is exposed. There is **no `EFI_ABSOLUTE_POINTER_PROTOCOL` yet** (planned, so UEFI
  applications and a future in-firmware OS can use the raw touch input).
- Apps that draw straight into the framebuffer will collide with the overlay. A small custom
  protocol (show / hide / keyboard rectangle) would let them coordinate.
- Reads 256 bytes per message (about 6 ms at 400 kHz); a shorter read may work but is untested.
- Polled, not interrupt-driven: the 10 ms timer checks INT.
- Everything depends on firmware internals and SoC addresses specific to **SM8550 / this firmware
  build** (GUIDs, DRV id, CmdDb names, MMIO addresses).
- No palm rejection, gestures or key-press feedback beyond highlight.

## Safety

This driver writes to hardware registers directly (GCC clock votes, TLMM pins, QUP/GENI registers)
and sends an RPMh request to the always-on processor.

- Run it only on a device and firmware build matching the one it was developed on.
- A wrong DRV id hangs in RpmhDxe; an invalid PMIC/LDO index hangs in PmicDxe; a direct PMIC
  write crashes. None of these should brick the device (a forced reboot recovers), but expect to
  power-cycle if you experiment.
- It only changes state it restores at `ExitBootServices` (clock votes it added, gpio44/45 mux);
  the L4B/VDD rails stay on afterwards.
- No warranty. You are responsible for what you flash or boot.

## Development notes

The order of discovery matters because several "obvious" paths do not work:

1. **Linux/Android side:** the touch driver ported from Linux (power: vdd + vcc, reset low 10 ms then
   high, wait 80 ms; IDENTIFY arrives via the INT line; ENABLE_REPORT is a bare byte).
2. **Standalone EFI probes** (run from GRUB, log to the ESP) proved each layer independently:
   GPIO state, engine survey, I2C read, PMIC status, RPMh request, touch decoding.
3. **The stock I2C protocol cannot reach `i2c4`** (instance list), and opening a hub instance
   hangs, so a polled GENI master was ported from the Linux driver.
4. **A first gate rejected `s_ahb`** because its status reads OFF; it is hardware-gated and wakes on
   access.
5. **Direct PMIC LDO writes crash** while reads work; the rail is RPMh-managed, so the request goes
   through RPMh/CmdDb like Linux does. Reading `GetStatus` before and after proves the effect.
6. **Console integration:** the keyboard overlay first collided with the shell's full-height text
   mode; the text-mode clamp fixed it.

## Suggested repository layout

```
tcmkbd/
  README.md
  src/TcmKbd.c                  the resident driver
  build.sh                      the two commands from "Build"
  tools/                        optional bring-up probes (kept for reference / porting)
    SeSurvey, SeProbe, TcmTouch, RpmhEnable, PmicScan, ...
  docs/
    hardware.md                 register/GPIO tables, pin map
    firmware-interfaces.md      GUIDs and protocol layouts above
    bringup-log-example.txt     a healthy tcmkbd.log
```

## Credits and licensing

- Touch protocol handling is ported from the author's Linux `synaptics_tcm_oncell.c` driver.
- GENI I2C register sequences, clock tables and the `geni_se` setup come from the **Linux kernel**
  (`drivers/i2c/busses/i2c-qcom-geni.c`, `drivers/soc/qcom/qcom-geni-se.c`,
  `drivers/clk/qcom/gcc-sm8550.c`, `drivers/pinctrl/qcom/pinctrl-sm8550.c`), which are
  **GPL-2.0** works. Code derived from them is a derivative work: **release the combined driver under
  GPL-2.0(-or-later) unless you rewrite those parts independently.** Add the SPDX header and a
  `LICENSE` file before publishing.
- Firmware GUIDs and layouts were recovered by disassembling Qualcomm-built firmware components.
  The repository should contain only the driver and notes, not the proprietary binaries themselves.
- The on-screen keyboard uses a small built-in 5x7 bitmap font (`font5x7`).
