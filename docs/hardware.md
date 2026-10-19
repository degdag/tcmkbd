# Hardware reference – OnePlus 11 (SM8550 "salami") touch subsystem

Everything TcmKbd touches directly, with the values observed on the device. Each fact is marked:

- **[DTS]** from the mainline `sm8550-oneplus-salami.dts` / `sm8550.dtsi`
- **[Linux]** from a Linux kernel source file named in the row
- **[Observed]** read or measured on the device from UEFI (probe logs)
- **[Inferred]** deduced, not confirmed on hardware

SoC: `soc_id 519` (0x207), revision **2.0** (Kalama v2) **[Observed]**.

---

## 1. Touch controller

| Item | Value | Mark |
|---|---|---|
| IC | Synaptics **S3908**, TCM protocol; part number string `s3908` in IDENTIFY | [DTS] [Observed] |
| I2C address | **0x4B** | [DTS] |
| Bus | Linux `&i2c4` = QUP wrapper 1, serial engine 4, registers at **0xA90000** | [DTS] |
| Interrupt | **gpio25**, level **low** while a message is pending | [DTS] [Observed] |
| Reset | **gpio24**, active **low** (hold low 10 ms, release, wait 80 ms) | [DTS] + Linux driver sequence |
| VDD | **gpio10**, active-high enable of a fixed 2.8 V regulator (`enable2v8_gpio`, always-on in the DTS) | [DTS] |
| VCC | `vreg_l4b_1p8`, 1.8 V (PM8550 LDO4 on RPMh, "b" regulator set, initial mode HPM) | [DTS] |
| Panel size in DTS | 1440 x 3216, max 10 objects | [DTS] |
| Raw touch range | x 277..5719, y 320..12720 seen in a 30 s test; default used by the driver 5760 x 12864 (4x the panel) | [Observed] / [Inferred] (GET_APPLICATION_INFO max values were never captured) |
| UEFI GOP | 1080 x 2412, 32 bpp | [Observed] |

DTS excerpt (device tree used on this phone):

```dts
&i2c4 {
    touchscreen@4b {
        compatible = "syna,s3908";
        reg = <0x4b>;
        interrupts-extended = <&tlmm 25 0x2008>;       /* type bits 0x8 = level low */
        reset-gpios = <&tlmm 24 GPIO_ACTIVE_HIGH>;
        pinctrl-0 = <&ts_int_active &ts_reset_active>;
        vdd-supply = <&enable2v8_gpio>;                /* gpio10, active high */
        vcc-supply = <&vreg_l4b_1p8>;
        Panel_Max_X = <1440>;  Panel_Max_Y = <3216>;  max-objects = <10>;
    };
};
/* ts_reset_active: gpio24 output-low, drive 8mA;  ts_int_active: gpio25 input, pull-up */
```

Power-on sequence the working Linux driver uses: enable vdd, enable vcc, reset **low** for
10 ms, reset **high**, wait 80 ms. The driver in this repo follows the same order with larger waits.

## 2. GPIO / TLMM

TLMM base **0x0F100000** [DTS]. Per-pin registers (from `pinctrl-sm8550.c` [Linux]):

| Register | Address | Fields |
|---|---|---|
| Control | `base + 0x1000*n` | pull `[1:0]` (0 none, 1 pull-down, 2 keeper, 3 pull-up), function `[5:2]`, drive `[8:6]`, output-enable bit 9 |
| I/O | `base + 0x1000*n + 4` | bit 0 = input level, bit 1 = output level |

Function numbers: `gpio44` and `gpio45` use **function 1 = `qup1_se4`** [Linux] (SDA = 44, SCL = 45).

Pin state as found by the first UEFI probe, and what the driver sets:

| Pin | Role | Found in UEFI | Driver sets |
|---|---|---|---|
| gpio10 | VDD enable | `cfg=0x01` input, pull-down | output high |
| gpio24 | reset | `cfg=0x201` output, **already driven low** | low 20 ms, then high |
| gpio25 | INT | `cfg=0x01` input, pull-down | input, pull-up (`cfg=0x03`) |
| gpio44 | SDA | `cfg=0x01` plain GPIO | function 1, pull-up, 2 mA (`cfg=0x07`) |
| gpio45 | SCL | `cfg=0x01` plain GPIO | function 1, pull-up, 2 mA (`cfg=0x07`) |

[Observed] INT goes low about **40 ms** after reset is released once the rails are on.
After the driver sets the mux, both lines read high (idle bus).

## 3. Rails

| Rail | How it is switched in UEFI | Notes |
|---|---|---|
| 2.8 V (VDD) | gpio10 high | straightforward |
| 1.8 V (VCC, `vreg_l4b_1p8`) | **RPMh request** (CmdDb `ldob4` at **0x40800**, voltage 1800 mV at +0, enable at +4) | **Off at UEFI entry** (status `enabled=0 ready=0`), on after the request (`1/1`). Direct PMIC register writes crash the firmware for this rail. See `firmware-interfaces.md`. |

PMIC LDO status scan (PmicDxe `GetStatus`, LDO index 0-based) **[Observed]**, fields `f0 f1 f2 f3 e`:

| PMIC index | Notes | Result |
|---|---|---|
| 0 | PMK8550 (**[Inferred]**) | every call returns an error (no LDOs) |
| 1 | PM8550, DTS regulator set "b" (**[Inferred]**) | LDO0: `1 3 0 0 0`, LDO1: `1 3 0 0 0`, LDO2: `1 0 0 1 1`, **LDO3 (L4B): `0 3 0 0 0` (off)**, LDO4: `1 3 0 1 1`, LDO5: `1 3 0 0 0`, LDO6: `1 3 0 0 0`, LDO7: `1 3 0 1 1`, LDO8: `1 3 0 1 1` |
| 2 | "c" set (**[Inferred]**) | LDO0..2 valid (`1 3 0 0 0`), 3..8 error |
| 3 | "d" set (**[Inferred]**) | LDO0: `1 3 0 1 1`, LDO1: `1 4 0 1 1`, LDO2: `1 3 0 0 0`, 3..8 error |

Rails that XBL itself enabled show `f3=1 e=1`; off rails `0 0`. L4B flips from `f0=0 f1=3 f3=0 e=0`
to `f3=1 e=1` after the RPMh request and its voltage reads **1,800,000 uV** both before and after.

## 4. Clocks (GCC, base 0x100000)

The APCS vote register is **GCC + 0x52008** (value `0x40000200` at UEFI entry; after the driver's
votes `0x443C0200`). Bits **[Linux `gcc-sm8550.c`]**:

| Bit | Clock | Status/CBCR register (GCC +) | Observed at entry |
|---|---|---|---|
| 18 | `gcc_qupv3_wrap1_core_2x_clk` | `0x23294` | on (`0x4000`) |
| 19 | `gcc_qupv3_wrap1_core_clk` | `0x23284` | on (`0x4000`) |
| 20 | `gcc_qupv3_wrap_1_m_ahb_clk` | `0x2327C` | on (`0x0`) |
| 21 | `gcc_qupv3_wrap_1_s_ahb_clk` | `0x23280` | **hardware-gated**: reads `0x80000002` (OFF + HW gating bit) until accessed |
| 26 | `gcc_qupv3_wrap1_s4_clk` | `0x184E4` | off (`0x80004000`) until voted |

Halt bit = bit 31 (1 = OFF). The `s_ahb` branch has hardware clock gating (CBCR bit 1): it reads OFF
while idle and wakes on register access. Linux skips the halt check for such branches; so does the driver.

SE4 root clock generator (RCG) at GCC **+0x184F0** (`CMD`; `+4` CFG, `+8` M, `+0xC` N, `+0x10` D);
DFS control at `+0x14` (bit 0 enables DFS, current level in bits `[3:1]`), per-level registers at
`+0x1C + 4*l` (CFG), `+0x5C + 4*l` (M), `+0x9C + 4*l` (N). Observed (**[Observed]**; frequencies
**[Inferred]** from CFG/M/N with a 300 MHz GPLL0-even and 19.2 MHz XO):

| Level | CFG | M / N | Source / rate |
|---|---|---|---|
| 0 | `0x001` | none | XO, div 1 = **19.2 MHz** (used by the driver) |
| 1 | `0x2601` | 8 / 75 | GPLL0-even x 8/75 = 32 MHz |
| 2 | `0x2601` | 4 / 25 | 48 MHz |
| 3 | `0x2601` | 16 / 75 | 64 MHz |
| 4 | `0x2601` | 4 / 15 | 80 MHz |
| 5 | `0x2601` | 8 / 25 | 96 MHz |
| 6 | `0x605` | none | GPLL0-even / 3 = 100 MHz |
| 7 | `0x000` | - | unused |

CFG decode: source select = bits `[10:8]` (0 = XO, 6 = GPLL0-even), mode = bits `[13:12]`
(2 = dual-edge M/N), half-integer divider field = `2*div - 1`; `N` is stored as `~(N - M)`.
Level 0 and level 1 are the rates the Linux GENI I2C timing tables support.

## 5. QUP / GENI serial engine (wrapper 1, SE4)

Wrapper common block **0xAC0000** (`+4` = `QUPV3_HW_VER`, observed `0x30090000` = 3.9),
SE4 registers at **0xA90000** (all offsets below are relative to it). Values observed with the clocks on:
`GENI_FW_REVISION = 0x303` (protocol field `[15:8]` = **3 = I2C**), `GENI_IF_DISABLE = 0`,
`SE_HW_PARAM_0 = 0x20102864` -> TX FIFO depth **16** words (bits `[21:16]` for QUP < 3.10).

Registers the driver uses (names from Linux `geni-se.h` / `i2c-qcom-geni.c`):

| Offset | Name | Use |
|---|---|---|
| `0x20` | `GENI_FORCE_DEFAULT_REG` | set to 1 at setup |
| `0x24` | `GENI_OUTPUT_CTRL` | `0x7F` at setup |
| `0x28` | `GENI_CGC_CTRL` | OR `0x7F` (clock gating) |
| `0x40` | `GENI_STATUS` | bit 0 = main command active |
| `0x48` | `GENI_SER_M_CLK_CFG` | `(clk_div << 4) \| 1` (`0x21`) |
| `0x60`/`0x64`/`0x68` | `CLK_CTRL_RO`, `IF_DISABLE_RO`, `FW_REVISION_RO` | read-only checks |
| `0x7C` | `GENI_CLK_SEL` | DFS level index (0) |
| `0x254` | `GENI_BYTE_GRAN` | 0 |
| `0x258` | `GENI_DMA_MODE_EN` | clear bit 0 (FIFO mode) |
| `0x260`/`0x264`, `0x284`/`0x288` | TX / RX packing config 0/1 | `0x7F8FE` / `0xFFEFE` (8-bit, 4 words, MSB-to-LSB) |
| `0x26C` / `0x270` | `TX_TRANS_LEN` / `RX_TRANS_LEN` | byte counts |
| `0x278` | `I2C_SCL_COUNTERS` | `(t_high << 20) \| (t_low << 10) \| t_cycle` |
| `0x600` | `M_CMD0` | `(opcode << 27) \| (addr << 9)`; opcode 1 = write, 2 = read |
| `0x604` | `M_CMD_CTRL` | bit 2 cancel, bit 1 abort |
| `0x610` / `0x614` / `0x618` | `M_IRQ_STATUS` / `EN` / `CLEAR` | polled; EN mask `0x33C0007E` plus done/watermark bits |
| `0x648` | `S_IRQ_CLEAR` | cleared at setup |
| `0x700` / `0x780` | `TX_FIFOn` / `RX_FIFOn` | data windows |
| `0x800` / `0x804` | TX / RX FIFO status | RX word count `[24:0]`, last-byte-valid `[30:28]`, last = bit 31 |
| `0x80C` / `0x810` / `0x814` | TX watermark / RX watermark / RX RFR watermark | `1` for writes, `depth-1`, `depth` |
| `0x908` | `GENI_IOS` | line levels (`7` = idle, both lines high) |
| `0x910` | `M_GP_LENGTH` | with GP_IRQ_1: 0 = address NACK |
| `0xC44` / `0xD44` | DMA TX / RX IRQ clear | cleared at setup |
| `0xE18` / `0xE1C` | `SE_GSI_EVENT_EN` / `SE_IRQ_EN` | 0 / all ones |
| `0xE24` / `0xE28` / `0xE30` | `SE_HW_PARAM_0/1`, `SE_DMA_GENERAL_CFG` | FIFO depth; OR `0xF` |

`M_IRQ_STATUS` bits used: 0 command done, 1 overrun, 2 illegal, 3 command failure, 4 cancel,
5 abort, **10 GP_IRQ_1 (NACK)**, 12/13 GP_IRQ_3/4, 26 RX watermark, 27 RX last, 30 TX watermark.

I2C timing at the 19.2 MHz root (from `i2c-qcom-geni.c`), `{clk_div, t_high, t_low, t_cycle}`:

| Rate | Values | `M_CLK_CFG` / `SCL_COUNTERS` |
|---|---|---|
| 100 kHz | 7, 10, 12, 26 | `0x71` / `0xA0301A` |
| **400 kHz (used)** | 2, 5, 11, 22 | `0x21` / `0x502C16` |
| 1 MHz | 1, 2, 8, 18 | n/a |

A 256-byte message read takes a few milliseconds at 400 kHz.

## 6. Other I2C engines present in the firmware (for reference)

The stock I2C driver only knows these instances (from the XBL device-config blob). Pins are decoded
from the `scl/sda_encoding` properties assuming `gpio = (enc >> 4) & 0x3FF` **[Inferred]**:

| Instance | Engine address | Hub? | SCL / SDA |
|---|---|---|---|
| 1 | 0x980000 | yes | 17 / 16 |
| 3 | 0x988000 | yes | 21 / 20 |
| 4 | 0x98C000 | yes | 23 / 22 |
| 19 | 0x880000 | no | 1 / 0 |
| 23 | 0x890000 | no | 3 / 2 |

The touch engine (`0xA90000`) is **not** in the list, which is why the driver programs GENI itself.

## 7. Measurements worth knowing

- Reset-to-INT: ~40 ms. INT stays low until the pending message is read.
- A 30 s touch test returned 86 messages, 67 of them touch reports; two-finger touches showed
  object indexes 0 and 1; reports repeat while a finger rests.
- Touch `z` nibble reads 0 on this panel (ignore it).
