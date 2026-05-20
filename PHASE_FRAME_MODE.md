# Direct Phase Frame Mode

This version adds a laptop-calculated phase mode while keeping the original FPGA
coordinate-calculation path in place.

The old flow still exists:

```text
Raspberry Pi keyboard CLI -> coordinates/FIFO/matrix over SPI -> FPGA CalcPhase -> FPGA PWM
```

The new flow is:

```text
Laptop C UI -> 200 calculated phases over TCP -> Raspberry Pi bridge -> 200 phases over SPI -> FPGA PWM
```

Each FPGA still drives 50 outputs. The laptop sends a full 200-phase frame using
the Option A layout, and every FPGA receives the same 200 phases. Each FPGA uses
its physical `top` and `left` pins to keep only the 50 phases that match its
installed board position.

## What Changed

### FPGA

`fpga/Holo.sv` now has a new SPI command:

```text
0xB = CMD_SET_PHASE_FRAME
```

The payload is 200 little-endian `uint16_t` phase values:

```text
command byte, phase0 low, phase0 high, phase1 low, phase1 high, ...
```

The command byte uses the same on-wire encoding as the old commands: the command
number is shifted left by one bit, so `0xB` is sent as `0x16`.

The FPGA receives all 200 phases, but stores only its local 50:

| Global phase indices | Board selected by pins |
| --- | --- |
| `0..49` | `top=0`, `left=1`, left/top |
| `50..99` | `top=0`, `left=0`, right/top |
| `100..149` | `top=1`, `left=1`, left/bottom |
| `150..199` | `top=1`, `left=0`, right/bottom |

The FPGA loads the received phases into the active PWM phase array on
`cycleStart`, so a frame update does not intentionally split a 40 kHz PWM cycle.

The original `CalcPhase.sv` path is not removed. When a direct phase frame is
received, `Holo.sv` enters direct phase mode. When the old FIFO processing path
is enabled again with `CMD_ENABLE_FIFO_PROC`, `Holo.sv` switches back to the
original calculated phase mode.

`HoloTop.sv`, clock generation, `syncin`/`syncout`, `PwmCtrl.sv`, and the PWM
counter behavior are left intact.

### Raspberry Pi

`cli/cli.c` keeps the original keyboard command interface, and adds a bridge
mode:

```bash
./a.out --phase-bridge [tcp-port]
```

Default port:

```text
5656
```

In bridge mode the Pi:

1. Opens SPI0 as before.
2. Listens for laptop TCP connections.
3. Receives fixed-size `HoloPhaseFrame` packets.
4. Validates magic, version, phase count, phase max, CRC, and phase ranges.
5. Forwards the 200 phases to the FPGA with SPI command `0xB`.

The Pi does not calculate phases in bridge mode.

### Laptop

`cli/laptop_phase_sender.c` is the new laptop-side C program. It contains the
same hard-coded transducer coordinate tables as `fpga/CalcPhase.sv`:

- `xLeftTop`
- `xRightTop`
- `xLeftBottom`
- `xRightBottom`
- `zTransducer`

It calculates all 200 phases for the current target point and sends the whole
frame to the Pi.

The laptop uses:

- 50 channels per board
- 4 boards
- 512 phase ticks per 40 kHz cycle
- board distance supplied on the command line
- top/bottom half-height matching the old FPGA `halfHeight` behavior
- a 256-tick phase offset for the bottom boards, matching the old
  `MAX_PHASE_CNT / 2` behavior in `CalcPhase.sv`

## Build

### Raspberry Pi

From the `cli` directory:

```bash
gcc cli.c -lm -o holo_pi_cli
```

This still builds the old CLI and the new bridge mode into the same executable.

### Laptop on Linux/macOS

From the `cli` directory:

```bash
gcc laptop_phase_sender.c -lm -o laptop_phase_sender
```

### Laptop on Windows with MinGW

From the `cli` directory:

```bash
gcc laptop_phase_sender.c -lws2_32 -lm -o laptop_phase_sender.exe
```

## Run

### 1. Program the FPGAs

Build/program the Quartus project as usual. The top-level entity remains
`HoloTop`.

The four FPGA boards still need their physical `top` and `left` pins set
correctly because those pins choose which 50 phases each board uses from the
200-phase frame.

### 2. Start the Raspberry Pi bridge

On the Pi:

```bash
cd cli
gcc cli.c -lm -o holo_pi_cli
./holo_pi_cli --phase-bridge 5656
```

The bridge prints each valid forwarded frame.

### 3. Start the laptop sender

On the laptop:

```bash
cd cli
gcc laptop_phase_sender.c -lm -o laptop_phase_sender
./laptop_phase_sender <pi-ip-address> 5656 135
```

The last argument is board separation in millimeters. `135` matches the value
commonly used by the original CLI.

### 4. Move the focus point

The laptop sender has a small keyboard interface:

| Key | Action |
| --- | --- |
| `h` | home to `0,0,0` |
| `z` / `a` | decrease/increase Z |
| `x` / `s` | decrease/increase X |
| `c` / `d` | decrease/increase Y |
| `p` | print current position |
| `q` | quit |

The sender transmits a complete 200-phase frame after each command.

## Frame Format

`cli/phase_protocol.h` defines the laptop-to-Pi packet:

```c
typedef struct HoloPhaseFrame {
    uint32_t magic;       // "HOLO"
    uint16_t version;     // 1
    uint16_t frame_id;
    uint16_t phase_count; // 200
    uint16_t phase_max;   // 512
    uint16_t phases[200];
    uint32_t crc32;
} HoloPhaseFrame;
```

The phase values are valid in the range `0..511`.

## Notes and Bring-Up Checks

- The old Raspberry Pi coordinate CLI still works with:

```bash
  ./holo_pi_cli 135 2000
```

- Direct phase mode starts only after the FPGA receives a valid `0xB` SPI frame.
- The Pi bridge currently forwards all 200 phases to the FPGA SPI bus exactly as
  Option A described. If each FPGA eventually gets its own chip-select, the Pi
  can be optimized later to send only the matching 50 phases per board.
- If motion looks mirrored or swapped, first verify the physical `top` and
  `left` pins and then verify the global phase order in the laptop sender.
- The laptop constants intentionally mirror the FPGA constants in
  `CalcPhase.sv`; a future cleanup could generate both the C tables and the
  SystemVerilog tables from one source file.
