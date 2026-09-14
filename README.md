# OSCP IMU Rust Driver (oscp-imu)

Lightweight, `no_std`-friendly Rust driver for OSCP-family IMUs (MK2M2/MK2E2). The
crate decodes the device's operating frames and encodes its ASCII command set,
with no allocator, no runtime, and no transport assumptions baked in.

## Overview

- Supported frames: raw IMU, Euler angles, quaternion, rotation matrix, GNSS,
  debug 1/2, startup, and command responses (`Success`/`Failed`/`Unknown`/`NotImpl`).
- Designed for both streamed and message-based transports; ships a stateful
  byte-stream parser for RS-422/UART and a stateless single-call decoder for
  CAN-FD, sharing the same frame types and dispatch logic underneath.

## Features

- Dual transport support (RS-422 byte-stream parser and CAN-FD message decode)
- Full OSCP ASCII command surface (reset, config entry/exit, output frame
  select, operating mode, dynamic ranges, user register writes, save)
- `no_std`, no-alloc throughout — fixed-size buffers, no heap
- Optional `serde` feature for `Serialize` on frame/response types

## Installation

Add it as a git dependency in `Cargo.toml`:

```toml
[dependencies]
oscp-imu = { git = "https://github.com/OSCPS/oscp-imu-rs.git" }

# with serde support:
oscp-imu = { git = "https://github.com/OSCPS/oscp-imu-rs.git", features = ["serde"] }
```

## Usage

### RS-422 / UART (byte-stream)

Feed bytes to `OscpParser` as they arrive; it syncs on the `0x00` delimiter,
COBS-decodes, verifies CRC, and yields a decoded `OscpFrame` per valid frame.

```rust
use oscp_imu::{OscpParser, OscpFrame};

let mut parser = OscpParser::new();

for frame in parser.feed_buf(&serial_bytes) {
    match frame {
        OscpFrame::Raw(raw) => { /* ... */ }
        OscpFrame::Quat(quat) => { /* ... */ }
        OscpFrame::CmdResponse(resp) => { /* ... */ }
        _ => {}
    }
}
```

### CAN-FD (message decode)

The controller already delivers whole messages, so there's no parser state —
one stateless call per payload:

```rust
use oscp_imu::oscp_frame_decode;

let frame = oscp_frame_decode(&can_payload)?;
```

## Commands

Commands are encoded per-transport: COBS + `0x00`-delimited for RS-422, raw
ASCII (caller sets CAN id/DLC) for CAN-FD. Key command functions include:

- `oscp_cmd_reset` / `oscp_cmd_exit` / `oscp_cmd_refs`: silent commands, no response expected
- `oscp_cmd_config` / `oscp_cmd_suf`: ack-class commands, response expected
- `oscp_cmd_of` (`OscpFrameSel`): select active output frame
- `oscp_cmd_om` (`OscpOmSel`): select operating mode (Idle/Low/Medium)
- `oscp_cmd_enable_oft` / `oscp_cmd_disable_oft`: enable/disable an output frame type
- `oscp_cmd_drg` / `oscp_cmd_dra` / `oscp_cmd_dri`: set gyro/accel/inclinometer dynamic range
- `oscp_cmd_enable_mcorr` / `oscp_cmd_disable_mcorr`: misalignment correction toggle
- `oscp_cmd_wr` (`OscpUsrReg`, `u32`): write a user register
- `oscp_cmd_save`: persist current config to the unit

Each takes a caller-supplied `&mut [u8]` buffer and an `OscpTransport`
(`Rs422` or `Canfd`), returning the encoded length.

## Frame Types

`OscpFrame` (in `oscp_imu::types`) is the decoded output of both transports:

- `OscpFrame::Raw(OscpRaw)`
- `OscpFrame::Euler(OscpEuler)`
- `OscpFrame::Quat(OscpQuat)`
- `OscpFrame::RotMat(OscpRotMat)`
- `OscpFrame::Gnss(OscpGnss)`
- `OscpFrame::Debug1(OscpDebug1)`
- `OscpFrame::Debug2(OscpDebug2)`
- `OscpFrame::Startup(OscpStartup)`
- `OscpFrame::CmdResponse(OscpFrameCmd)`

## Transport Guidance

For RS-422/UART, drive `OscpParser` with `feed`/`feed_buf` as bytes arrive —
it is stateful and tracks sync, buffering, and parse statistics internally.
For CAN-FD, use `oscp_frame_decode` directly on each complete payload; there
is no parser to construct.

Notes:

- CAN-FD payloads are padded to the DLC bucket (e.g. a 61-byte RAW frame in a
  64-byte message), so `oscp_frame_decode` accepts `payload.len() >= expected`
  rather than requiring an exact match.
- `oscp_cmd_of` (switch output frame format) has no command response by
  design — the unit switches immediately and silently, so a caller waiting
  on a `CmdResponse` for it will always time out even on success.
- Enable the `serde` feature only if you need `Serialize` on frame/response
  types (e.g. for IPC or logging); it is off by default to keep the crate
  fully `no_std`/no-alloc.

## Debugging & Testing

- Run the full test suite:

```bash
cargo test
```

- Run with the `serde` feature enabled:

```bash
cargo test --features serde
```

- Inspect parser statistics after feeding a byte stream:

```rust
let stats = parser.stats; // frames_ok, framing_errors, crc_errors, cobs_errors, overflows
```
