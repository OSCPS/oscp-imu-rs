use cobs::{decode, encode};
use core::fmt::{self, Write};

use crate::types::{
    OscpAccelDr, OscpDebug1, OscpDebug2, OscpErr, OscpEuler, OscpFrame, OscpFrameCmd,
    OscpFrameSel, OscpFrameType, OscpGnss, OscpGyroDr, OscpInclDr, OscpOmSel, OscpParseError,
    OscpParser, OscpQuat, OscpRaw, OscpRotMat, OscpStartup, OscpStats, OscpTransport, OscpUsrReg,
    OSCP_FRAME_DELIM, OSCP_FRAME_MAX_LEN, OSCP_FRAME_MIN_LEN,
};

struct OscpCmdRspEntry {
    suffix: &'static str,
    frame_type: OscpFrameCmd,
}

static CRC_LUT: [u16; 256] = [
    0x0000, 0xA2EB, 0xE73D, 0x45D6, 0x6C91, 0xCE7A, 0x8BAC, 0x2947, 0xD922, 0x7BC9, 0x3E1F, 0x9CF4,
    0xB5B3, 0x1758, 0x528E, 0xF065, 0x10AF, 0xB244, 0xF792, 0x5579, 0x7C3E, 0xDED5, 0x9B03, 0x39E8,
    0xC98D, 0x6B66, 0x2EB0, 0x8C5B, 0xA51C, 0x07F7, 0x4221, 0xE0CA, 0x215E, 0x83B5, 0xC663, 0x6488,
    0x4DCF, 0xEF24, 0xAAF2, 0x0819, 0xF87C, 0x5A97, 0x1F41, 0xBDAA, 0x94ED, 0x3606, 0x73D0, 0xD13B,
    0x31F1, 0x931A, 0xD6CC, 0x7427, 0x5D60, 0xFF8B, 0xBA5D, 0x18B6, 0xE8D3, 0x4A38, 0x0FEE, 0xAD05,
    0x8442, 0x26A9, 0x637F, 0xC194, 0x42BC, 0xE057, 0xA581, 0x076A, 0x2E2D, 0x8CC6, 0xC910, 0x6BFB,
    0x9B9E, 0x3975, 0x7CA3, 0xDE48, 0xF70F, 0x55E4, 0x1032, 0xB2D9, 0x5213, 0xF0F8, 0xB52E, 0x17C5,
    0x3E82, 0x9C69, 0xD9BF, 0x7B54, 0x8B31, 0x29DA, 0x6C0C, 0xCEE7, 0xE7A0, 0x454B, 0x009D, 0xA276,
    0x63E2, 0xC109, 0x84DF, 0x2634, 0x0F73, 0xAD98, 0xE84E, 0x4AA5, 0xBAC0, 0x182B, 0x5DFD, 0xFF16,
    0xD651, 0x74BA, 0x316C, 0x9387, 0x734D, 0xD1A6, 0x9470, 0x369B, 0x1FDC, 0xBD37, 0xF8E1, 0x5A0A,
    0xAA6F, 0x0884, 0x4D52, 0xEFB9, 0xC6FE, 0x6415, 0x21C3, 0x8328, 0x8578, 0x2793, 0x6245, 0xC0AE,
    0xE9E9, 0x4B02, 0x0ED4, 0xAC3F, 0x5C5A, 0xFEB1, 0xBB67, 0x198C, 0x30CB, 0x9220, 0xD7F6, 0x751D,
    0x95D7, 0x373C, 0x72EA, 0xD001, 0xF946, 0x5BAD, 0x1E7B, 0xBC90, 0x4CF5, 0xEE1E, 0xABC8, 0x0923,
    0x2064, 0x828F, 0xC759, 0x65B2, 0xA426, 0x06CD, 0x431B, 0xE1F0, 0xC8B7, 0x6A5C, 0x2F8A, 0x8D61,
    0x7D04, 0xDFEF, 0x9A39, 0x38D2, 0x1195, 0xB37E, 0xF6A8, 0x5443, 0xB489, 0x1662, 0x53B4, 0xF15F,
    0xD818, 0x7AF3, 0x3F25, 0x9DCE, 0x6DAB, 0xCF40, 0x8A96, 0x287D, 0x013A, 0xA3D1, 0xE607, 0x44EC,
    0xC7C4, 0x652F, 0x20F9, 0x8212, 0xAB55, 0x09BE, 0x4C68, 0xEE83, 0x1EE6, 0xBC0D, 0xF9DB, 0x5B30,
    0x7277, 0xD09C, 0x954A, 0x37A1, 0xD76B, 0x7580, 0x3056, 0x92BD, 0xBBFA, 0x1911, 0x5CC7, 0xFE2C,
    0x0E49, 0xACA2, 0xE974, 0x4B9F, 0x62D8, 0xC033, 0x85E5, 0x270E, 0xE69A, 0x4471, 0x01A7, 0xA34C,
    0x8A0B, 0x28E0, 0x6D36, 0xCFDD, 0x3FB8, 0x9D53, 0xD885, 0x7A6E, 0x5329, 0xF1C2, 0xB414, 0x16FF,
    0xF635, 0x54DE, 0x1108, 0xB3E3, 0x9AA4, 0x384F, 0x7D99, 0xDF72, 0x2F17, 0x8DFC, 0xC82A, 0x6AC1,
    0x4386, 0xE16D, 0xA4BB, 0x0650,
];

static FRAME_LEN: [u8; 8] = [
    61, /* RAW */
    25, /* EULER */
    29, /* QUATERNION */
    49, /* ROT_MAT */
    64, /* GNSS */
    58, /* DEBUG1 */
    58, /* DEBUG2 */
    40, /* STARTUP */
];

/* Compile-time layout guards — these fail the BUILD, not the test run.
 *
 * Unlike the C driver, this port never reads a frame via a packed struct cast,
 * so there is no `sizeof(struct)` to pin against the wire length. Instead there
 * are two independently-maintained sources of frame length that must never
 * drift apart: each frame type's OSCP_*_LEN constant in types.rs (used by its
 * own from_bytes), and this file's FRAME_LEN table (used by the parser and
 * oscp_frame_decode to pick a length before dispatching). */
const _: () = assert!(crate::types::OSCP_RAW_LEN == FRAME_LEN[0] as usize);
const _: () = assert!(crate::types::OSCP_EULER_LEN == FRAME_LEN[1] as usize);
const _: () = assert!(crate::types::OSCP_QUAT_LEN == FRAME_LEN[2] as usize);
const _: () = assert!(crate::types::OSCP_ROT_MAT_LEN == FRAME_LEN[3] as usize);
const _: () = assert!(crate::types::OSCP_GNSS_LEN == FRAME_LEN[4] as usize);
const _: () = assert!(crate::types::OSCP_DEBUG_1_LEN == FRAME_LEN[5] as usize);
const _: () = assert!(crate::types::OSCP_DEBUG_2_LEN == FRAME_LEN[6] as usize);
const _: () = assert!(crate::types::OSCP_STARTUP_LEN == FRAME_LEN[7] as usize);

/* OSCP_USR_REG_STR's index space must stay exactly matched to OscpUsrReg's
 * variant count, since oscp_cmd_wr indexes into it with `reg as usize` and
 * relies on the type system (not a runtime bounds check) to keep that in range. */
const _: () = assert!(OSCP_USR_REG_STR.len() == 33);

static OSCP_CMD_RSP_TABLE: [OscpCmdRspEntry; 4] = [
    OscpCmdRspEntry { suffix: "Command succeed.\r\n", frame_type: OscpFrameCmd::Success },
    OscpCmdRspEntry { suffix: "Command failed.\r\n", frame_type: OscpFrameCmd::Failed },
    OscpCmdRspEntry { suffix: "This command is erroneous.\r\n", frame_type: OscpFrameCmd::Unknown },
    OscpCmdRspEntry { suffix: "This command is not implemented yet.\r\n", frame_type: OscpFrameCmd::NotImpl },
];

static OSCP_USR_REG_STR: [&str; 33] = [
    "GXB", "GYB", "GZB", "GOB", /* Gyro bias */
    "AXB", "AYB", "AZB", /* Accel bias */
    "IXB", "IYB", /* Inclinometer bias */
    "MXB", "MYB", "MZB", /* Mag bias */
    "MXX", "MYX", "MZX", /* Mag calibration matrix col X */
    "MXY", "MYY", "MZY", /* Mag calibration matrix col Y */
    "MXZ", "MYZ", "MZZ", /* Mag calibration matrix col Z */
    "FCO", "FHS", "FRT", /* AHRS fusion convention, heading src, recovery period */
    "FGA", "FAR", "FMR", /* AHRS gain, accel rejection, mag rejection */
    "GFI", "GLP", "GHP", /* Gyro filter, LPF, HPF */
    "AFI", "ALP", "AHP", /* Accel filter, LPF, HPF */
];

/* ============================================================================
 * SHARED — used by every integration path
 * ==========================================================================*/

pub fn oscp_crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        let pos = ((crc >> 8) as u8) ^ byte;
        crc = (crc << 8) ^ CRC_LUT[pos as usize];
    }
    crc
}

pub fn crc_check(frame: &[u8]) -> bool {
    if frame.len() < 3 {
        return false;
    }
    let (data, crc_bytes) = frame.split_at(frame.len() - 2);
    let computed = oscp_crc16(data);
    let received = u16::from_le_bytes([crc_bytes[0], crc_bytes[1]]);
    computed == received
}

pub fn oscp_cobs_decode(data: &[u8], dec: &mut [u8]) -> Result<usize, OscpErr> {
    decode(data, dec)
        .map(|report| report.frame_size())
        .map_err(|_| OscpErr::Err)
}

pub fn oscp_cobs_encode(data: &[u8], enc: &mut [u8]) -> Result<usize, OscpErr> {
    Ok(encode(data, enc))
}

pub fn dispatch_frame(frame_type: u8, payload: &[u8]) -> Result<OscpFrame, OscpParseError> {
    match OscpFrameType::try_from(frame_type) {
        Ok(OscpFrameType::Raw)        => Ok(OscpFrame::Raw(OscpRaw::from_bytes(payload)?)),
        Ok(OscpFrameType::Euler)      => Ok(OscpFrame::Euler(OscpEuler::from_bytes(payload)?)),
        Ok(OscpFrameType::Quaternion) => Ok(OscpFrame::Quat(OscpQuat::from_bytes(payload)?)),
        Ok(OscpFrameType::RotMatrix)  => Ok(OscpFrame::RotMat(OscpRotMat::from_bytes(payload)?)),
        Ok(OscpFrameType::Gnss)       => Ok(OscpFrame::Gnss(OscpGnss::from_bytes(payload)?)),
        Ok(OscpFrameType::Debug1)     => Ok(OscpFrame::Debug1(OscpDebug1::from_bytes(payload)?)),
        Ok(OscpFrameType::Debug2)     => Ok(OscpFrame::Debug2(OscpDebug2::from_bytes(payload)?)),
        Ok(OscpFrameType::Startup)    => Ok(OscpFrame::Startup(OscpStartup::from_bytes(payload)?)),
        _ => Err(OscpParseError::UnknownFrameType(frame_type)),
    }
}

/* ============================================================================
 * TRANSPORT A - Byte-stream parser (RS-422 variant, or any stream transport)
 *
 * Use this when bytes arrive WITHOUT message boundaries. The parser syncs on
 * the 0x00 delimiter, COBS-decodes, verifies CRC, and yields one OscpFrame
 * per valid frame. Feed it bytes as they arrive (single byte or buffer).
 *
 * CAN-FD users: do NOT use this section - see Transport B below.
 * ==========================================================================*/

/* Matches a decoded buffer's tail against the known command-response suffixes. */
fn match_cmd_response(dec: &[u8]) -> Option<OscpFrameCmd> {
    OSCP_CMD_RSP_TABLE
        .iter()
        .find(|entry| dec.ends_with(entry.suffix.as_bytes()))
        .map(|entry| entry.frame_type)
}

impl OscpParser {
    pub fn new() -> Self {
        Self {
            buf: [0u8; OSCP_FRAME_MAX_LEN],
            buf_len: 0,
            synced: false,
            stats: OscpStats::default(),
        }
    }

    pub fn reset(&mut self) {
        self.buf_len = 0;
        self.synced = false;
    }

    pub fn feed(&mut self, byte: u8) -> Option<OscpFrame> {
        /* Delimiter found */
        if byte == OSCP_FRAME_DELIM {
            /* When synced and data available, parse buffer */
            let frame = if self.synced && self.buf_len > 0 {
                self.parse_buffered()
            } else {
                None
            };

            /* In any case, reset the buffer and assert sync */
            self.buf_len = 0;
            self.synced = true;
            return frame;
        }

        /* When not synced, drop */
        if !self.synced {
            return None;
        }

        /* Protect from overflow */
        if self.buf_len >= OSCP_FRAME_MAX_LEN {
            self.stats.overflows += 1;
            self.buf_len = 0;
            self.synced = false;
            return None;
        }

        /* Insert the byte in the current buffer */
        self.buf[self.buf_len] = byte;
        self.buf_len += 1;
        None
    }

    pub fn feed_buf<'a>(&'a mut self, buf: &'a [u8]) -> impl Iterator<Item = OscpFrame> + 'a {
        buf.iter().filter_map(move |&b| self.feed(b))
    }

    pub fn stats_reset(&mut self) {
        self.stats = OscpStats::default();
    }

    fn parse_buffered(&mut self) -> Option<OscpFrame> {
        let mut dec = [0u8; OSCP_FRAME_MAX_LEN];

        /* Decode */
        let dec_len = match oscp_cobs_decode(&self.buf[..self.buf_len], &mut dec) {
            Ok(len) => len,
            Err(_) => {
                /* Decode Error */
                self.stats.cobs_errors += 1;
                return None;
            }
        };
        let dec = &dec[..dec_len];

        /* Look for command responses first */
        if let Some(resp_type) = match_cmd_response(dec) {
            self.stats.frames_ok += 1;
            return Some(OscpFrame::CmdResponse(resp_type));
        }

        /* Guard against small frame */
        if dec.len() < OSCP_FRAME_MIN_LEN {
            self.stats.framing_errors += 1;
            return None;
        }

        /* Extract the frame type from the first byte */
        let frame_type = dec[0] & 0x07;

        /* frame_type is 0..7 after the 3-bit mask */
        if dec.len() != FRAME_LEN[frame_type as usize] as usize {
            self.stats.framing_errors += 1;
            return None;
        }

        /* Guard against CRC errors */
        if !crc_check(dec) {
            self.stats.crc_errors += 1;
            return None;
        }

        /* Decode a given valid frame */
        match dispatch_frame(frame_type, dec) {
            Ok(frame) => {
                self.stats.frames_ok += 1;
                Some(frame)
            }
            Err(_) => {
                self.stats.framing_errors += 1;
                None
            }
        }
    }
}

impl Default for OscpParser {
    fn default() -> Self {
        Self::new()
    }
}

/* ============================================================================
 * TRANSPORT B - Message decode (CAN-FD variant, or any framed transport)
 *
 * Use this when the transport ALREADY delivers whole messages (e.g. an FD-CAN
 * controller hands a complete payload). Single stateless call:
 * no parser, no ring buffer, no COBS, no callback.
 *
 * RS-422 users: do NOT need this - see Transport A above.
 * ==========================================================================*/

pub fn oscp_frame_decode(payload: &[u8]) -> Result<OscpFrame, OscpParseError> {
    if payload.is_empty() {
        return Err(OscpParseError::WrongLength { expected: 1, actual: 0 });
    }

    /* The frame type comes from the header byte. The per-type CAN identifier carries
     * the same meaning and can be cross-checked by the caller before calling, but is not required here. */
    let frame_type = payload[0] & 0x07;
    let expected = FRAME_LEN[frame_type as usize] as usize;

    /* CAN-FD pads the payload up to the DLC bucket (as an example, a 61-byte RAW frame
     * rides in a 64-byte message), so the received length is >= the frame's own
     * length rather than exactly equal. */
    if payload.len() < expected {
        return Err(OscpParseError::WrongLength { expected, actual: payload.len() });
    }
    let payload = &payload[..expected];

    /* CRC-16 is retained on CAN-FD. */
    if !crc_check(payload) {
        return Err(OscpParseError::CrcMismatch);
    }

    dispatch_frame(frame_type, payload)
}

/* ============================================================================
 * COMMANDS (to unit, both transports layer)
 *
 * Each command encodes an ASCII string into a buffer, framed for a transport:
 *   OscpTransport::Rs422: COBS-encoded, 0x00-delimited
 *   OscpTransport::Canfd: Raw ASCII payload; caller sets the CAN id + DLC
 * ==========================================================================*/

/* Minimal no-alloc `core::fmt::Write` sink over a caller-supplied buffer,
 * used to format the small dynamic command strings below without heap allocation. */
struct FixedBuf<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl<'a> FixedBuf<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, len: 0 }
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap()
    }
}

impl<'a> Write for FixedBuf<'a> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();
        if self.len + bytes.len() > self.buf.len() {
            return Err(fmt::Error);
        }
        self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
        self.len += bytes.len();
        Ok(())
    }
}

/* Frames an ASCII command for the selected transport:
 *   - RS422: COBS-encode the ASCII, then append the 0x00 delimiter.
 *   - CAN-FD: Copy the raw ASCII; the CAN-FD controller frames the message,
 *             and the caller sets the command identifier + DLC. No COBS, no delimiter. */
fn frame_ascii_cmd(ascii: &str, enc: &mut [u8], transport: OscpTransport) -> Result<usize, OscpErr> {
    let ascii = ascii.as_bytes();

    if transport == OscpTransport::Canfd {
        if ascii.len() > enc.len() {
            return Err(OscpErr::Err);
        }
        enc[..ascii.len()].copy_from_slice(ascii);
        return Ok(ascii.len());
    }

    /* OscpTransport::Rs422 from here */
    if enc.is_empty() {
        return Err(OscpErr::Err);
    }
    let cap = enc.len() - 1;
    let cobs_len = oscp_cobs_encode(ascii, &mut enc[..cap])?;
    enc[cobs_len] = OSCP_FRAME_DELIM;
    Ok(cobs_len + 1)
}

/** COMMANDS - Silent Class, ANY (no response expected, valid in ANY state) */
pub fn oscp_cmd_reset(cmd: &mut [u8], transport: OscpTransport) -> Result<usize, OscpErr> {
    frame_ascii_cmd("RESET\r\n", cmd, transport)
}

/** COMMANDS - Silent Class, CONFIG (no response expected, valid in CONFIG state only) */
pub fn oscp_cmd_exit(cmd: &mut [u8], transport: OscpTransport) -> Result<usize, OscpErr> {
    frame_ascii_cmd("EXIT\r\n", cmd, transport)
}

pub fn oscp_cmd_refs(cmd: &mut [u8], transport: OscpTransport) -> Result<usize, OscpErr> {
    frame_ascii_cmd("REFS\r\n", cmd, transport)
}

/** COMMANDS - Ack Class, OPERATING (reply expected, valid in OPERATING state only) */
pub fn oscp_cmd_config(cmd: &mut [u8], transport: OscpTransport) -> Result<usize, OscpErr> {
    frame_ascii_cmd("CONFIG\r\n", cmd, transport)
}

/** COMMANDS - Ack Class, CONFIG (reply expected, valid in CONFIG state only) */
pub fn oscp_cmd_suf(cmd: &mut [u8], transport: OscpTransport) -> Result<usize, OscpErr> {
    frame_ascii_cmd("SUF\r\n", cmd, transport)
}

/** COMMANDS - Silent Class, CONFIG (no CmdResponse - CONFIRMED live: the
 * unit switches its active output frame format immediately and silently;
 * the very next bytes on the wire are already in the new format. A caller
 * waiting for an ack here will always time out even though the command
 * already succeeded - see execute_config_request's SetOutputFrame handling
 * in the dashboard app for how this was diagnosed and worked around.) */
pub fn oscp_cmd_of(cmd: &mut [u8], frame: OscpFrameSel, transport: OscpTransport) -> Result<usize, OscpErr> {
    /* frame: OscpFrameSel already restricts this to a valid selector - no runtime check needed. */
    let mut ascii = [0u8; 5];
    let mut w = FixedBuf::new(&mut ascii);
    write!(w, "OF{}\r\n", frame as u8 as char).map_err(|_| OscpErr::Err)?;
    frame_ascii_cmd(w.as_str(), cmd, transport)
}

pub fn oscp_cmd_om(cmd: &mut [u8], mode: OscpOmSel, transport: OscpTransport) -> Result<usize, OscpErr> {
    let mut ascii = [0u8; 5];
    let mut w = FixedBuf::new(&mut ascii);
    write!(w, "OM{}\r\n", mode as u8 as char).map_err(|_| OscpErr::Err)?;
    frame_ascii_cmd(w.as_str(), cmd, transport)
}

pub fn oscp_cmd_enable_oft(cmd: &mut [u8], frame: OscpFrameSel, transport: OscpTransport) -> Result<usize, OscpErr> {
    /* Unlike oscp_cmd_of, Debug is not a valid selector here. */
    if frame == OscpFrameSel::Debug {
        return Err(OscpErr::ErrInvalid);
    }
    let mut ascii = [0u8; 7];
    let mut w = FixedBuf::new(&mut ascii);
    write!(w, "EOFT{}\r\n", frame as u8 as char).map_err(|_| OscpErr::Err)?;
    frame_ascii_cmd(w.as_str(), cmd, transport)
}

pub fn oscp_cmd_disable_oft(cmd: &mut [u8], frame: OscpFrameSel, transport: OscpTransport) -> Result<usize, OscpErr> {
    if frame == OscpFrameSel::Debug {
        return Err(OscpErr::ErrInvalid);
    }
    let mut ascii = [0u8; 7];
    let mut w = FixedBuf::new(&mut ascii);
    write!(w, "DOFT{}\r\n", frame as u8 as char).map_err(|_| OscpErr::Err)?;
    frame_ascii_cmd(w.as_str(), cmd, transport)
}

/* REVERTED (09-09-2026): removing this zero-padding was tried after
 * 125/250/500dps all acked "Command succeed" but never actually clipped the
 * gyro — but sending the unpadded form ("DRG125\r\n") got a "This command
 * is erroneous" (Unknown) response back instead, i.e. WORSE than before
 * (rejected outright vs silently not applied). That's evidence the
 * opposite way from what was assumed: the firmware likely parses a FIXED 4
 * digits after "DRG" (a common fixed-width-field pattern in this class of
 * ASCII protocol), so the zero-padded form was actually correct and the
 * real cause of 125/250/500dps not clipping is still open — see git
 * history for the reverted attempt/its reasoning, not repeated here since
 * it's now confirmed wrong. Back to the original 4-digit zero-padded form
 * this was changed from. */
pub fn oscp_cmd_drg(cmd: &mut [u8], dr: OscpGyroDr, transport: OscpTransport) -> Result<usize, OscpErr> {
    let mut ascii = [0u8; 9];
    let mut w = FixedBuf::new(&mut ascii);
    write!(w, "DRG{:04}\r\n", dr as u16).map_err(|_| OscpErr::Err)?;
    frame_ascii_cmd(w.as_str(), cmd, transport)
}

pub fn oscp_cmd_dra(cmd: &mut [u8], dr: OscpAccelDr, transport: OscpTransport) -> Result<usize, OscpErr> {
    let mut ascii = [0u8; 7];
    let mut w = FixedBuf::new(&mut ascii);
    write!(w, "DRA{:02}\r\n", dr as u8).map_err(|_| OscpErr::Err)?;
    frame_ascii_cmd(w.as_str(), cmd, transport)
}

pub fn oscp_cmd_dri(cmd: &mut [u8], dr: OscpInclDr, transport: OscpTransport) -> Result<usize, OscpErr> {
    let tenths = dr as u8;
    let mut ascii = [0u8; 8];
    let mut w = FixedBuf::new(&mut ascii);
    write!(w, "DRI{}.{}\r\n", tenths / 10, tenths % 10).map_err(|_| OscpErr::Err)?;
    frame_ascii_cmd(w.as_str(), cmd, transport)
}

pub fn oscp_cmd_enable_mcorr(cmd: &mut [u8], transport: OscpTransport) -> Result<usize, OscpErr> {
    frame_ascii_cmd("EMCORR\r\n", cmd, transport)
}

pub fn oscp_cmd_disable_mcorr(cmd: &mut [u8], transport: OscpTransport) -> Result<usize, OscpErr> {
    frame_ascii_cmd("DMCORR\r\n", cmd, transport)
}

pub fn oscp_cmd_wr(cmd: &mut [u8], reg: OscpUsrReg, value: u32, transport: OscpTransport) -> Result<usize, OscpErr> {
    /* reg: OscpUsrReg already restricts this to a valid index into OSCP_USR_REG_STR - no bounds check needed. */
    let mnemonic = OSCP_USR_REG_STR[reg as usize];
    /* Format: WR + 3-char mnemonic + 8 uppercase hex digits + \r\n = 15 chars */
    let mut ascii = [0u8; 15];
    let mut w = FixedBuf::new(&mut ascii);
    write!(w, "WR{}{:08X}\r\n", mnemonic, value).map_err(|_| OscpErr::Err)?;
    frame_ascii_cmd(w.as_str(), cmd, transport)
}

pub fn oscp_cmd_save(cmd: &mut [u8], transport: OscpTransport) -> Result<usize, OscpErr> {
    frame_ascii_cmd("SAVE\r\n", cmd, transport)
}

/* ============================================================================
 * Test bench for oscp_imu.rs
 *
 * Organized to mirror the library's transport split:
 *   SHARED       — CRC, COBS
 *   TRANSPORT A  — byte-stream parser (RS-422 / UART)
 *   TRANSPORT B  — message decode (CAN-FD)
 *   EQUIVALENCE  — both transports must decode identical bytes identically
 *   COMMANDS     — encoding for both transports, enum validation, buffer sizing
 *
 * Ported from a C/Unity test bench for the original driver. Adapted for
 * differences in this port's design:
 *   - No callback + ctx: OscpParser::feed/feed_buf return decoded frames
 *     directly, so tests collect return values instead of inspecting
 *     mutated state written by a registered callback.
 *   - No tagged union: OscpFrame is a real enum, so "decoded as the right
 *     variant" is checked with `matches!`/pattern match, not a `.type` tag.
 *   - No null-pointer or ctx-forwarding tests: neither concept exists in
 *     safe Rust (references can't be null; there's no `void *ctx` at all).
 *   - Struct-layout static asserts (`sizeof(oscp_raw_t) == 61`) don't apply,
 *     since decoding here never reinterprets memory via a packed struct cast.
 *     The equivalent guards (OSCP_*_LEN vs FRAME_LEN, cross-checked at
 *     compile time) live above FRAME_LEN's definition instead.
 * ==========================================================================*/
#[cfg(test)]
#[path = "oscp_imu_tests.rs"]
mod tests;
