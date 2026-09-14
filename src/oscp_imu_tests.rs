use super::*;
use crate::types::{
    DynRangeCfg, InclAhrsCfg, OscpMisalignmentCorr, OscpOperatingMode, OSCP_ENABLE_GNSS,
    OSCP_ENABLE_QUAT, OSCP_ENABLE_RAW, OSCP_STATUS_MEMS_ERR, OSCP_STATUS_OG_ERR, OSCP_STATUS_OK,
    OSCP_STATUS_OVERRUN,
};

/* ==========================================================================
 * Byte-level payload builders
 * ========================================================================*/

#[derive(Default)]
struct PayloadBuilder {
    buf: Vec<u8>,
}

impl PayloadBuilder {
    fn new() -> Self {
        Self::default()
    }

    fn u8(mut self, v: u8) -> Self {
        self.buf.push(v);
        self
    }

    fn u16(mut self, v: u16) -> Self {
        self.buf.extend_from_slice(&v.to_le_bytes());
        self
    }

    fn u32(mut self, v: u32) -> Self {
        self.buf.extend_from_slice(&v.to_le_bytes());
        self
    }

    fn u64(mut self, v: u64) -> Self {
        self.buf.extend_from_slice(&v.to_le_bytes());
        self
    }

    fn f32(mut self, v: f32) -> Self {
        self.buf.extend_from_slice(&v.to_le_bytes());
        self
    }

    fn bytes(mut self, v: &[u8]) -> Self {
        self.buf.extend_from_slice(v);
        self
    }

    /* Appends the little-endian CRC-16 over everything written so far and
     * returns the finished frame - mirrors append_crc() in the C bench. */
    fn with_crc(mut self) -> Vec<u8> {
        let crc = oscp_crc16(&self.buf);
        self.buf.extend_from_slice(&crc.to_le_bytes());
        self.buf
    }
}

fn make_byte0(frame_type: OscpFrameType, mode: OscpOperatingMode, misalign: OscpMisalignmentCorr) -> u8 {
    ((misalign as u8 & 0x03) << 6) | ((mode as u8 & 0x07) << 3) | (frame_type as u8 & 0x07)
}

fn build_raw_payload(counter: u8, ts_ms: u64, gyro_x: f32, gyro_z: f32, accel_z: f32, temp: f32, status: u8) -> Vec<u8> {
    PayloadBuilder::new()
        .u8(make_byte0(OscpFrameType::Raw, OscpOperatingMode::Medium, OscpMisalignmentCorr::Enabled))
        .u8(counter)
        .u64(ts_ms)
        .f32(gyro_x)
        .f32(0.0) /* gyro_y */
        .f32(gyro_z)
        .f32(0.0) /* accel_x */
        .f32(0.0) /* accel_y */
        .f32(accel_z)
        .f32(0.0) /* incl_x */
        .f32(0.0) /* incl_y */
        .f32(0.0) /* mag_x */
        .f32(0.0) /* mag_y */
        .f32(0.0) /* mag_z */
        .f32(temp)
        .u8(status)
        .with_crc()
}

fn build_euler_payload(counter: u8, ts_ms: u64, roll: f32, pitch: f32, yaw: f32, status: u8) -> Vec<u8> {
    PayloadBuilder::new()
        .u8(make_byte0(OscpFrameType::Euler, OscpOperatingMode::Low, OscpMisalignmentCorr::Disabled))
        .u8(counter)
        .u64(ts_ms)
        .f32(roll)
        .f32(pitch)
        .f32(yaw)
        .u8(status)
        .with_crc()
}

fn build_quat_payload(counter: u8, ts_ms: u64, w: f32, x: f32, y: f32, z: f32, status: u8) -> Vec<u8> {
    PayloadBuilder::new()
        .u8(make_byte0(OscpFrameType::Quaternion, OscpOperatingMode::Medium, OscpMisalignmentCorr::Disabled))
        .u8(counter)
        .u64(ts_ms)
        .f32(w)
        .f32(x)
        .f32(y)
        .f32(z)
        .u8(status)
        .with_crc()
}

fn build_rot_mat_payload(counter: u8, ts_ms: u64, rm: &[[f32; 3]; 3], status: u8) -> Vec<u8> {
    let mut b = PayloadBuilder::new()
        .u8(make_byte0(OscpFrameType::RotMatrix, OscpOperatingMode::Medium, OscpMisalignmentCorr::Disabled))
        .u8(counter)
        .u64(ts_ms);
    for row in rm {
        for &v in row {
            b = b.f32(v);
        }
    }
    b.u8(status).with_crc()
}

#[allow(clippy::too_many_arguments)]
fn build_gnss_payload(
    counter: u8,
    ts_ms: u64,
    fix_type: u8,
    num_sats: u8,
    lon: f32,
    lat: f32,
    height: i32,
    vel_n: i32,
    vel_e: i32,
    vel_d: i32,
    pdop: f32,
    fix_ok: bool,
    status: u8,
) -> Vec<u8> {
    PayloadBuilder::new()
        .u8(make_byte0(OscpFrameType::Gnss, OscpOperatingMode::Medium, OscpMisalignmentCorr::Disabled))
        .u8(counter)
        .u64(ts_ms)
        .u8(fix_type)
        .u8(num_sats)
        .f32(lon)
        .f32(lat)
        .u32(height as u32)
        .u32(1500) /* horizontal_accuracy */
        .u32(2500) /* vertical_accuracy */
        .u32(vel_n as u32)
        .u32(vel_e as u32)
        .u32(vel_d as u32)
        .u32(300) /* speed_accuracy */
        .f32(45.0) /* heading_of_motion */
        .f32(1.5) /* heading_accuracy */
        .f32(pdop)
        .u8(if fix_ok { 0x01 } else { 0x00 }) /* gnss_flags: gnssFixOk bit only */
        .u8(status)
        .with_crc()
}

fn build_debug_1_payload(counter: u8, gxb: u32, mzb: u32, status: u8) -> Vec<u8> {
    PayloadBuilder::new()
        .u8(make_byte0(OscpFrameType::Debug1, OscpOperatingMode::Medium, OscpMisalignmentCorr::Disabled))
        .u8(counter)
        .u32(gxb)
        .u32(0) /* gyb */
        .u32(0) /* gzb */
        .u32(0) /* gob */
        .u32(0) /* axb */
        .u32(0) /* ayb */
        .u32(0) /* azb */
        .u32(0) /* ixb */
        .u32(0) /* iyb */
        .u32(0) /* mxb */
        .u32(0) /* myb */
        .u32(mzb)
        .u8(0) /* gyro_filter_cfg */
        .u8(0) /* accel_filter_cfg */
        .u16(0) /* reserved_0 */
        .u8(0) /* reserved_1 */
        .u8(status)
        .with_crc()
}

fn build_debug_2_payload(counter: u8, mxx: u32, fusion_gain: u32, status: u8) -> Vec<u8> {
    PayloadBuilder::new()
        .u8(make_byte0(OscpFrameType::Debug2, OscpOperatingMode::Medium, OscpMisalignmentCorr::Disabled))
        .u8(counter)
        .u32(mxx)
        .u32(0) /* myx */
        .u32(0) /* mzx */
        .u32(0) /* mxy */
        .u32(0) /* myy */
        .u32(0) /* mzy */
        .u32(0) /* mxz */
        .u32(0) /* myz */
        .u32(0) /* mzz */
        .u32(fusion_gain)
        .u32(0) /* fusion_accel_rejection */
        .u32(0) /* fusion_mag_rejection */
        .u32(0) /* fusion_recovery_trigger_period */
        .u8(0) /* fusion_cfg */
        .u8(status)
        .with_crc()
}

#[allow(clippy::too_many_arguments)]
fn build_startup_payload(mark: &str, unit: u16, maj: u8, min: u8, pat: u8, enabled_frames: u8, status: u8) -> Vec<u8> {
    let mut mark_bytes = [0u8; 10];
    let src = mark.as_bytes();
    let n = src.len().min(10);
    mark_bytes[..n].copy_from_slice(&src[..n]);

    PayloadBuilder::new()
        .u8(make_byte0(OscpFrameType::Startup, OscpOperatingMode::Low, OscpMisalignmentCorr::Disabled))
        .bytes(&mark_bytes)
        .u16(unit)
        .u8(maj)
        .u8(min)
        .u8(pat)
        .u8(enabled_frames)
        .u8(0) /* dyn_range_cfg */
        .u8(0) /* gyro_filter_cfg */
        .u8(0) /* accel_filter_cfg */
        .u8(0) /* incl_ahrs_cfg */
        .f32(0.0) /* ahrs_gain */
        .f32(0.0) /* ahrs_accel_rej */
        .f32(0.0) /* ahrs_mag_rej */
        .u32(0) /* ahrs_rec_trig_per */
        .u8(status)
        .with_crc()
}

/* ==========================================================================
 * Transport A (UART) helpers — COBS-wrap a payload and feed the parser
 * ========================================================================*/

fn cobs_wrap(payload: &[u8]) -> Vec<u8> {
    let mut enc = [0u8; 256];
    let enc_len = oscp_cobs_encode(payload, &mut enc).expect("cobs encode failed in test helper");
    let mut wire = enc[..enc_len].to_vec();
    wire.push(OSCP_FRAME_DELIM);
    wire
}

/* Prepend a 0x00 so the parser syncs, then COBS-wrap and feed `payload`
 * (a raw frame payload OR a raw ASCII command-response string - both are
 * just bytes to COBS). Returns every frame the parser yielded. */
fn feed_payload(parser: &mut OscpParser, payload: &[u8]) -> Vec<OscpFrame> {
    let wire = cobs_wrap(payload);
    let mut frames: Vec<OscpFrame> = parser.feed(OSCP_FRAME_DELIM).into_iter().collect();
    frames.extend(parser.feed_buf(&wire));
    frames
}

fn assert_f32_close(expected: f32, actual: f32, eps: f32) {
    assert!(
        (expected - actual).abs() <= eps,
        "expected {expected}, got {actual} (eps {eps})"
    );
}

/* ==========================================================================
 * SHARED — CRC
 * ========================================================================*/

#[test]
fn crc_empty_buffer() {
    /* Empty input must return the init value 0xFFFF */
    assert_eq!(oscp_crc16(&[]), 0xFFFF);
}

/* Golden vectors: these PIN the polynomial (0xD175 koopman / 0xA2EB normal)
 * and init (0xFFFF). A weaker "is non-zero and deterministic" check would
 * still pass if the polynomial silently changed - these will not. */
#[test]
fn crc_golden_check_string() {
    /* "123456789" — the conventional CRC check string */
    assert_eq!(oscp_crc16(b"123456789"), 0x9DB1);
}

#[test]
fn crc_golden_abc() {
    assert_eq!(oscp_crc16(&[0x41, 0x42, 0x43]), 0x5C67);
}

#[test]
fn crc_golden_single_zero() {
    assert_eq!(oscp_crc16(&[0x00]), 0xF950);
}

/* Independent bitwise CRC-16 reference. The library documents "polynomial
 * 0xD175", which is the Koopman representation; the normal (MSB-first) form
 * used by a textbook shift-register implementation is (0xD175 << 1) | 1 ==
 * 0xA2EB. Using 0xD175 directly below would produce different, wrong CRCs. */
const OSCP_CRC_POLY_NORMAL: u16 = 0xA2EB;

fn crc16_bitwise_ref(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            let shifted = crc << 1;
            crc = if crc & 0x8000 != 0 { shifted ^ OSCP_CRC_POLY_NORMAL } else { shifted };
        }
    }
    crc
}

/* Cross-check the 256-entry lookup table against the bitwise reference.
 * Single-byte inputs exercise EVERY LUT entry: with init 0xFFFF the table
 * index is (crc >> 8) ^ byte == 0xFF ^ byte, which ranges over 0..255 as
 * byte does. The golden vectors above only reach ~13 of the 256 entries,
 * so a corrupted entry would otherwise slip through undetected. */
#[test]
fn crc_lut_matches_bitwise_reference_all_entries() {
    for b in 0u8..=255 {
        assert_eq!(
            oscp_crc16(&[b]),
            crc16_bitwise_ref(&[b]),
            "CRC LUT entry disagrees with bitwise reference for byte {b:#04x}"
        );
    }
}

/* Multi-byte cross-check over deterministic pseudo-random buffers. */
#[test]
fn crc_lut_matches_bitwise_reference_multibyte() {
    let mut seed: u32 = 12345;
    for _ in 0..200 {
        let mut buf = [0u8; 64];
        for b in &mut buf {
            seed = seed.wrapping_mul(1103515245).wrapping_add(12345);
            *b = (seed >> 16) as u8;
        }
        assert_eq!(oscp_crc16(&buf), crc16_bitwise_ref(&buf));
    }
}

#[test]
fn crc_single_bit_flip_detected() {
    let mut data = [
        0x08, 0x01, 0x00, 0x00, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x80, 0x00, 0x00,
    ];
    let crc1 = oscp_crc16(&data);
    data[0] ^= 0x01;
    assert_ne!(crc1, oscp_crc16(&data));
}

/* Every single-bit flip across a full frame must change the CRC. */
#[test]
fn crc_all_single_bit_flips_detected() {
    let mut p = build_raw_payload(0x11, 4242, 1.0, 2.0, 3.0, 25.0, OSCP_STATUS_OK);
    p.truncate(59); /* drop the CRC bytes append_crc/with_crc added */
    let good = oscp_crc16(&p);

    for byte in 0..p.len() {
        for bit in 0..8u8 {
            p[byte] ^= 1 << bit;
            assert_ne!(good, oscp_crc16(&p), "bit flip not detected");
            p[byte] ^= 1 << bit; /* restore */
        }
    }
}

#[test]
fn crc_endianness_little_endian() {
    let frame = PayloadBuilder::new().u8(0x08).u8(0x01).u8(0x00).with_crc();
    let stored_le = u16::from_le_bytes([frame[3], frame[4]]);
    assert_eq!(oscp_crc16(&frame[..3]), stored_le);
}

/* ==========================================================================
 * SHARED — COBS
 * ========================================================================*/

fn assert_cobs_roundtrip(orig: &[u8]) {
    let mut enc = [0u8; 256];
    let enc_len = oscp_cobs_encode(orig, &mut enc).expect("encode failed");
    for &b in &enc[..enc_len] {
        assert_ne!(b, 0x00, "0x00 found inside COBS-encoded stream");
    }

    let mut dec = [0u8; 256];
    let dec_len = oscp_cobs_decode(&enc[..enc_len], &mut dec).expect("decode failed");
    assert_eq!(dec_len, orig.len());
    assert_eq!(&dec[..dec_len], orig);
}

#[test]
fn cobs_roundtrip_no_zeros() {
    assert_cobs_roundtrip(&[0x11, 0x22, 0x33, 0x44]);
}

#[test]
fn cobs_roundtrip_single_zero() {
    assert_cobs_roundtrip(&[0x00]);
}

#[test]
fn cobs_roundtrip_all_zeros() {
    assert_cobs_roundtrip(&[0; 8]);
}

#[test]
fn cobs_roundtrip_mixed() {
    assert_cobs_roundtrip(&[0x00, 0xAB, 0x00, 0xCD, 0x00]);
}

#[test]
fn cobs_roundtrip_realistic_raw_payload() {
    let p = build_raw_payload(0x2A, 1000, -9.81, 0.0, 1.0, 30.0, OSCP_STATUS_OK);
    assert_cobs_roundtrip(&p);
}

/* Every OSCP frame length must survive a COBS round-trip within the
 * parser's buffer budget (OSCP_FRAME_MAX_LEN). Guards against a future
 * frame growing past what the parser can hold once COBS overhead is added. */
#[test]
fn cobs_max_frame_fits_parser_buffer() {
    let worst = [0x00u8; 64]; /* all-zero = worst case COBS overhead */
    let mut enc = [0u8; 128];
    let enc_len = oscp_cobs_encode(&worst, &mut enc).expect("encode failed");
    assert!(
        enc_len <= OSCP_FRAME_MAX_LEN,
        "worst-case COBS-encoded frame ({enc_len}) exceeds OSCP_FRAME_MAX_LEN ({OSCP_FRAME_MAX_LEN})"
    );
}

/* ==========================================================================
 * TRANSPORT A — Byte-stream parser (RS-422 / UART)
 * ========================================================================*/

#[test]
fn parser_raw_frame_decoded() {
    let mut parser = OscpParser::new();
    let payload = build_raw_payload(0x2A, 1000, 1.5, -0.5, -1.0, 38.0, OSCP_STATUS_OK);
    let frames = feed_payload(&mut parser, &payload);

    assert_eq!(frames.len(), 1);
    let OscpFrame::Raw(r) = frames[0] else { panic!("expected Raw, got {:?}", frames[0]) };
    assert_eq!(r.counter, 0x2A);
    assert_eq!(r.timestamp_ms, 1000);
    assert_f32_close(1.5, r.gyro_x, 1e-5);
    assert_f32_close(-0.5, r.gyro_z, 1e-5);
    assert_f32_close(-1.0, r.accel_z, 1e-5);
    assert_f32_close(38.0, r.temp, 1e-3);
    assert_eq!(r.status, OSCP_STATUS_OK);
}

#[test]
fn parser_euler_frame_decoded() {
    let mut parser = OscpParser::new();
    let payload = build_euler_payload(0x01, 500, 10.0, -5.0, 270.0, OSCP_STATUS_OK);
    let frames = feed_payload(&mut parser, &payload);

    assert_eq!(frames.len(), 1);
    let OscpFrame::Euler(e) = frames[0] else { panic!("expected Euler, got {:?}", frames[0]) };
    assert_f32_close(10.0, e.roll, 1e-4);
    assert_f32_close(-5.0, e.pitch, 1e-4);
    assert_f32_close(270.0, e.yaw, 1e-4);
}

#[test]
fn parser_quat_frame_decoded() {
    let mut parser = OscpParser::new();
    let payload = build_quat_payload(0x07, 2000, 0.9239, 0.3827, 0.0, 0.0, OSCP_STATUS_OK);
    let frames = feed_payload(&mut parser, &payload);

    assert_eq!(frames.len(), 1);
    let OscpFrame::Quat(q) = frames[0] else { panic!("expected Quat, got {:?}", frames[0]) };
    assert_f32_close(0.9239, q.w, 1e-4);
    assert_f32_close(0.3827, q.x, 1e-4);
    assert_f32_close(0.0, q.y, 1e-5);
    assert_f32_close(0.0, q.z, 1e-5);
}

/* Distinct values in every cell so a transposed decode would fail. */
#[test]
fn parser_rot_mat_frame_decoded() {
    let mut parser = OscpParser::new();
    let rm = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
    let payload = build_rot_mat_payload(0x03, 300, &rm, OSCP_STATUS_OK);
    let frames = feed_payload(&mut parser, &payload);

    assert_eq!(frames.len(), 1);
    let OscpFrame::RotMat(m) = frames[0] else { panic!("expected RotMat, got {:?}", frames[0]) };
    assert_eq!(m.counter, 0x03);
    assert_eq!(m.status, OSCP_STATUS_OK);
    for row in 0..3 {
        for col in 0..3 {
            assert_f32_close(rm[row][col], m.rm[row][col], 1e-5);
        }
    }
}

#[test]
fn parser_gnss_frame_decoded() {
    let mut parser = OscpParser::new();
    let payload = build_gnss_payload(0x09, 7000, 3, 12, -73.5673, 45.5017, 36000, 1200, -800, 50, 1.8, true, OSCP_STATUS_OK);
    let frames = feed_payload(&mut parser, &payload);

    assert_eq!(frames.len(), 1);
    let OscpFrame::Gnss(g) = frames[0] else { panic!("expected Gnss, got {:?}", frames[0]) };
    assert_eq!(g.gnss_fix_type, 3);
    assert_eq!(g.num_satellites, 12);
    assert_f32_close(-73.5673, g.longitude, 1e-3);
    assert_f32_close(45.5017, g.latitude, 1e-3);
    assert_eq!(g.height, 36000);
    assert_eq!(g.velocity_north, 1200);
    assert_eq!(g.velocity_east, -800); /* negative i32 */
    assert_eq!(g.velocity_down, 50);
    assert_f32_close(1.8, g.pdop, 1e-4);
    assert!(g.gnss_flags.gnss_fix_ok());
    assert!(!g.gnss_flags.invalid_llh());
}

#[test]
fn parser_debug_1_frame_decoded() {
    let mut parser = OscpParser::new();
    let payload = build_debug_1_payload(0x0D, 0xDEADBEEF, 0xCAFEBABE, OSCP_STATUS_OK);
    let frames = feed_payload(&mut parser, &payload);

    assert_eq!(frames.len(), 1);
    let OscpFrame::Debug1(d) = frames[0] else { panic!("expected Debug1, got {:?}", frames[0]) };
    assert_eq!(d.counter, 0x0D);
    assert_eq!(d.gxb, 0xDEADBEEF);
    assert_eq!(d.mzb, 0xCAFEBABE);
}

#[test]
fn parser_debug_2_frame_decoded() {
    let mut parser = OscpParser::new();
    let payload = build_debug_2_payload(0x0E, 0x11223344, 0x55667788, OSCP_STATUS_OK);
    let frames = feed_payload(&mut parser, &payload);

    assert_eq!(frames.len(), 1);
    let OscpFrame::Debug2(d) = frames[0] else { panic!("expected Debug2, got {:?}", frames[0]) };
    assert_eq!(d.counter, 0x0E);
    assert_eq!(d.mxx, 0x11223344);
    assert_eq!(d.fusion_gain, 0x55667788);
}

#[test]
fn parser_startup_frame_decoded() {
    let mut parser = OscpParser::new();
    let payload = build_startup_payload("OSCP-MK2E2", 42, 1, 2, 3, OSCP_ENABLE_RAW | OSCP_ENABLE_QUAT, OSCP_STATUS_OK);
    let frames = feed_payload(&mut parser, &payload);

    assert_eq!(frames.len(), 1);
    let OscpFrame::Startup(s) = frames[0] else { panic!("expected Startup, got {:?}", frames[0]) };
    assert_eq!(s.unit_number, 42);
    assert_eq!(s.sw_major_ver, 1);
    assert_eq!(s.sw_minor_ver, 2);
    assert_eq!(s.sw_patch_ver, 3);
    assert_eq!(s.enabled_frames, OSCP_ENABLE_RAW | OSCP_ENABLE_QUAT);
    assert_eq!(&s.mark_number, b"OSCP-MK2E2");
}

#[test]
fn parser_header_accessors() {
    let mut parser = OscpParser::new();
    let mut payload = build_quat_payload(0, 0, 1.0, 0.0, 0.0, 0.0, OSCP_STATUS_OK);
    payload.truncate(27);
    payload[0] = make_byte0(OscpFrameType::Quaternion, OscpOperatingMode::Medium, OscpMisalignmentCorr::Enabled);
    let crc = oscp_crc16(&payload);
    payload.extend_from_slice(&crc.to_le_bytes());

    let frames = feed_payload(&mut parser, &payload);
    assert_eq!(frames.len(), 1);
    let OscpFrame::Quat(q) = frames[0] else { panic!("expected Quat, got {:?}", frames[0]) };
    assert_eq!(q.header_byte.frame_type(), Ok(OscpFrameType::Quaternion));
    assert_eq!(q.header_byte.operating_mode(), Ok(OscpOperatingMode::Medium));
    assert_eq!(q.header_byte.misalignment_corr(), Ok(OscpMisalignmentCorr::Enabled));
}

/* The decoded frame's variant must be the masked type, not the raw header byte. */
#[test]
fn parser_frame_is_clean_enum_variant() {
    let mut parser = OscpParser::new();
    let mut payload = build_quat_payload(0x00, 0, 1.0, 0.0, 0.0, 0.0, OSCP_STATUS_OK);
    payload.truncate(27);
    payload[0] = make_byte0(OscpFrameType::Quaternion, OscpOperatingMode::Medium, OscpMisalignmentCorr::Enabled); /* 0x52 */
    let crc = oscp_crc16(&payload);
    payload.extend_from_slice(&crc.to_le_bytes());

    let frames = feed_payload(&mut parser, &payload);
    assert_eq!(frames.len(), 1);
    assert!(matches!(frames[0], OscpFrame::Quat(_)));
    let OscpFrame::Quat(q) = frames[0] else { unreachable!() };
    assert_f32_close(1.0, q.w, 1e-5);
}

/* --- Parser error paths --- */

#[test]
fn parser_crc_error_increments_counter() {
    let mut parser = OscpParser::new();
    let mut payload = build_raw_payload(0x01, 100, 0.0, 0.0, 0.0, 0.0, OSCP_STATUS_OK);
    payload[5] ^= 0xFF; /* corrupt after CRC was appended */
    let frames = feed_payload(&mut parser, &payload);

    assert!(frames.is_empty());
    assert_eq!(parser.stats.crc_errors, 1);
    assert_eq!(parser.stats.frames_ok, 0);
}

#[test]
fn parser_framing_error_wrong_length() {
    /* Quaternion type (2) but padded to raw length (61) -> expected 29, got 61 */
    let mut parser = OscpParser::new();
    let mut payload = vec![0u8; 59];
    payload[0] = make_byte0(OscpFrameType::Quaternion, OscpOperatingMode::Low, OscpMisalignmentCorr::Disabled);
    let crc = oscp_crc16(&payload);
    payload.extend_from_slice(&crc.to_le_bytes());
    assert_eq!(payload.len(), 61);

    let frames = feed_payload(&mut parser, &payload);
    assert!(frames.is_empty());
    assert_eq!(parser.stats.framing_errors, 1);
}

#[test]
fn parser_framing_error_too_short() {
    /* Below OSCP_FRAME_MIN_LEN -> framing error, not a crash */
    let mut parser = OscpParser::new();
    let mut payload = [0x01u8; 8];
    payload[0] = make_byte0(OscpFrameType::Euler, OscpOperatingMode::Low, OscpMisalignmentCorr::Disabled);
    let frames = feed_payload(&mut parser, &payload);

    assert!(frames.is_empty());
    assert_eq!(parser.stats.framing_errors, 1);
}

#[test]
fn parser_cobs_error_increments_counter() {
    /* A COBS overhead byte pointing past the end of the frame is invalid. */
    let mut parser = OscpParser::new();
    let bad = [0xFFu8, 0x01, 0x02]; /* overhead 0xFF but only 2 bytes follow */
    let mut frames: Vec<OscpFrame> = parser.feed(OSCP_FRAME_DELIM).into_iter().collect();
    frames.extend(parser.feed_buf(&bad));
    frames.extend(parser.feed(OSCP_FRAME_DELIM));

    assert!(frames.is_empty());
    assert_eq!(parser.stats.cobs_errors, 1);
}

#[test]
fn parser_no_frame_before_first_delimiter() {
    let mut parser = OscpParser::new();
    let garbage = [0x04u8, 0xAB, 0xCD, 0xEF];
    let frames: Vec<OscpFrame> = parser.feed_buf(&garbage).collect();

    assert!(frames.is_empty());
    assert_eq!(parser.stats.frames_ok, 0);
}

#[test]
fn parser_overflow_increments_counter() {
    let mut parser = OscpParser::new();
    let mut frames: Vec<OscpFrame> = parser.feed(OSCP_FRAME_DELIM).into_iter().collect();
    for _ in 0..=OSCP_FRAME_MAX_LEN {
        frames.extend(parser.feed(0x01));
    }

    assert!(parser.stats.overflows > 0);
    assert!(frames.is_empty());
}

/* After an overflow the parser must resync and decode the NEXT frame cleanly. */
#[test]
fn parser_recovers_after_overflow() {
    let mut parser = OscpParser::new();
    parser.feed(OSCP_FRAME_DELIM);
    for _ in 0..=OSCP_FRAME_MAX_LEN {
        parser.feed(0x01);
    }
    assert!(parser.stats.overflows > 0);

    let payload = build_euler_payload(0x42, 1, 1.0, 2.0, 3.0, OSCP_STATUS_OK);
    let frames = feed_payload(&mut parser, &payload);

    assert_eq!(frames.len(), 1);
    let OscpFrame::Euler(e) = frames[0] else { panic!("expected Euler, got {:?}", frames[0]) };
    assert_eq!(e.counter, 0x42);
}

/* After a CRC error the parser must still decode the NEXT frame. */
#[test]
fn parser_recovers_after_crc_error() {
    let mut parser = OscpParser::new();
    let mut bad = build_euler_payload(0x01, 1, 0.0, 0.0, 0.0, OSCP_STATUS_OK);
    bad[3] ^= 0xFF;
    feed_payload(&mut parser, &bad);
    assert_eq!(parser.stats.crc_errors, 1);

    let good = build_euler_payload(0x02, 2, 7.0, 8.0, 9.0, OSCP_STATUS_OK);
    let frames = feed_payload(&mut parser, &good);

    assert_eq!(frames.len(), 1);
    let OscpFrame::Euler(e) = frames[0] else { panic!("expected Euler, got {:?}", frames[0]) };
    assert_eq!(e.counter, 0x02);
    assert_eq!(parser.stats.frames_ok, 1);
}

#[test]
fn parser_reset_clears_partial_frame() {
    let mut parser = OscpParser::new();

    /* Sync, then feed half a frame */
    parser.feed(OSCP_FRAME_DELIM);
    for &b in &[0x05u8, 0x06, 0x07] {
        parser.feed(b);
    }

    /* Reset drops the partial frame and de-syncs */
    parser.reset();
    assert_eq!(parser.stats.frames_ok, 0); /* stats survive the reset */

    /* Parser re-syncs and decodes the next complete frame */
    let payload = build_euler_payload(0x77, 5, 1.0, 1.0, 1.0, OSCP_STATUS_OK);
    let frames = feed_payload(&mut parser, &payload);

    assert_eq!(frames.len(), 1);
    let OscpFrame::Euler(e) = frames[0] else { panic!("expected Euler, got {:?}", frames[0]) };
    assert_eq!(e.counter, 0x77);
}

/* --- Parser streaming behaviour --- */

#[test]
fn parser_byte_by_byte_quat() {
    let mut parser = OscpParser::new();
    let payload = build_quat_payload(0xBB, 9999, 0.7071, 0.7071, 0.0, 0.0, OSCP_STATUS_OK);
    let wire = cobs_wrap(&payload);

    let mut frames: Vec<OscpFrame> = parser.feed(OSCP_FRAME_DELIM).into_iter().collect();
    for &b in &wire {
        frames.extend(parser.feed(b));
    }

    assert_eq!(frames.len(), 1);
    let OscpFrame::Quat(q) = frames[0] else { panic!("expected Quat, got {:?}", frames[0]) };
    assert_eq!(q.counter, 0xBB);
    assert_f32_close(0.7071, q.w, 1e-4);
}

/* feed() one byte at a time and feed_buf() in bulk must be equivalent. */
#[test]
fn parser_feed_buf_equals_feed_byte() {
    let payload = build_quat_payload(0x55, 77, 0.5, 0.5, 0.5, 0.5, OSCP_STATUS_OK);
    let wire = cobs_wrap(&payload);

    let mut p1 = OscpParser::new();
    p1.feed(OSCP_FRAME_DELIM);
    let mut byte_wise = Vec::new();
    for &b in &wire {
        byte_wise.extend(p1.feed(b));
    }

    let mut p2 = OscpParser::new();
    p2.feed(OSCP_FRAME_DELIM);
    let buf_wise: Vec<OscpFrame> = p2.feed_buf(&wire).collect();

    assert_eq!(byte_wise.len(), 1);
    assert_eq!(buf_wise.len(), 1);
    assert_eq!(byte_wise[0], buf_wise[0]);
    assert_eq!(p1.stats.frames_ok, 1);
    assert_eq!(p2.stats.frames_ok, 1);
}

#[test]
fn parser_two_consecutive_frames() {
    let mut parser = OscpParser::new();
    let raw = build_raw_payload(0x01, 100, 0.0, 0.0, -1.0, 25.0, OSCP_STATUS_OK);
    let quat = build_quat_payload(0x02, 200, 1.0, 0.0, 0.0, 0.0, OSCP_STATUS_OK);

    let mut wire = cobs_wrap(&raw);
    wire.extend(cobs_wrap(&quat));

    let mut frames: Vec<OscpFrame> = parser.feed(OSCP_FRAME_DELIM).into_iter().collect();
    frames.extend(parser.feed_buf(&wire));

    assert_eq!(frames.len(), 2);
    assert_eq!(parser.stats.frames_ok, 2);
    assert!(matches!(frames[1], OscpFrame::Quat(_)));
}

#[test]
fn parser_back_to_back_delimiters_ignored() {
    /* Empty frames (0x00 0x00) must not produce frames or errors. */
    let mut parser = OscpParser::new();
    let mut frames = Vec::new();
    for _ in 0..5 {
        frames.extend(parser.feed(OSCP_FRAME_DELIM));
    }

    let payload = build_euler_payload(0x09, 9, 1.0, 1.0, 1.0, OSCP_STATUS_OK);
    frames.extend(feed_payload(&mut parser, &payload));

    assert_eq!(frames.len(), 1);
    assert_eq!(parser.stats.frames_ok, 1);
    assert_eq!(parser.stats.framing_errors, 0);
    assert_eq!(parser.stats.cobs_errors, 0);
}

#[test]
fn parser_status_byte_mems_error() {
    let mut parser = OscpParser::new();
    let payload = build_quat_payload(0x00, 0, 1.0, 0.0, 0.0, 0.0, OSCP_STATUS_MEMS_ERR);
    let frames = feed_payload(&mut parser, &payload);

    assert_eq!(frames.len(), 1);
    let OscpFrame::Quat(q) = frames[0] else { panic!("expected Quat, got {:?}", frames[0]) };
    assert_eq!(q.status & OSCP_STATUS_MEMS_ERR, OSCP_STATUS_MEMS_ERR);
}

#[test]
fn parser_status_byte_multiple_flags() {
    let mut parser = OscpParser::new();
    let st = OSCP_STATUS_OVERRUN | OSCP_STATUS_OG_ERR;
    let payload = build_quat_payload(0x00, 0, 1.0, 0.0, 0.0, 0.0, st);
    let frames = feed_payload(&mut parser, &payload);

    assert_eq!(frames.len(), 1);
    let OscpFrame::Quat(q) = frames[0] else { panic!("expected Quat, got {:?}", frames[0]) };
    assert_eq!(q.status & OSCP_STATUS_OVERRUN, OSCP_STATUS_OVERRUN);
    assert_eq!(q.status & OSCP_STATUS_OG_ERR, OSCP_STATUS_OG_ERR);
    assert_eq!(q.status & OSCP_STATUS_MEMS_ERR, 0);
}

#[test]
fn parser_status_byte_ok_is_zero() {
    let mut parser = OscpParser::new();
    let payload = build_quat_payload(0x00, 0, 1.0, 0.0, 0.0, 0.0, OSCP_STATUS_OK);
    let frames = feed_payload(&mut parser, &payload);

    assert_eq!(frames.len(), 1);
    let OscpFrame::Quat(q) = frames[0] else { panic!("expected Quat, got {:?}", frames[0]) };
    assert_eq!(q.status, OSCP_STATUS_OK);
}

#[test]
fn parser_stats_frames_ok() {
    let mut parser = OscpParser::new();
    parser.feed(OSCP_FRAME_DELIM);

    let mut frames = Vec::new();
    for i in 0..5u8 {
        let payload = build_quat_payload(i, (i as u64) * 10, 1.0, 0.0, 0.0, 0.0, OSCP_STATUS_OK);
        let wire = cobs_wrap(&payload);
        frames.extend(parser.feed_buf(&wire));
    }

    assert_eq!(frames.len(), 5);
    assert_eq!(parser.stats.frames_ok, 5);
    assert_eq!(parser.stats.crc_errors, 0);
    assert_eq!(parser.stats.cobs_errors, 0);
}

/* --- Command response frames (UART only: ASCII over COBS) --- */

#[test]
fn parser_cmd_success_delivered() {
    let mut parser = OscpParser::new();
    let frames = feed_payload(&mut parser, b"OSCP-MK2M2: Command succeed.\r\n");

    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0], OscpFrame::CmdResponse(OscpFrameCmd::Success));
    assert_eq!(parser.stats.frames_ok, 1);
    assert_eq!(parser.stats.framing_errors, 0);
}

#[test]
fn parser_cmd_failed_delivered() {
    let mut parser = OscpParser::new();
    let frames = feed_payload(&mut parser, b"OSCP-MK2E2: Command failed.\r\n");

    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0], OscpFrame::CmdResponse(OscpFrameCmd::Failed));
}

#[test]
fn parser_cmd_unknown_delivered() {
    let mut parser = OscpParser::new();
    let frames = feed_payload(&mut parser, b"OSCP-MK2M2: This command is erroneous.\r\n");

    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0], OscpFrame::CmdResponse(OscpFrameCmd::Unknown));
}

#[test]
fn parser_cmd_not_impl_delivered() {
    let mut parser = OscpParser::new();
    let frames = feed_payload(&mut parser, b"OSCP-MK2E2: This command is not implemented yet.\r\n");

    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0], OscpFrame::CmdResponse(OscpFrameCmd::NotImpl));
}

#[test]
fn parser_cmd_success_suffix_only_matches() {
    let mut parser = OscpParser::new();
    let frames = feed_payload(&mut parser, b"Command succeed.\r\n");

    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0], OscpFrame::CmdResponse(OscpFrameCmd::Success));
}

/* A response whose suffix does not match any table entry must not be delivered. */
#[test]
fn parser_unrecognised_ascii_not_delivered() {
    let mut parser = OscpParser::new();
    let frames = feed_payload(&mut parser, b"Some other text.\r\n");

    assert!(frames.is_empty());
    assert_eq!(parser.stats.frames_ok, 0);
    assert_eq!(parser.stats.framing_errors, 1);
}

#[test]
fn parser_full_session() {
    let mut parser = OscpParser::new();
    let mut frames = Vec::new();

    let euler1 = build_euler_payload(0x01, 100, 1.0, 2.0, 3.0, OSCP_STATUS_OK);
    frames.extend(feed_payload(&mut parser, &euler1));
    assert_eq!(frames.len(), 1);
    assert!(matches!(frames[0], OscpFrame::Euler(_)));

    frames.extend(feed_payload(&mut parser, b"OSCP-MK2M2: Command succeed.\r\n"));
    assert_eq!(frames.len(), 2);
    assert_eq!(frames[1], OscpFrame::CmdResponse(OscpFrameCmd::Success));

    frames.extend(feed_payload(&mut parser, b"OSCP-MK2M2: Command failed.\r\n"));
    assert_eq!(frames.len(), 3);
    assert_eq!(frames[2], OscpFrame::CmdResponse(OscpFrameCmd::Failed));

    let euler2 = build_euler_payload(0x02, 200, 4.0, 5.0, 6.0, OSCP_STATUS_OK);
    frames.extend(feed_payload(&mut parser, &euler2));
    assert_eq!(frames.len(), 4);
    assert!(matches!(frames[3], OscpFrame::Euler(_)));

    assert_eq!(parser.stats.frames_ok, 4);
    assert_eq!(parser.stats.crc_errors, 0);
    assert_eq!(parser.stats.framing_errors, 0);
}

/* ==========================================================================
 * TRANSPORT B — Message decode (CAN-FD)
 *
 * No parser, no COBS. Payloads may be padded up to a DLC bucket; the CRC
 * must be checked over the frame length only, never over the padding.
 * ========================================================================*/

/* Copy a payload into a CAN-FD message buffer and fill the padding with
 * noise, so any decoder that reads past the frame length will be caught. */
fn pad_to_dlc(payload: &[u8], dlc: usize) -> Vec<u8> {
    let mut msg = vec![0xAAu8; dlc];
    msg[..payload.len()].copy_from_slice(payload);
    msg
}

#[test]
fn can_decode_euler_exact_length() {
    let payload = build_euler_payload(0x2A, 123456, 1.5, -2.5, 90.0, OSCP_STATUS_OK);
    let f = oscp_frame_decode(&payload).expect("decode failed");

    let OscpFrame::Euler(e) = f else { panic!("expected Euler, got {f:?}") };
    assert_eq!(e.counter, 0x2A);
    assert_f32_close(90.0, e.yaw, 1e-4);
}

/* THE critical CAN-FD case: 25-byte frame inside a 32-byte DLC bucket. */
#[test]
fn can_decode_euler_with_dlc_padding() {
    let payload = build_euler_payload(0x2A, 123456, 1.5, -2.5, 90.0, OSCP_STATUS_OK);
    let msg = pad_to_dlc(&payload, 32);

    let f = oscp_frame_decode(&msg).expect("DLC-padded frame must decode; CRC must cover the frame only");
    let OscpFrame::Euler(e) = f else { panic!("expected Euler, got {f:?}") };
    assert_eq!(e.counter, 0x2A);
    assert_f32_close(1.5, e.roll, 1e-4);
    assert_f32_close(-2.5, e.pitch, 1e-4);
    assert_f32_close(90.0, e.yaw, 1e-4);
}

/* Padding content must be irrelevant: 0x00 padding and 0xFF padding decode alike. */
#[test]
fn can_decode_padding_content_irrelevant() {
    let payload = build_quat_payload(0x11, 7, 1.0, 0.0, 0.0, 0.0, OSCP_STATUS_OK);

    let mut msg_a = vec![0x00u8; 32];
    msg_a[..payload.len()].copy_from_slice(&payload);
    let mut msg_b = vec![0xFFu8; 32];
    msg_b[..payload.len()].copy_from_slice(&payload);

    let fa = oscp_frame_decode(&msg_a).expect("decode a failed");
    let fb = oscp_frame_decode(&msg_b).expect("decode b failed");
    assert_eq!(fa, fb);
}

#[test]
fn can_decode_raw_in_64_byte_message() {
    let payload = build_raw_payload(0x33, 555, 1.0, -1.0, 9.81, 42.0, OSCP_STATUS_OK);
    let msg = pad_to_dlc(&payload, 64);

    let f = oscp_frame_decode(&msg).expect("decode failed");
    let OscpFrame::Raw(r) = f else { panic!("expected Raw, got {f:?}") };
    assert_eq!(r.timestamp_ms, 555);
    assert_f32_close(9.81, r.accel_z, 1e-4);
    assert_f32_close(42.0, r.temp, 1e-3);
}

#[test]
fn can_decode_gnss_full_64() {
    /* GNSS is exactly 64 bytes: no padding possible. */
    let payload = build_gnss_payload(0x04, 8000, 3, 9, -73.0, 45.0, 1000, 10, -20, 30, 2.1, true, OSCP_STATUS_OK);

    let f = oscp_frame_decode(&payload).expect("decode failed");
    let OscpFrame::Gnss(g) = f else { panic!("expected Gnss, got {f:?}") };
    assert_eq!(g.velocity_east, -20);
    assert_eq!(g.num_satellites, 9);
}

#[test]
fn can_decode_all_frame_types() {
    let raw = build_raw_payload(1, 1, 0.0, 0.0, 0.0, 0.0, 0);
    assert!(matches!(oscp_frame_decode(&raw).unwrap(), OscpFrame::Raw(_)));

    let euler = build_euler_payload(1, 1, 0.0, 0.0, 0.0, 0);
    assert!(matches!(oscp_frame_decode(&euler).unwrap(), OscpFrame::Euler(_)));

    let quat = build_quat_payload(1, 1, 1.0, 0.0, 0.0, 0.0, 0);
    assert!(matches!(oscp_frame_decode(&quat).unwrap(), OscpFrame::Quat(_)));

    let rm = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let rot_mat = build_rot_mat_payload(1, 1, &rm, 0);
    assert!(matches!(oscp_frame_decode(&rot_mat).unwrap(), OscpFrame::RotMat(_)));

    let gnss = build_gnss_payload(1, 1, 3, 5, 0.0, 0.0, 0, 0, 0, 0, 1.0, true, 0);
    assert!(matches!(oscp_frame_decode(&gnss).unwrap(), OscpFrame::Gnss(_)));

    let debug1 = build_debug_1_payload(1, 0xAA, 0xBB, 0);
    assert!(matches!(oscp_frame_decode(&debug1).unwrap(), OscpFrame::Debug1(_)));

    let debug2 = build_debug_2_payload(1, 0xCC, 0xDD, 0);
    assert!(matches!(oscp_frame_decode(&debug2).unwrap(), OscpFrame::Debug2(_)));

    let startup = build_startup_payload("OSCP-MK2M2", 1, 0, 1, 0, OSCP_ENABLE_RAW, 0);
    assert!(matches!(oscp_frame_decode(&startup).unwrap(), OscpFrame::Startup(_)));
}

/* --- CAN-FD error paths --- */

#[test]
fn can_decode_rejects_truncated() {
    let payload = build_euler_payload(1, 1, 0.0, 0.0, 0.0, 0);

    /* len < frame length -> reject, never read past the buffer */
    assert!(oscp_frame_decode(&payload[..24]).is_err());
    assert!(oscp_frame_decode(&payload[..10]).is_err());
    assert!(oscp_frame_decode(&payload[..1]).is_err());
}

#[test]
fn can_decode_rejects_zero_length() {
    assert!(oscp_frame_decode(&[]).is_err());
}

#[test]
fn can_decode_rejects_bad_crc() {
    let mut payload = build_euler_payload(1, 1, 1.0, 2.0, 3.0, 0);
    payload[5] ^= 0xFF;
    assert_eq!(oscp_frame_decode(&payload), Err(OscpParseError::CrcMismatch));
}

/* Corruption anywhere in the frame body must be caught, padding must not matter. */
#[test]
fn can_decode_rejects_bad_crc_under_padding() {
    let payload = build_quat_payload(1, 1, 1.0, 0.0, 0.0, 0.0, 0);
    let mut msg = pad_to_dlc(&payload, 32);
    msg[3] ^= 0x01; /* corrupt inside the frame, not the padding */

    assert_eq!(oscp_frame_decode(&msg), Err(OscpParseError::CrcMismatch));
}

/* oscp_frame_decode must not touch COBS: a COBS-encoded buffer should fail. */
#[test]
fn can_decode_does_not_expect_cobs() {
    let payload = build_euler_payload(1, 1, 0.0, 0.0, 0.0, 0);
    let wire = cobs_wrap(&payload); /* includes the trailing 0x00 delimiter */

    /* COBS-encoded bytes are not a valid raw frame -> CRC (or type) rejects it */
    assert!(oscp_frame_decode(&wire).is_err());
}

/* ==========================================================================
 * EQUIVALENCE — both transports share dispatch_frame, so identical payload
 * bytes must produce equal OscpFrame values on either path.
 * ========================================================================*/

fn assert_transports_agree(payload: &[u8]) {
    /* Path A: UART parser (COBS-wrapped) */
    let mut parser = OscpParser::new();
    let frames = feed_payload(&mut parser, payload);
    assert_eq!(frames.len(), 1, "UART path did not deliver a frame");
    let via_uart = frames[0];

    /* Path B: CAN-FD direct decode, with padding to prove it is ignored */
    let mut msg = vec![0xA5u8; 72];
    msg[..payload.len()].copy_from_slice(payload);
    let via_can = oscp_frame_decode(&msg).expect("CAN path did not decode the frame");

    assert_eq!(via_uart, via_can, "decoded frame differs between UART and CAN-FD");
}

#[test]
fn equivalence_raw() {
    let p = build_raw_payload(0x7F, 987654, -1.25, 2.5, -9.81, 21.5, OSCP_STATUS_OVERRUN);
    assert_transports_agree(&p);
}

#[test]
fn equivalence_euler() {
    let p = build_euler_payload(0x08, 4321, -179.9, 89.9, 359.9, OSCP_STATUS_OK);
    assert_transports_agree(&p);
}

#[test]
fn equivalence_quat() {
    let p = build_quat_payload(0x09, 1, 0.7071, 0.0, 0.7071, 0.0, OSCP_STATUS_OK);
    assert_transports_agree(&p);
}

#[test]
fn equivalence_startup() {
    let p = build_startup_payload("OSCP-MK2Z1", 1234, 9, 8, 7, OSCP_ENABLE_GNSS, OSCP_STATUS_OK);
    assert_transports_agree(&p);
}

/* ==========================================================================
 * COMMANDS
 *
 * The C reference for this section was cut off mid-file when it was pasted
 * in, so this is written directly from oscp_imu.rs's own implementation
 * rather than ported line-for-line like the sections above.
 * ========================================================================*/

/* UART: COBS-encoded, delimiter-terminated, no interior 0x00. Decodes the
 * command back and compares against the expected ASCII. */
fn assert_uart_cmd(buf: &[u8], expected_ascii: &str) {
    let (last, body) = buf.split_last().expect("empty command buffer");
    assert_eq!(*last, OSCP_FRAME_DELIM, "missing 0x00 delimiter");
    for &b in body {
        assert_ne!(b, 0x00, "0x00 inside COBS-encoded command");
    }
    let mut decoded = [0u8; 64];
    let dec_len = oscp_cobs_decode(body, &mut decoded).expect("cobs decode failed");
    assert_eq!(&decoded[..dec_len], expected_ascii.as_bytes());
}

/* CAN: raw ASCII, no COBS, no delimiter. */
fn assert_can_cmd(buf: &[u8], expected_ascii: &str) {
    assert_eq!(buf, expected_ascii.as_bytes());
}

fn assert_cmd_both_transports<F>(expected_ascii: &str, mut build: F)
where
    F: FnMut(&mut [u8], OscpTransport) -> Result<usize, OscpErr>,
{
    let mut uart_buf = [0u8; 64];
    let uart_len = build(&mut uart_buf, OscpTransport::Rs422).expect("rs422 build failed");
    assert_uart_cmd(&uart_buf[..uart_len], expected_ascii);

    let mut can_buf = [0u8; 64];
    let can_len = build(&mut can_buf, OscpTransport::Canfd).expect("canfd build failed");
    assert_can_cmd(&can_buf[..can_len], expected_ascii);
}

#[test]
fn cmd_reset() {
    assert_cmd_both_transports("RESET\r\n", |buf, t| oscp_cmd_reset(buf, t));
}

#[test]
fn cmd_exit() {
    assert_cmd_both_transports("EXIT\r\n", |buf, t| oscp_cmd_exit(buf, t));
}

#[test]
fn cmd_refs() {
    assert_cmd_both_transports("REFS\r\n", |buf, t| oscp_cmd_refs(buf, t));
}

#[test]
fn cmd_config() {
    assert_cmd_both_transports("CONFIG\r\n", |buf, t| oscp_cmd_config(buf, t));
}

#[test]
fn cmd_suf() {
    assert_cmd_both_transports("SUF\r\n", |buf, t| oscp_cmd_suf(buf, t));
}

#[test]
fn cmd_of_all_selectors() {
    for (sel, ch) in [
        (OscpFrameSel::Raw, 'R'),
        (OscpFrameSel::Euler, 'E'),
        (OscpFrameSel::Quaternion, 'Q'),
        (OscpFrameSel::RotMatrix, 'M'),
        (OscpFrameSel::Gnss, 'G'),
        (OscpFrameSel::Debug, 'D'),
    ] {
        let expected = format!("OF{ch}\r\n");
        assert_cmd_both_transports(&expected, |buf, t| oscp_cmd_of(buf, sel, t));
    }
}

#[test]
fn cmd_om_all_modes() {
    for (sel, ch) in [(OscpOmSel::Idle, 'I'), (OscpOmSel::Low, 'L'), (OscpOmSel::Medium, 'M')] {
        let expected = format!("OM{ch}\r\n");
        assert_cmd_both_transports(&expected, |buf, t| oscp_cmd_om(buf, sel, t));
    }
}

#[test]
fn cmd_enable_oft_all_valid_selectors() {
    for (sel, ch) in [
        (OscpFrameSel::Raw, 'R'),
        (OscpFrameSel::Euler, 'E'),
        (OscpFrameSel::Quaternion, 'Q'),
        (OscpFrameSel::RotMatrix, 'M'),
        (OscpFrameSel::Gnss, 'G'),
    ] {
        let expected = format!("EOFT{ch}\r\n");
        assert_cmd_both_transports(&expected, |buf, t| oscp_cmd_enable_oft(buf, sel, t));
    }
}

/* Unlike oscp_cmd_of, Debug is not a valid selector for enable/disable OFT. */
#[test]
fn cmd_enable_oft_rejects_debug() {
    let mut buf = [0u8; 16];
    assert_eq!(oscp_cmd_enable_oft(&mut buf, OscpFrameSel::Debug, OscpTransport::Rs422), Err(OscpErr::ErrInvalid));
    assert_eq!(oscp_cmd_enable_oft(&mut buf, OscpFrameSel::Debug, OscpTransport::Canfd), Err(OscpErr::ErrInvalid));
}

#[test]
fn cmd_disable_oft_all_valid_selectors() {
    for (sel, ch) in [
        (OscpFrameSel::Raw, 'R'),
        (OscpFrameSel::Euler, 'E'),
        (OscpFrameSel::Quaternion, 'Q'),
        (OscpFrameSel::RotMatrix, 'M'),
        (OscpFrameSel::Gnss, 'G'),
    ] {
        let expected = format!("DOFT{ch}\r\n");
        assert_cmd_both_transports(&expected, |buf, t| oscp_cmd_disable_oft(buf, sel, t));
    }
}

#[test]
fn cmd_disable_oft_rejects_debug() {
    let mut buf = [0u8; 16];
    assert_eq!(oscp_cmd_disable_oft(&mut buf, OscpFrameSel::Debug, OscpTransport::Rs422), Err(OscpErr::ErrInvalid));
}

// REVERTED (09-09-2026): an unpadded format was tried and tested here
// briefly, but got a real "This command is erroneous" response from live
// hardware — worse than the zero-padded form's silent non-application. See
// oscp_cmd_drg's own doc comment. Back to pinning the original zero-padded
// format.
#[test]
fn cmd_drg_all_ranges() {
    for (dr, dps) in [
        (OscpGyroDr::Dps125, 125),
        (OscpGyroDr::Dps250, 250),
        (OscpGyroDr::Dps500, 500),
        (OscpGyroDr::Dps1000, 1000),
        (OscpGyroDr::Dps2000, 2000),
        (OscpGyroDr::Dps4000, 4000),
    ] {
        let expected = format!("DRG{dps:04}\r\n");
        assert_cmd_both_transports(&expected, |buf, t| oscp_cmd_drg(buf, dr, t));
    }
}

// Pinned against the OSCP protocol doc's "Mapping of MEMS Gyroscope/
// Accelerometer/Inclinometer Dynamic Ranges" tables (09-09-2026) — this is
// the READBACK code (from a Startup frame's DynRangeCfg/InclAhrsCfg), a
// separate, non-sequential mapping from the literal dps/g values
// OscpGyroDr/OscpAccelDr/OscpInclDr's own discriminants use for the
// DRG/DRA/DRI write commands above.
#[test]
fn gyro_dr_from_dyn_range_code_matches_the_protocol_doc_table() {
    assert_eq!(OscpGyroDr::from_dyn_range_code(0b0000), Ok(OscpGyroDr::Dps250));
    assert_eq!(OscpGyroDr::from_dyn_range_code(0b0001), Ok(OscpGyroDr::Dps4000));
    assert_eq!(OscpGyroDr::from_dyn_range_code(0b0010), Ok(OscpGyroDr::Dps125));
    assert_eq!(OscpGyroDr::from_dyn_range_code(0b0100), Ok(OscpGyroDr::Dps500));
    assert_eq!(OscpGyroDr::from_dyn_range_code(0b1000), Ok(OscpGyroDr::Dps1000));
    assert_eq!(OscpGyroDr::from_dyn_range_code(0b1100), Ok(OscpGyroDr::Dps2000));
    assert_eq!(OscpGyroDr::from_dyn_range_code(0b0011), Err(0b0011));
}

#[test]
fn accel_dr_from_dyn_range_code_matches_the_protocol_doc_table() {
    assert_eq!(OscpAccelDr::from_dyn_range_code(0b00), Ok(OscpAccelDr::G2));
    assert_eq!(OscpAccelDr::from_dyn_range_code(0b01), Ok(OscpAccelDr::G16));
    assert_eq!(OscpAccelDr::from_dyn_range_code(0b10), Ok(OscpAccelDr::G4));
    assert_eq!(OscpAccelDr::from_dyn_range_code(0b11), Ok(OscpAccelDr::G8));
    assert_eq!(OscpAccelDr::from_dyn_range_code(0b100), Err(0b100));
}

#[test]
fn incl_dr_from_dyn_range_code_matches_the_protocol_doc_table() {
    assert_eq!(OscpInclDr::from_dyn_range_code(0b00), Ok(OscpInclDr::G0_5));
    assert_eq!(OscpInclDr::from_dyn_range_code(0b01), Ok(OscpInclDr::G3_0));
    assert_eq!(OscpInclDr::from_dyn_range_code(0b10), Ok(OscpInclDr::G1_0));
    assert_eq!(OscpInclDr::from_dyn_range_code(0b11), Ok(OscpInclDr::G2_0));
    assert_eq!(OscpInclDr::from_dyn_range_code(0b100), Err(0b100));
}

// DynRangeCfg/InclAhrsCfg only extract the raw nibble — this pins that the
// bit positions match the doc's Startup byte 17 / byte 20 layout, end to
// end (packed byte -> nibble -> decoded range).
#[test]
fn dyn_range_cfg_extracts_the_correct_nibbles() {
    // accel=0b0011 (8g) in [7:4], gyro=0b0010 (125dps) in [3:0] -> 0x32
    let cfg = DynRangeCfg::from_byte(0x32);
    assert_eq!(cfg.accel_dr(), 0b0011);
    assert_eq!(cfg.gyro_dr(), 0b0010);
    assert_eq!(OscpAccelDr::from_dyn_range_code(cfg.accel_dr()), Ok(OscpAccelDr::G8));
    assert_eq!(OscpGyroDr::from_dyn_range_code(cfg.gyro_dr()), Ok(OscpGyroDr::Dps125));
}

#[test]
fn incl_ahrs_cfg_extracts_the_correct_incl_range_nibble() {
    // heading_src=0b01 [7:6], convention=0b10 [5:4], incl_dr=0b01 (3.0g) [3:0] -> 0b01100001 = 0x61
    let cfg = InclAhrsCfg::from_byte(0x61);
    assert_eq!(cfg.incl_dr(), 0b0001);
    assert_eq!(cfg.ahrs_convention(), 0b10);
    assert_eq!(cfg.ahrs_heading_src(), 0b01);
    assert_eq!(OscpInclDr::from_dyn_range_code(cfg.incl_dr()), Ok(OscpInclDr::G3_0));
}

#[test]
fn cmd_dra_all_ranges() {
    for (dr, g) in [(OscpAccelDr::G2, 2), (OscpAccelDr::G4, 4), (OscpAccelDr::G8, 8), (OscpAccelDr::G16, 16)] {
        let expected = format!("DRA{g:02}\r\n");
        assert_cmd_both_transports(&expected, |buf, t| oscp_cmd_dra(buf, dr, t));
    }
}

#[test]
fn cmd_dri_all_ranges() {
    for (dr, expected) in [
        (OscpInclDr::G0_5, "DRI0.5\r\n"),
        (OscpInclDr::G1_0, "DRI1.0\r\n"),
        (OscpInclDr::G2_0, "DRI2.0\r\n"),
        (OscpInclDr::G3_0, "DRI3.0\r\n"),
    ] {
        assert_cmd_both_transports(expected, |buf, t| oscp_cmd_dri(buf, dr, t));
    }
}

#[test]
fn cmd_enable_mcorr() {
    assert_cmd_both_transports("EMCORR\r\n", |buf, t| oscp_cmd_enable_mcorr(buf, t));
}

#[test]
fn cmd_disable_mcorr() {
    assert_cmd_both_transports("DMCORR\r\n", |buf, t| oscp_cmd_disable_mcorr(buf, t));
}

#[test]
fn cmd_wr_encodes_mnemonic_and_hex_value() {
    assert_cmd_both_transports("WRGXB000003E8\r\n", |buf, t| oscp_cmd_wr(buf, OscpUsrReg::Gxb, 1000, t));
    assert_cmd_both_transports("WRAHPFFFFFFFF\r\n", |buf, t| oscp_cmd_wr(buf, OscpUsrReg::Ahp, u32::MAX, t));
}

/* Every register mnemonic must round-trip through OSCP_USR_REG_STR correctly. */
#[test]
fn cmd_wr_all_registers_use_correct_mnemonic() {
    let regs = [
        (OscpUsrReg::Gxb, "GXB"), (OscpUsrReg::Gyb, "GYB"), (OscpUsrReg::Gzb, "GZB"), (OscpUsrReg::Gob, "GOB"),
        (OscpUsrReg::Axb, "AXB"), (OscpUsrReg::Ayb, "AYB"), (OscpUsrReg::Azb, "AZB"),
        (OscpUsrReg::Ixb, "IXB"), (OscpUsrReg::Iyb, "IYB"),
        (OscpUsrReg::Mxb, "MXB"), (OscpUsrReg::Myb, "MYB"), (OscpUsrReg::Mzb, "MZB"),
        (OscpUsrReg::Mxx, "MXX"), (OscpUsrReg::Myx, "MYX"), (OscpUsrReg::Mzx, "MZX"),
        (OscpUsrReg::Mxy, "MXY"), (OscpUsrReg::Myy, "MYY"), (OscpUsrReg::Mzy, "MZY"),
        (OscpUsrReg::Mxz, "MXZ"), (OscpUsrReg::Myz, "MYZ"), (OscpUsrReg::Mzz, "MZZ"),
        (OscpUsrReg::Fco, "FCO"), (OscpUsrReg::Fhs, "FHS"), (OscpUsrReg::Frt, "FRT"),
        (OscpUsrReg::Fga, "FGA"), (OscpUsrReg::Far, "FAR"), (OscpUsrReg::Fmr, "FMR"),
        (OscpUsrReg::Gfi, "GFI"), (OscpUsrReg::Glp, "GLP"), (OscpUsrReg::Ghp, "GHP"),
        (OscpUsrReg::Afi, "AFI"), (OscpUsrReg::Alp, "ALP"), (OscpUsrReg::Ahp, "AHP"),
    ];
    for (reg, mnemonic) in regs {
        let expected = format!("WR{mnemonic}00000000\r\n");
        assert_cmd_both_transports(&expected, |buf, t| oscp_cmd_wr(buf, reg, 0, t));
    }
}

#[test]
fn cmd_save() {
    assert_cmd_both_transports("SAVE\r\n", |buf, t| oscp_cmd_save(buf, t));
}

/* --- Buffer sizing --- */

#[test]
fn cmd_canfd_rejects_buffer_too_small() {
    let mut buf = [0u8; 3]; /* "RESET\r\n" is 7 bytes */
    assert_eq!(oscp_cmd_reset(&mut buf, OscpTransport::Canfd), Err(OscpErr::Err));
}

#[test]
fn cmd_rs422_rejects_empty_buffer() {
    let mut buf: [u8; 0] = [];
    assert_eq!(oscp_cmd_reset(&mut buf, OscpTransport::Rs422), Err(OscpErr::Err));
}

#[test]
fn cmd_wr_buffer_sized_exactly_15_succeeds() {
    /* "WR" + 3-char mnemonic + 8 hex digits + "\r\n" = 15 bytes exactly for CAN-FD */
    let mut buf = [0u8; 15];
    assert_eq!(oscp_cmd_wr(&mut buf, OscpUsrReg::Gxb, 0, OscpTransport::Canfd), Ok(15));
}
