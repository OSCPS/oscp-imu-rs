pub const OSCP_FRAME_DELIM: u8 = 0x00;
pub const OSCP_FRAME_MIN_LEN: usize = 25;
pub const OSCP_FRAME_MAX_LEN: usize = 72;

pub const OSCP_STATUS_OK: u8 = 0x00;        /* No bits are set */
pub const OSCP_STATUS_OVERRUN: u8 = 0x01;   /* IMU Real Time Controller Overrun */
pub const OSCP_STATUS_MEMS_ERR: u8 = 0x02;  /* MEMS Sensors Error */
pub const OSCP_STATUS_INCL_ERR: u8 = 0x04;  /* Inclinometer Error */
pub const OSCP_STATUS_MAG_ERR: u8 = 0x08;   /* Magnetometer Error */
pub const OSCP_STATUS_TEMP_ERR: u8 = 0x10;  /* Temperature Sensor Error */
pub const OSCP_STATUS_GNSS_ERR: u8 = 0x20;  /* GNSS Error */
pub const OSCP_STATUS_OG_ERR: u8 = 0x40;    /* Optical Gyroscope Error */

pub const OSCP_ENABLE_RAW: u8 = 0x01;
pub const OSCP_ENABLE_EULER: u8 = 0x02;
pub const OSCP_ENABLE_QUAT: u8 = 0x04;
pub const OSCP_ENABLE_ROT_MAT: u8 = 0x08;
pub const OSCP_ENABLE_GNSS: u8 = 0x10;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OscpFrameType {
    Raw         = 0x00, /* Raw Operating Frame */
    Euler       = 0x01, /* AHRS Euler Angles Operating Frame */
    Quaternion  = 0x02, /* AHRS Quaternions Operating Frame */
    RotMatrix   = 0x03, /* AHRS Rotation Matrix Operating Frame */
    Gnss        = 0x04, /* GNSS Operating Frame */
    Debug1      = 0x05, /* Debug Operating Frame 1 */
    Debug2      = 0x06, /* Debug Operating Frame 2 */
    Startup     = 0x07, /* Startup Frame */
}

impl TryFrom<u8> for OscpFrameType {
    type Error = u8;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(Self::Raw),
            0x01 => Ok(Self::Euler),
            0x02 => Ok(Self::Quaternion),
            0x03 => Ok(Self::RotMatrix),
            0x04 => Ok(Self::Gnss),
            0x05 => Ok(Self::Debug1),
            0x06 => Ok(Self::Debug2),
            0x07 => Ok(Self::Startup),
            other => Err(other),
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OscpFrameCmd {
    Success  = 0xA0, /* Command executed successfully */
    Failed   = 0xA1, /* Command rejected by the unit */
    Unknown  = 0xA2, /* Command is not recognized by the unit */
    NotImpl  = 0xA3, /* Command is recognized but not implemented yet */
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OscpOperatingMode {
    Idle   = 0x00,
    Low    = 0x01,
    Medium = 0x02,
}

impl TryFrom<u8> for OscpOperatingMode {
    type Error = u8;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(Self::Idle),
            0x01 => Ok(Self::Low),
            0x02 => Ok(Self::Medium),
            other => Err(other),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OscpMisalignmentCorr {
    Disabled = 0x00,
    Enabled  = 0x01,
}

impl TryFrom<u8> for OscpMisalignmentCorr {
    type Error = u8;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(Self::Disabled),
            0x01 => Ok(Self::Enabled),
            other => Err(other),
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OscpHeader {
    header_byte: u8,
}

impl OscpHeader {
    pub fn from_byte(header_byte: u8) -> Self {
        Self { header_byte }
    }

    pub fn to_byte(&self) -> u8 {
        self.header_byte
    }

    pub fn frame_type(&self) -> Result<OscpFrameType, u8> {
        (self.header_byte & 0x07).try_into()
    }

    pub fn operating_mode(&self) -> Result<OscpOperatingMode, u8> {
        ((self.header_byte >> 3) & 0x07).try_into()
    }

    pub fn misalignment_corr(&self) -> Result<OscpMisalignmentCorr, u8> {
        ((self.header_byte >> 6) & 0x03).try_into()
    }
}

struct ByteReader<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> ByteReader<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }

    fn u8(&mut self) -> u8 {
        let v = self.data[self.pos];
        self.pos += 1;
        v
    }

    fn u16(&mut self) -> u16 {
        let v = u16::from_le_bytes(self.data[self.pos..self.pos + 2].try_into().unwrap());
        self.pos += 2;
        v
    }

    fn u64(&mut self) -> u64 {
        let v = u64::from_le_bytes(self.data[self.pos..self.pos + 8].try_into().unwrap());
        self.pos += 8;
        v
    }

    fn f32(&mut self) -> f32 {
        let v = f32::from_le_bytes(self.data[self.pos..self.pos + 4].try_into().unwrap());
        self.pos += 4;
        v
    }

    fn i32(&mut self) -> i32 {
        let v = i32::from_le_bytes(self.data[self.pos..self.pos + 4].try_into().unwrap());
        self.pos += 4;
        v
    }

    fn u32(&mut self) -> u32 {
        let v = u32::from_le_bytes(self.data[self.pos..self.pos + 4].try_into().unwrap());
        self.pos += 4;
        v
    }

    fn bytes<const N: usize>(&mut self) -> [u8; N] {
        let v: [u8; N] = self.data[self.pos..self.pos + N].try_into().unwrap();
        self.pos += N;
        v
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OscpParseError {
    WrongLength { expected: usize, actual: usize },
    UnknownFrameType(u8),
    CrcMismatch,
}

pub const OSCP_RAW_LEN: usize = 61;

#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OscpRaw {
    pub header_byte: OscpHeader,  /* [7:6] Misalignment Correction (2b) | [5:3] Operating Mode (3b) | [2:0] Frame Type (3b) */
    pub counter: u8,              /* Wrapping frame counter [0,255] */
    pub timestamp_ms: u64,        /* Timestamp in ms since last power up or reset */
    pub gyro_x: f32,              /* dps */
    pub gyro_y: f32,              /* dps */
    pub gyro_z: f32,              /* dps */
    pub accel_x: f32,             /* g */
    pub accel_y: f32,             /* g */
    pub accel_z: f32,             /* g */
    pub incl_x: f32,              /* mg */
    pub incl_y: f32,              /* mg */
    pub mag_x: f32,               /* uT */
    pub mag_y: f32,               /* uT */
    pub mag_z: f32,               /* uT */
    pub temp: f32,                /* degC */
    pub status: u8,               /* Status byte - use OSCP_STATUS_* masks */
    pub crc: u16,                 /* Checksum */
}

impl OscpRaw {
    pub fn from_bytes(data: &[u8]) -> Result<Self, OscpParseError> {
        if data.len() != OSCP_RAW_LEN {
            return Err(OscpParseError::WrongLength { expected: OSCP_RAW_LEN, actual: data.len() });
        }
        let mut r = ByteReader::new(data);
        Ok(Self {
            header_byte: OscpHeader::from_byte(r.u8()),
            counter: r.u8(),
            timestamp_ms: r.u64(),
            gyro_x: r.f32(),
            gyro_y: r.f32(),
            gyro_z: r.f32(),
            accel_x: r.f32(),
            accel_y: r.f32(),
            accel_z: r.f32(),
            incl_x: r.f32(),
            incl_y: r.f32(),
            mag_x: r.f32(),
            mag_y: r.f32(),
            mag_z: r.f32(),
            temp: r.f32(),
            status: r.u8(),
            crc: r.u16(),
        })
    }
}

pub const OSCP_EULER_LEN: usize = 25;

#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OscpEuler {
    pub header_byte: OscpHeader,  /* [7:6] Misalignment Correction (2b) | [5:3] Operating Mode (3b) | [2:0] Frame Type (3b) */
    pub counter: u8,              /* Wrapping frame counter [0,255] */
    pub timestamp_ms: u64,        /* Timestamp in ms since last power up or reset */
    pub roll: f32,                /* deg */
    pub pitch: f32,               /* deg */
    pub yaw: f32,                 /* deg */
    pub status: u8,               /* Status byte - use OSCP_STATUS_* masks */
    pub crc: u16,                 /* Checksum */
}

impl OscpEuler {
    pub fn from_bytes(data: &[u8]) -> Result<Self, OscpParseError> {
        if data.len() != OSCP_EULER_LEN {
            return Err(OscpParseError::WrongLength { expected: OSCP_EULER_LEN, actual: data.len() });
        }
        let mut r = ByteReader::new(data);
        Ok(Self {
            header_byte: OscpHeader::from_byte(r.u8()),
            counter: r.u8(),
            timestamp_ms: r.u64(),
            roll: r.f32(),
            pitch: r.f32(),
            yaw: r.f32(),
            status: r.u8(),
            crc: r.u16(),
        })
    }
}

pub const OSCP_QUAT_LEN: usize = 29;

#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OscpQuat {
    pub header_byte: OscpHeader,  /* [7:6] Misalignment Correction (2b) | [5:3] Operating Mode (3b) | [2:0] Frame Type (3b) */
    pub counter: u8,              /* Wrapping frame counter [0,255] */
    pub timestamp_ms: u64,        /* Timestamp in ms since last power up or reset */
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub status: u8,               /* Status byte - use OSCP_STATUS_* masks */
    pub crc: u16,                 /* Checksum */
}

impl OscpQuat {
    pub fn from_bytes(data: &[u8]) -> Result<Self, OscpParseError> {
        if data.len() != OSCP_QUAT_LEN {
            return Err(OscpParseError::WrongLength { expected: OSCP_QUAT_LEN, actual: data.len() });
        }
        let mut r = ByteReader::new(data);
        Ok(Self {
            header_byte: OscpHeader::from_byte(r.u8()),
            counter: r.u8(),
            timestamp_ms: r.u64(),
            w: r.f32(),
            x: r.f32(),
            y: r.f32(),
            z: r.f32(),
            status: r.u8(),
            crc: r.u16(),
        })
    }
}

pub const OSCP_ROT_MAT_LEN: usize = 49;

#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OscpRotMat {
    pub header_byte: OscpHeader,  /* [7:6] Misalignment Correction (2b) | [5:3] Operating Mode (3b) | [2:0] Frame Type (3b) */
    pub counter: u8,              /* Wrapping frame counter [0,255] */
    pub timestamp_ms: u64,        /* Timestamp in ms since last power up or reset */
    pub rm: [[f32; 3]; 3],        /* Row-major 3x3 rotation matrix */
    pub status: u8,               /* Status byte - use OSCP_STATUS_* masks */
    pub crc: u16,                 /* Checksum */
}

impl OscpRotMat {
    pub fn from_bytes(data: &[u8]) -> Result<Self, OscpParseError> {
        if data.len() != OSCP_ROT_MAT_LEN {
            return Err(OscpParseError::WrongLength { expected: OSCP_ROT_MAT_LEN, actual: data.len() });
        }
        let mut r = ByteReader::new(data);
        Ok(Self {
            header_byte: OscpHeader::from_byte(r.u8()),
            counter: r.u8(),
            timestamp_ms: r.u64(),
            rm: [
                [r.f32(), r.f32(), r.f32()],
                [r.f32(), r.f32(), r.f32()],
                [r.f32(), r.f32(), r.f32()],
            ],
            status: r.u8(),
            crc: r.u16(),
        })
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GnssFlags {
    byte: u8,
}

impl GnssFlags {
    pub fn from_byte(byte: u8) -> Self {
        Self { byte }
    }

    pub fn to_byte(&self) -> u8 {
        self.byte
    }

    pub fn gnss_fix_ok(&self) -> bool {
        (self.byte & 0x01) != 0
    }

    pub fn invalid_llh(&self) -> bool {
        ((self.byte >> 1) & 0x01) != 0
    }

    pub fn last_correction_age(&self) -> u8 {
        (self.byte >> 4) & 0x0F
    }
}

pub const OSCP_GNSS_LEN: usize = 64;

#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OscpGnss {
    pub header_byte: OscpHeader,  /* [7:6] Misalignment Correction (2b) | [5:3] Operating Mode (3b) | [2:0] Frame Type (3b) */
    pub counter: u8,              /* Wrapping frame counter [0,255] */
    pub timestamp_ms: u64,        /* Timestamp in ms since last power up or reset */
    pub gnss_fix_type: u8,
    pub num_satellites: u8,
    pub longitude: f32,                /* deg */
    pub latitude: f32,                 /* deg */
    pub height: i32,                /* mm */
    pub horizontal_accuracy: u32,   /* mm */
    pub vertical_accuracy: u32,     /* mm */
    pub velocity_north: i32,         /* mm/s */
    pub velocity_east: i32,          /* mm/s */
    pub velocity_down: i32,         /* mm/s */
    pub speed_accuracy: u32,        /* mm/s */
    pub heading_of_motion: f32,       /* deg */
    pub heading_accuracy: f32,         /* deg */
    pub pdop: f32,
    pub gnss_flags: GnssFlags,    /* [7:4] Last Correction Age (4b) | [3:2] Reserved (2b) | [1] Invalid LLH (1b) | [0] GNSS Fix OK (1b) */
    pub status: u8,               /* Status byte - use OSCP_STATUS_* masks */
    pub crc: u16,                 /* Checksum */
}

impl OscpGnss {
    pub fn from_bytes(data: &[u8]) -> Result<Self, OscpParseError> {
        if data.len() != OSCP_GNSS_LEN {
            return Err(OscpParseError::WrongLength { expected: OSCP_GNSS_LEN, actual: data.len() });
        }
        let mut r = ByteReader::new(data);
        Ok(Self {
            header_byte: OscpHeader::from_byte(r.u8()),
            counter: r.u8(),
            timestamp_ms: r.u64(),
            gnss_fix_type: r.u8(),
            num_satellites: r.u8(),
            longitude: r.f32(),                /* deg */
            latitude: r.f32(),               /* deg */
            height: r.i32(),             /* mm */
            horizontal_accuracy: r.u32(),   /* mm */
            vertical_accuracy: r.u32(),     /* mm */
            velocity_north: r.i32(),         /* mm/s */
            velocity_east: r.i32(),          /* mm/s */
            velocity_down: r.i32(),         /* mm/s */
            speed_accuracy: r.u32(),        /* mm/s */
            heading_of_motion: r.f32(),       /* deg */
            heading_accuracy: r.f32(),         /* deg */
            pdop: r.f32(),
            gnss_flags: GnssFlags::from_byte(r.u8()),
            status: r.u8(),
            crc: r.u16(),
        })
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FilterCfg {
    byte: u8,
}

impl FilterCfg {
    pub fn from_byte(byte: u8) -> Self {
        Self { byte }
    }

    pub fn to_byte(&self) -> u8 {
        self.byte
    }

    pub fn filters(&self) -> u8 {
        self.byte & 0x03
    }

    pub fn lpf(&self) -> u8 {
        (self.byte >> 2) & 0x07
    }

    pub fn hpf(&self) -> u8 {
        (self.byte >> 5) & 0x07
    }
}

pub const OSCP_DEBUG_1_LEN: usize = 58;

#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OscpDebug1 {
    pub header_byte: OscpHeader,  /* [7:6] Misalignment Correction (2b) | [5:3] Operating Mode (3b) | [2:0] Frame Type (3b) */
    pub counter: u8,              /* Wrapping frame counter [0,255] */
    pub gxb: u32,                 /* Gyro X bias */
    pub gyb: u32,                 /* Gyro Y bias */
    pub gzb: u32,                 /* Gyro Z bias */
    pub gob: u32,                 /* Gyro output bias */
    pub axb: u32,                 /* Accel X bias */
    pub ayb: u32,                 /* Accel Y bias */
    pub azb: u32,                 /* Accel Z bias */
    pub ixb: u32,                 /* Inclinometer X bias */
    pub iyb: u32,                 /* Inclinometer Y bias */
    pub mxb: u32,                 /* Mag X bias */
    pub myb: u32,                 /* Mag Y bias */
    pub mzb: u32,                 /* Mag Z bias */
    pub gyro_filter_cfg: FilterCfg,   /* [7:5] Gyro HPF (3b) | [4:2] Gyro LPF (3b) | [1:0] Gyro Filters (2b) */
    pub accel_filter_cfg: FilterCfg,  /* [7:5] Accel HPF (3b) | [4:2] Accel LPF (3b) | [1:0] Accel Filters (2b) */
    pub reserved_0: u16,
    pub reserved_1: u8,
    pub status: u8,               /* Status byte - use OSCP_STATUS_* masks */
    pub crc: u16,                 /* Checksum */
}

impl OscpDebug1 {
    pub fn from_bytes(data: &[u8]) -> Result<Self, OscpParseError> {
        if data.len() != OSCP_DEBUG_1_LEN {
            return Err(OscpParseError::WrongLength { expected: OSCP_DEBUG_1_LEN, actual: data.len() });
        }
        let mut r = ByteReader::new(data);
        Ok(Self {
            header_byte: OscpHeader::from_byte(r.u8()),
            counter: r.u8(),
            gxb: r.u32(),
            gyb: r.u32(),
            gzb: r.u32(),
            gob: r.u32(),
            axb: r.u32(),
            ayb: r.u32(),
            azb: r.u32(),
            ixb: r.u32(),
            iyb: r.u32(),
            mxb: r.u32(),
            myb: r.u32(),
            mzb: r.u32(),
            gyro_filter_cfg: FilterCfg::from_byte(r.u8()),
            accel_filter_cfg: FilterCfg::from_byte(r.u8()),
            reserved_0: r.u16(),
            reserved_1: r.u8(),
            status: r.u8(),
            crc: r.u16(),
        })
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FusionCfg {
    byte: u8,
}

impl FusionCfg {
    pub fn from_byte(byte: u8) -> Self {
        Self { byte }
    }

    pub fn to_byte(&self) -> u8 {
        self.byte
    }

    pub fn fusion_convention(&self) -> u8 {
        self.byte & 0x0F
    }

    pub fn fusion_heading_source(&self) -> u8 {
        (self.byte >> 4) & 0x0F
    }
}

pub const OSCP_DEBUG_2_LEN: usize = 58;

#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OscpDebug2 {
    pub header_byte: OscpHeader,  /* [7:6] Misalignment Correction (2b) | [5:3] Operating Mode (3b) | [2:0] Frame Type (3b) */
    pub counter: u8,              /* Wrapping frame counter [0,255] */
    pub mxx: u32,                 /* Mag calibration matrix col X, row X */
    pub myx: u32,                 /* Mag calibration matrix col X, row Y */
    pub mzx: u32,                 /* Mag calibration matrix col X, row Z */
    pub mxy: u32,                 /* Mag calibration matrix col Y, row X */
    pub myy: u32,                 /* Mag calibration matrix col Y, row Y */
    pub mzy: u32,                 /* Mag calibration matrix col Y, row Z */
    pub mxz: u32,                 /* Mag calibration matrix col Z, row X */
    pub myz: u32,                 /* Mag calibration matrix col Z, row Y */
    pub mzz: u32,                 /* Mag calibration matrix col Z, row Z */
    pub fusion_gain: u32,                     /* AHRS fusion gain */
    pub fusion_accel_rejection: u32,          /* AHRS fusion accel rejection */
    pub fusion_mag_rejection: u32,            /* AHRS fusion mag rejection */
    pub fusion_recovery_trigger_period: u32,  /* AHRS fusion recovery trigger period */
    pub fusion_cfg: FusionCfg,    /* [7:4] Fusion Heading Source (4b) | [3:0] Fusion Convention (4b) */
    pub status: u8,               /* Status byte - use OSCP_STATUS_* masks */
    pub crc: u16,                 /* Checksum */
}

impl OscpDebug2 {
    pub fn from_bytes(data: &[u8]) -> Result<Self, OscpParseError> {
        if data.len() != OSCP_DEBUG_2_LEN {
            return Err(OscpParseError::WrongLength { expected: OSCP_DEBUG_2_LEN, actual: data.len() });
        }
        let mut r = ByteReader::new(data);
        Ok(Self {
            header_byte: OscpHeader::from_byte(r.u8()),
            counter: r.u8(),
            mxx: r.u32(),
            myx: r.u32(),
            mzx: r.u32(),
            mxy: r.u32(),
            myy: r.u32(),
            mzy: r.u32(),
            mxz: r.u32(),
            myz: r.u32(),
            mzz: r.u32(),
            fusion_gain: r.u32(),
            fusion_accel_rejection: r.u32(),
            fusion_mag_rejection: r.u32(),
            fusion_recovery_trigger_period: r.u32(),
            fusion_cfg: FusionCfg::from_byte(r.u8()),
            status: r.u8(),
            crc: r.u16(),
        })
    }
}

/* RENAMED from RateCfg (09-09-2026): CONFIRMED against the OSCP protocol
 * doc's Startup Frame byte table — this byte is "Accelerometer Dynamic
 * Range [7:4] | Gyroscope Dynamic Range [3:0]", not a data rate at all.
 * There is no separate per-sensor output-data-rate register anywhere in
 * the Startup frame; data rate is determined entirely by Operating Mode
 * (see the protocol doc's own Operating Mode table). The OLD name/doc
 * comment calling this "rate" was never verified against a real spec (see
 * git history / lib.rs's and eskf.rs's now-stale doc comments referencing
 * that uncertainty) — gyro_dr()/accel_dr() already used "dr" in their
 * names, which turned out to be the correct reading (dynamic Range) all
 * along. Decode the raw code further with OscpGyroDr::from_dyn_range_code /
 * OscpAccelDr::from_dyn_range_code — this struct only extracts the nibble,
 * it doesn't know the (separate, non-sequential) code->range mapping. */
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DynRangeCfg {
    byte: u8,
}

impl DynRangeCfg {
    pub fn from_byte(byte: u8) -> Self {
        Self { byte }
    }

    pub fn to_byte(&self) -> u8 {
        self.byte
    }

    pub fn gyro_dr(&self) -> u8 {
        self.byte & 0x0F
    }

    pub fn accel_dr(&self) -> u8 {
        (self.byte >> 4) & 0x0F
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InclAhrsCfg {
    byte: u8,
}

impl InclAhrsCfg {
    pub fn from_byte(byte: u8) -> Self {
        Self { byte }
    }

    pub fn to_byte(&self) -> u8 {
        self.byte
    }

    /* CONFIRMED against the OSCP protocol doc (09-09-2026): this nibble is
     * "Inclinometer Dynamic Range", not a data rate — same correction as
     * DynRangeCfg's rename above, same reason (no separate data-rate
     * register exists in the Startup frame at all). Decode further with
     * OscpInclDr::from_dyn_range_code. */
    pub fn incl_dr(&self) -> u8 {
        self.byte & 0x0F
    }

    pub fn ahrs_convention(&self) -> u8 {
        (self.byte >> 4) & 0x03
    }

    pub fn ahrs_heading_src(&self) -> u8 {
        (self.byte >> 6) & 0x03
    }
}

pub const OSCP_STARTUP_LEN: usize = 40;

#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OscpStartup {
    pub header_byte: OscpHeader,   /* [7:6] Misalignment Correction (2b) | [5:3] Operating Mode (3b) | [2:0] Frame Type (3b) */
    pub mark_number: [u8; 10],
    pub unit_number: u16,
    pub sw_major_ver: u8,
    pub sw_minor_ver: u8,
    pub sw_patch_ver: u8,
    pub enabled_frames: u8,        /* Use OSCP_ENABLE_* masks */
    pub dyn_range_cfg: DynRangeCfg,      /* [7:4] Accelerometer Dynamic Range (4b) | [3:0] Gyroscope Dynamic Range (4b) */
    pub gyro_filter_cfg: FilterCfg,      /* [7:5] Gyro HPF (3b) | [4:2] Gyro LPF (3b) | [1:0] Gyro Filters (2b) */
    pub accel_filter_cfg: FilterCfg,     /* [7:5] Accel HPF (3b) | [4:2] Accel LPF (3b) | [1:0] Accel Filters (2b) */
    pub incl_ahrs_cfg: InclAhrsCfg,      /* [7:6] AHRS Heading Source (2b) | [5:4] AHRS Convention (2b) | [3:0] Inclinometer Dynamic Range (4b) */
    pub ahrs_gain: f32,
    pub ahrs_accel_rej: f32,
    pub ahrs_mag_rej: f32,
    pub ahrs_rec_trig_per: u32,    /* Recovery Trigger Period in s */
    pub status: u8,                /* Status byte - use OSCP_STATUS_* masks */
    pub crc: u16,                  /* Checksum */
}

impl OscpStartup {
    pub fn from_bytes(data: &[u8]) -> Result<Self, OscpParseError> {
        if data.len() != OSCP_STARTUP_LEN {
            return Err(OscpParseError::WrongLength { expected: OSCP_STARTUP_LEN, actual: data.len() });
        }
        let mut r = ByteReader::new(data);
        Ok(Self {
            header_byte: OscpHeader::from_byte(r.u8()),
            mark_number: r.bytes(),
            unit_number: r.u16(),
            sw_major_ver: r.u8(),
            sw_minor_ver: r.u8(),
            sw_patch_ver: r.u8(),
            enabled_frames: r.u8(),
            dyn_range_cfg: DynRangeCfg::from_byte(r.u8()),
            gyro_filter_cfg: FilterCfg::from_byte(r.u8()),
            accel_filter_cfg: FilterCfg::from_byte(r.u8()),
            incl_ahrs_cfg: InclAhrsCfg::from_byte(r.u8()),
            ahrs_gain: r.f32(),
            ahrs_accel_rej: r.f32(),
            ahrs_mag_rej: r.f32(),
            ahrs_rec_trig_per: r.u32(),
            status: r.u8(),
            crc: r.u16(),
        })
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OscpFrame {
    Raw(OscpRaw),
    Euler(OscpEuler),
    Quat(OscpQuat),
    RotMat(OscpRotMat),
    Gnss(OscpGnss),
    Debug1(OscpDebug1),
    Debug2(OscpDebug2),
    Startup(OscpStartup),
    CmdResponse(OscpFrameCmd),
}

#[derive(Debug, Clone, Copy, Default)]
pub struct OscpStats {
    pub frames_ok: u32,         /* Number of frames successfully parsed */
    pub framing_errors: u32,    /* Number of frames dropped due to framing errors */
    pub crc_errors: u32,        /* Number of frames dropped due to CRC errors */
    pub cobs_errors: u32,       /* Number of frames dropped due to COBS errors  */
    pub overflows: u32,         /* Number of frames dropped due to buffer overflow */
}

pub struct OscpParser {
    pub buf: [u8; OSCP_FRAME_MAX_LEN],
    pub buf_len: usize,
    pub synced: bool,
    pub stats: OscpStats,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OscpErr {
    Ok         = 0,
    Err        = -1,
    ErrInvalid = -3,
    ErrNull    = -2,
}

/**
 * Transport selector for command encoding.
 * - Rs422: COBS-encoded and 0x00-delimited (RS422-based units).
 * - Canfd: Raw ASCII payload for CAN-FD; the driver frames the
 *          message, and the caller needs to sets the command identifier + DLC.
 */
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OscpTransport {
    Rs422 = 0,
    Canfd = 1,
}

/**
 * Frame type selector — used by oscp_cmd_of, oscp_cmd_enable_oft,
 * oscp_cmd_disable_oft. Debug is only valid for oscp_cmd_of; passing it to
 * enable/disable OFT returns OscpErr::ErrInvalid.
 */
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OscpFrameSel {
    Raw        = b'R' as isize,
    Euler      = b'E' as isize,
    Quaternion = b'Q' as isize,
    RotMatrix  = b'M' as isize,
    Gnss       = b'G' as isize,
    Debug      = b'D' as isize, /* oscp_cmd_of only */
}

/** Operating mode selector — used by oscp_cmd_om */
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OscpOmSel {
    Idle   = b'I' as isize,
    Low    = b'L' as isize,
    Medium = b'M' as isize,
}

/** Gyroscope dynamic range — used by oscp_cmd_drg */
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OscpGyroDr {
    Dps125  = 125,
    Dps250  = 250,
    Dps500  = 500,
    Dps1000 = 1000,
    Dps2000 = 2000,
    Dps4000 = 4000,
}

impl OscpGyroDr {
    /* CONFIRMED against the OSCP protocol doc's "Mapping of MEMS Gyroscope
     * Dynamic Ranges" table (09-09-2026) — the 4-bit code reported back in
     * a Startup frame's dynamic-range byte (see DynRangeCfg::gyro_dr below)
     * is NOT the same value this enum's own discriminant uses for the
     * DRG write command (that's the literal dps figure, e.g. 125 — see
     * oscp_cmd_drg). The readback code is a separate, non-sequential
     * mapping: 250->0b0000, 4000->0b0001, 125->0b0010, 500->0b0100,
     * 1000->0b1000, 2000->0b1100 — decoding it as if it were the same
     * numbering as the write side would silently report the wrong range. */
    pub fn from_dyn_range_code(code: u8) -> Result<Self, u8> {
        match code {
            0b0000 => Ok(Self::Dps250),
            0b0001 => Ok(Self::Dps4000),
            0b0010 => Ok(Self::Dps125),
            0b0100 => Ok(Self::Dps500),
            0b1000 => Ok(Self::Dps1000),
            0b1100 => Ok(Self::Dps2000),
            other => Err(other),
        }
    }
}

/** Accelerometer dynamic range — used by oscp_cmd_dra */
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OscpAccelDr {
    G2  = 2,
    G4  = 4,
    G8  = 8,
    G16 = 16,
}

impl OscpAccelDr {
    /* CONFIRMED against the OSCP protocol doc's "Mapping of Accelerometer
     * Dynamic Ranges" table (09-09-2026) — see OscpGyroDr::from_dyn_range_code's
     * doc comment for why this readback code is a separate mapping from the
     * enum's own DRA-write discriminant: 2->0b0000, 16->0b0001, 4->0b0010,
     * 8->0b0011. */
    pub fn from_dyn_range_code(code: u8) -> Result<Self, u8> {
        match code {
            0b00 => Ok(Self::G2),
            0b01 => Ok(Self::G16),
            0b10 => Ok(Self::G4),
            0b11 => Ok(Self::G8),
            other => Err(other),
        }
    }
}

/**
 * Inclinometer dynamic range — used by oscp_cmd_dri.
 * Values are in tenths of g (5 = 0.5 g, 10 = 1.0 g, 20 = 2.0 g, 30 = 3.0 g).
 */
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OscpInclDr {
    G0_5 = 5,
    G1_0 = 10,
    G2_0 = 20,
    G3_0 = 30,
}

impl OscpInclDr {
    /* CONFIRMED against the OSCP protocol doc's "Mapping of Inclinometer
     * Dynamic Ranges" table (09-09-2026) — see OscpGyroDr::from_dyn_range_code's
     * doc comment for why this readback code is a separate mapping from the
     * enum's own DRI-write discriminant: 0.5g->0b0000, 3.0g->0b0001,
     * 1.0g->0b0010, 2.0g->0b0011. */
    pub fn from_dyn_range_code(code: u8) -> Result<Self, u8> {
        match code {
            0b00 => Ok(Self::G0_5),
            0b01 => Ok(Self::G3_0),
            0b10 => Ok(Self::G1_0),
            0b11 => Ok(Self::G2_0),
            other => Err(other),
        }
    }
}

/**
 * User register mnemonics — used by oscp_cmd_wr.
 * Gyro bias (Gxb/Gyb/Gzb/Gob), accelerometer bias (Axb/Ayb/Azb),
 * inclinometer bias (Ixb/Iyb), magnetometer bias (Mxb/Myb/Mzb),
 * magnetometer calibration matrix (Mxx…Mzz), AHRS fusion parameters
 * (Fco/Fhs/Frt/Fga/Far/Fmr), sensor filter settings (Gfi/Glp/Ghp/Afi/Alp/Ahp).
 */
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OscpUsrReg {
    Gxb, /* Gyroscope X axis user bias */
    Gyb, /* Gyroscope Y axis user bias */
    Gzb, /* Gyroscope Z axis user bias */
    Gob, /* Optical gyroscope user bias (MK2E2 and MK2Z only) */
    Axb, /* Accelerometer X axis user bias */
    Ayb, /* Accelerometer Y axis user bias */
    Azb, /* Accelerometer Z axis user bias */
    Ixb, /* Inclinometer X axis user bias */
    Iyb, /* Inclinometer Y axis user bias */
    Mxb, /* Magnetometer X axis user bias */
    Myb, /* Magnetometer Y axis user bias */
    Mzb, /* Magnetometer Z axis user bias */
    Mxx, /* Magnetometer calibration: X-to-X */
    Myx, /* Magnetometer calibration: Y-to-X */
    Mzx, /* Magnetometer calibration: Z-to-X */
    Mxy, /* Magnetometer calibration: X-to-Y */
    Myy, /* Magnetometer calibration: Y-to-Y */
    Mzy, /* Magnetometer calibration: Z-to-Y */
    Mxz, /* Magnetometer calibration: X-to-Z */
    Myz, /* Magnetometer calibration: Y-to-Z */
    Mzz, /* Magnetometer calibration: Z-to-Z */
    Fco, /* AHRS fusion convention */
    Fhs, /* AHRS fusion heading source */
    Frt, /* AHRS fusion recovery trigger period */
    Fga, /* AHRS fusion gain */
    Far, /* AHRS fusion accelerometer rejection */
    Fmr, /* AHRS fusion magnetometer rejection */
    Gfi, /* Gyroscope filters enable bitmask */
    Glp, /* Gyroscope low-pass filter setting */
    Ghp, /* Gyroscope high-pass filter setting */
    Afi, /* Accelerometer filters enable bitmask */
    Alp, /* Accelerometer low-pass filter setting */
    Ahp, /* Accelerometer high-pass filter setting */
}




