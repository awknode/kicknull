//! v0.18 sidechain modes + KICK COPY tone shaping.
//!
//! MODES (what the kick does to the 808):
//!   PHASE    - phase-inverted copy of the 808 mixed in (PhaseChain).
//!   VOLUME   - plain volume ducking, depth in dB.
//!   COMPRESS - sidechain compressor keyed by the kick.
//!   RINGMOD  - during the hit the 808 is ring-modulated by the kick waveform.
//!   SHAPE    - Kickstart-style LFO shapes, fired by the kick or synced to the beat.
//!
//! KICK COPY tone chain (after the lookahead limiter):
//!   LENGTH (trim the tail) -> LOW shelf -> HIGH shelf -> DRIVE -> LEVEL.

use nice_plug::prelude::util;

pub const MODE_PHASE: i32 = 0;
pub const MODE_VOLUME: i32 = 1;
pub const MODE_COMPRESS: i32 = 2;
pub const MODE_RING: i32 = 3;
pub const MODE_SHAPE: i32 = 4;
pub const MODE_NAMES: [&str; 5] = ["PHASE", "VOLUME", "COMPRESS", "RINGMOD", "SHAPE"];

pub const SHAPE_NAMES: [&str; 8] = ["SMOOTH", "PUNCH", "LINEAR", "EXPO", "SINE", "PUMP", "GATE", "DOUBLE"];

/// How much to duck (1 = full, 0 = none) at position `x` (0..1) through a shape.
/// `bend` (0..1) reshapes each one; 0.5 is the "normal" version.
#[inline]
pub fn shape_at(kind: usize, x: f32, bend: f32) -> f32 {
    if !(0.0..1.0).contains(&x) {
        return 0.0;
    }
    let k = 2.0f32.powf(bend * 4.0 - 1.0); // 0.5 .. 8, 2 at bend 0.5
    match kind {
        // The original Kickstart-style curve.
        0 => (1.0 - x).powf(k),
        // Holds the full cut, then lets go.
        1 => {
            let h = 0.05 + 0.4 * (1.0 - bend);
            if x < h { 1.0 } else { (1.0 - (x - h) / (1.0 - h)).powf(2.0) }
        }
        2 => 1.0 - x,
        3 => {
            let a = 2.0 + 14.0 * bend;
            ((-a * x).exp() - (-a).exp()) / (1.0 - (-a).exp())
        }
        4 => 0.5 * (1.0 + (std::f32::consts::PI * x.powf(1.0 / k.max(0.5))).cos()),
        // Hangs near full cut, then rushes back at the end (classic pump).
        5 => 1.0 - x.powf(k),
        // Hard cut for part of the length, short fade out.
        6 => {
            let g = 0.15 + 0.7 * bend;
            if x < g { 1.0 } else if x < g + 0.03 { 1.0 - (x - g) / 0.03 } else { 0.0 }
        }
        // Two dips per length, the second one smaller.
        _ => {
            if x < 0.5 { (1.0 - 2.0 * x).powf(k) } else { 0.6 * (1.0 - (2.0 * x - 1.0)).powf(k) }
        }
    }
}

/// Soft-knee gain reduction in dB (positive) for a level above `thr`.
#[inline]
pub fn comp_gr_db(level_db: f32, thr_db: f32, ratio: f32, knee_db: f32) -> f32 {
    let slope = 1.0 - 1.0 / ratio.max(1.0);
    let over = level_db - thr_db;
    if knee_db > 0.0 && (2.0 * over).abs() <= knee_db {
        slope * (over + knee_db / 2.0).powi(2) / (2.0 * knee_db)
    } else if over > 0.0 {
        slope * over
    } else {
        0.0
    }
}

/// Sidechain compressor: GR smoothed in dB with attack/release coefficients.
#[derive(Clone, Copy, Default)]
pub struct Comp {
    pub gr: f32,
}

impl Comp {
    #[inline]
    pub fn tick(&mut self, key: f32, thr_db: f32, ratio: f32, knee_db: f32, att: f32, rel: f32) -> f32 {
        let target = comp_gr_db(util::gain_to_db(key.max(1e-6)), thr_db, ratio, knee_db);
        let c = if target > self.gr { att } else { rel };
        self.gr = target + (self.gr - target) * c;
        util::db_to_gain(-self.gr)
    }
}

/// RBJ biquad used for the KICK COPY shelves.
#[derive(Clone, Copy)]
pub struct Shelf {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
    key: (f32, f32),
}

impl Default for Shelf {
    fn default() -> Self {
        Self { b0: 1.0, b1: 0.0, b2: 0.0, a1: 0.0, a2: 0.0, z1: 0.0, z2: 0.0, key: (f32::NAN, f32::NAN) }
    }
}

impl Shelf {
    /// `high`: high shelf, otherwise low shelf. Recomputes only when gain/sr change.
    pub fn set(&mut self, high: bool, freq: f32, gain_db: f32, sr: f32) {
        if self.key == (gain_db, sr) {
            return;
        }
        self.key = (gain_db, sr);
        if gain_db.abs() < 0.01 {
            (self.b0, self.b1, self.b2, self.a1, self.a2) = (1.0, 0.0, 0.0, 0.0, 0.0);
            return;
        }
        let a = 10.0f32.powf(gain_db / 40.0);
        let w0 = 2.0 * std::f32::consts::PI * (freq / sr).min(0.49);
        let (sin, cos) = w0.sin_cos();
        let alpha = sin / 2.0 * std::f32::consts::SQRT_2; // shelf slope S = 1
        let sa = 2.0 * a.sqrt() * alpha;
        let (b0, b1, b2, a0, a1, a2) = if high {
            (
                a * ((a + 1.0) + (a - 1.0) * cos + sa),
                -2.0 * a * ((a - 1.0) + (a + 1.0) * cos),
                a * ((a + 1.0) + (a - 1.0) * cos - sa),
                (a + 1.0) - (a - 1.0) * cos + sa,
                2.0 * ((a - 1.0) - (a + 1.0) * cos),
                (a + 1.0) - (a - 1.0) * cos - sa,
            )
        } else {
            (
                a * ((a + 1.0) - (a - 1.0) * cos + sa),
                2.0 * a * ((a - 1.0) - (a + 1.0) * cos),
                a * ((a + 1.0) - (a - 1.0) * cos - sa),
                (a + 1.0) + (a - 1.0) * cos + sa,
                -2.0 * ((a - 1.0) + (a + 1.0) * cos),
                (a + 1.0) + (a - 1.0) * cos - sa,
            )
        };
        (self.b0, self.b1, self.b2, self.a1, self.a2) = (b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0);
    }

    pub fn clear(&mut self) {
        self.z1 = 0.0;
        self.z2 = 0.0;
    }

    #[inline]
    pub fn tick(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        y
    }
}

/// DRIVE: tanh saturation normalised so a full-scale peak stays full-scale.
/// 0 dB = bypass (bit exact).
#[inline]
pub fn drive(x: f32, drive_db: f32) -> f32 {
    if drive_db < 0.05 {
        return x;
    }
    let d = util::db_to_gain(drive_db);
    (d * x).tanh() / d.tanh()
}

/// LENGTH ≥ this = the whole kick copy (no trimming).
pub const KICK_LEN_FULL_MS: f32 = 999.0;
pub const KICK_LEN_FADE_MS: f32 = 8.0;

/// Finds each kick hit (held level crossing a fixed -26 dB) and counts samples
/// since it, so LENGTH can trim the copy's tail per hit.
#[derive(Clone, Copy)]
pub struct Onset {
    level: f32,
    hold: u32,
    armed: bool,
    pub since: u32,
}

impl Default for Onset {
    fn default() -> Self {
        Self { level: 0.0, hold: 0, armed: true, since: u32::MAX / 2 }
    }
}

impl Onset {
    pub const THRESH: f32 = 0.05;
    #[inline]
    pub fn tick(&mut self, level: f32, sr: f32) -> u32 {
        let hold = (0.010 * sr) as u32;
        self.level = crate::follow(self.level, &mut self.hold, level, 0.0, crate::coef(30.0, sr), hold);
        if self.armed && self.level >= Self::THRESH {
            self.armed = false;
            self.since = 0;
        } else {
            if !self.armed && self.level < Self::THRESH * 0.5 {
                self.armed = true;
            }
            self.since = self.since.saturating_add(1).min(u32::MAX / 2);
        }
        self.since
    }
}

/// Gain for the kick copy `t_ms` after its hit: full until LENGTH, then a short fade.
#[inline]
pub fn length_gain(t_ms: f32, len_ms: f32) -> f32 {
    if len_ms >= KICK_LEN_FULL_MS || t_ms <= len_ms {
        1.0
    } else {
        (-(t_ms - len_ms) / KICK_LEN_FADE_MS).exp()
    }
}

/// TRANSIENT SHAPER (Neutron-style, 3 bands). Each band: ATTACK lifts/cuts the
/// hit, SUSTAIN lifts/cuts the tail. Knobs -10..+10; 10 = up to 15 dB.
pub const TS_DB_PER_UNIT: f32 = 1.5;
pub const TS_MAX_DB: f32 = 18.0;

#[derive(Clone, Copy, Default)]
pub struct BandShaper {
    fast: f32,
    slow: f32,
    hold: f32,
    gain_db: f32,
}

#[derive(Clone, Copy)]
pub struct ShaperCoefs {
    fast_att: f32,
    fast_rel: f32,
    slow_att: f32,
    hold_rel: f32,
    smooth: f32,
}

impl ShaperCoefs {
    pub fn new(sr: f32) -> Self {
        Self {
            fast_att: crate::coef(0.3, sr),
            fast_rel: crate::coef(15.0, sr),
            slow_att: crate::coef(15.0, sr),
            hold_rel: crate::coef(250.0, sr),
            smooth: crate::coef(1.0, sr),
        }
    }
}

impl BandShaper {
    /// `level`: this band's peak across channels. Returns the band gain.
    #[inline]
    pub fn tick(&mut self, level: f32, atk: f32, sus: f32, c: &ShaperCoefs) -> f32 {
        let one = |env: f32, att: f32, rel: f32| -> f32 {
            let k = if level > env { att } else { rel };
            level + (env - level) * k
        };
        self.fast = one(self.fast, c.fast_att, c.fast_rel);
        self.slow = one(self.slow, c.slow_att, c.fast_rel);
        self.hold = one(self.hold, c.fast_att, c.hold_rel);
        let (a, s) = if self.fast > 1e-5 && self.hold > 1e-5 {
            (
                ((self.fast - self.slow) / self.fast).clamp(0.0, 1.0),
                ((self.hold - self.fast) / self.hold).clamp(0.0, 1.0),
            )
        } else {
            (0.0, 0.0)
        };
        let target = (TS_DB_PER_UNIT * (atk * a + sus * s)).clamp(-TS_MAX_DB, TS_MAX_DB);
        self.gain_db = target + (self.gain_db - target) * c.smooth;
        util::db_to_gain(self.gain_db)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shapes_start_full_and_end_open() {
        for kind in 0..SHAPE_NAMES.len() {
            for bend in [0.0, 0.5, 1.0] {
                assert!(shape_at(kind, 0.0, bend) > 0.99, "{} starts at {}", SHAPE_NAMES[kind], shape_at(kind, 0.0, bend));
                assert!(shape_at(kind, 0.999, bend) < 0.06, "{} ends at {}", SHAPE_NAMES[kind], shape_at(kind, 0.999, bend));
                for i in 0..1000 {
                    let v = shape_at(kind, i as f32 / 1000.0, bend);
                    assert!((0.0..=1.0001).contains(&v) && v.is_finite());
                }
            }
        }
    }

    #[test]
    fn compressor_ratio_and_knee() {
        // 12 dB over at 4:1, hard knee -> 9 dB GR
        assert!((comp_gr_db(-12.0, -24.0, 4.0, 0.0) - 9.0).abs() < 1e-4);
        assert_eq!(comp_gr_db(-30.0, -24.0, 4.0, 6.0), 0.0);
        // soft knee is continuous at its edges
        let a = comp_gr_db(-21.0 + 1e-3, -24.0, 4.0, 6.0);
        let b = comp_gr_db(-21.0 - 1e-3, -24.0, 4.0, 6.0);
        assert!((a - b).abs() < 1e-2);
    }

    #[test]
    fn shelves_flat_at_zero_and_boost_where_asked() {
        let sr = 48000.0;
        let resp = |high: bool, gain: f32, f: f32| {
            let mut s = Shelf::default();
            s.set(high, if high { 3000.0 } else { 100.0 }, gain, sr);
            let mut pk = 0.0f32;
            for n in 0..(sr as usize / 2) {
                let y = s.tick((2.0 * std::f32::consts::PI * f * n as f32 / sr).sin());
                if n > sr as usize / 4 { pk = pk.max(y.abs()); }
            }
            util::gain_to_db(pk)
        };
        assert!(resp(false, 0.0, 50.0).abs() < 0.01);
        assert!((resp(false, 6.0, 30.0) - 6.0).abs() < 0.6, "low shelf {}", resp(false, 6.0, 30.0));
        assert!(resp(false, 6.0, 5000.0).abs() < 0.3);
        assert!((resp(true, -6.0, 12000.0) + 6.0).abs() < 0.6);
        assert!(resp(true, -6.0, 60.0).abs() < 0.3);
    }

    #[test]
    fn drive_is_bypass_at_zero_and_keeps_full_scale() {
        assert_eq!(drive(0.3721, 0.0), 0.3721);
        assert!((drive(1.0, 12.0) - 1.0).abs() < 1e-5);
        assert!(drive(0.25, 12.0) > 0.25); // saturation lifts the body
    }

    /// A decaying 50 Hz kick: +SUSTAIN must make the tail louder (longer low end),
    /// +ATTACK must make the first few ms louder, and 0/0 changes nothing.
    #[test]
    fn transient_shaper_attack_and_sustain() {
        let sr = 48000.0;
        let c = ShaperCoefs::new(sr);
        let run = |atk: f32, sus: f32| -> (f32, f32) {
            let mut b = BandShaper::default();
            let (mut early, mut late) = (0.0f32, 0.0f32);
            for n in 0..(sr as usize / 2) {
                let t = n as f32 / sr;
                let x = (2.0 * std::f32::consts::PI * 50.0 * t).sin() * (-t / 0.08).exp();
                let y = x * b.tick(x.abs(), atk, sus, &c);
                if t < 0.006 { early = early.max(y.abs()); }
                if (0.15..0.2).contains(&t) { late = late.max(y.abs()); }
            }
            (early, late)
        };
        let (e0, l0) = run(0.0, 0.0);
        let (e_a, _) = run(5.2, 0.0);
        let (_, l_s) = run(0.0, 5.9);
        assert!(e_a > e0 * 1.2, "attack {e_a} vs {e0}");
        assert!(l_s > l0 * 1.5, "sustain {l_s} vs {l0}");
        let mut b = BandShaper::default();
        assert_eq!(b.tick(0.5, 0.0, 0.0, &c), 1.0);
    }

    #[test]
    fn length_trims_the_tail() {
        assert_eq!(length_gain(50.0, KICK_LEN_FULL_MS + 1.0), 1.0);
        assert_eq!(length_gain(40.0, 60.0), 1.0);
        assert!(length_gain(60.0 + 5.0 * KICK_LEN_FADE_MS, 60.0) < 0.01);
    }
}
