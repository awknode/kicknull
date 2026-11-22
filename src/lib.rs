//! KickNull — phase-inverted sidechain, rebuilt from the "tSG PhaseChain" FL Patcher patch.
//!
//! PhaseChain's signal flow (decoded from PhaseChain.fst):
//!   808 ──────────────────────────────────────────────┐
//!   808 → Stereo Shaper (L/R at -100% = polarity flip) → Balance ──(+)→ out
//!   kick → Peak Controller (TENSION, DECAY, VOL) → Formula → Balance volume
//! So the inverted 808 is mixed back in at a level that follows the kick's
//! envelope: the louder the kick, the more of the 808 cancels; as the kick dies
//! away, the 808 swells back in.
//!
//! KickNull does the same thing sample-accurately, with zero latency:
//!   out = 808 - cancel(t) * band(808)
//!   cancel(t) = amount * curve(envelope(kick * sensitivity), tension)
//! where band() is the whole 808 ("Full") or just the part below Freq.

use crossbeam::queue::ArrayQueue;
use nih_plug::prelude::*;
use nih_plug_egui::EguiState;
use std::sync::atomic::{AtomicU32, Ordering};
use std::collections::VecDeque;
use std::sync::Arc;

mod awk_ui;
mod editor;
#[cfg(test)]
mod shot;
mod oversample;
mod presets;
use awk_ui::{BypassRamp, DryDelay, Meters, PeakFollower};
use oversample::{Oversampler, MAX_FACTOR};
pub mod modes;
pub use modes::*;

/// One slice of history for the editor's scope (~4 ms of audio).
#[derive(Clone, Copy, Default)]
pub struct ScopeFrame {
    /// Peak of the incoming 808.
    pub dry: f32,
    /// Peak of what's left of the 808 after cancelling.
    pub wet: f32,
    /// Peak of the kick (sidechain), after sensitivity.
    pub key: f32,
    /// Highest cancel amount in this slice (0..1).
    pub cancel: f32,
}

pub const SCOPE_QUEUE: usize = 4096;
const SCOPE_SLICE_SECONDS: f32 = 0.004;

struct KickNull {
    params: Arc<KickNullParams>,
    sample_rate: f32,
    /// Envelope follower state (the "Peak Controller").
    env: f32,
    /// Samples left in the peak hold.
    hold_left: u32,
    wf: WindowFollower,
    cancel_s: f32,
    /// Low-band splitter per channel (only used when Freq isn't "Full").
    split: [Crossover; 2],
    split_freq: f32,
    scope: Arc<ArrayQueue<ScopeFrame>>,
    slice: ScopeFrame,
    slice_pos: usize,
    slice_len: usize,
    /// Lookahead delay line for the 808 (per channel).
    delay: [Vec<f32>; 2],
    kdelay: [Vec<f32>; 2],
    klim: KickLimiter,
    comp: Comp,
    kshelf: [[Shelf; 2]; 2],
    konset: Onset,
    ts_split: [[Crossover; 2]; 2],
    ts_freqs: (f32, f32),
    ts_band: [BandShaper; 3],
    delay_pos: usize,
    latency: u32,
    /// Sidechain "click" filter (300 Hz - 3 kHz band, like PhaseChain's Pro-MB key filter).
    key_hp: [Biquad; 2],
    key_lp: [Biquad; 2],
    /// GATE stage (Pro-MB style): detector, hold, gain reduction in dB.
    gate_key: f32,
    gate_hold: u32,
    gate_gr: f32,
    /// CURVE mode trigger + curve player.
    trig: CurveTrigger,
    /// Host tempo, shared with the editor for THE DIP preview.
    bpm: Arc<AtomicU32>,
    /// IN / OUT peak meters for the editor.
    meters: Arc<Meters>,
    peak: PeakFollower,
    ceil: awk_ui::Ceiling,
    byp: BypassRamp,
    /// Oversampling: factor in use, and its delay in base-rate samples.
    osf: usize,
    os_lat: usize,
    /// RINGMOD: the 808 band and the kick carrier go up, multiply, come down.
    os_ring: [Oversampler; 2],
    os_car: Oversampler,
    /// KICK COPY drive stage.
    os_drv: [Oversampler; 2],
    /// Everything that isn't oversampled is delayed by `os_lat` to stay lined up.
    d_lin: [DryDelay; 2],
    d_cancel: DryDelay,
    d_gate: DryDelay,
    d_thru: [DryDelay; 2],
    d_dry: [DryDelay; 2],
}

#[derive(Params)]
pub struct KickNullParams {
    #[persist = "editor-state-v2"]
    pub editor_state: Arc<EguiState>,

    /// Gain applied to the kick before it drives the cancellation.
    #[id = "sens"]
    pub sensitivity: FloatParam,
    /// How fast the cancellation rises when the kick hits (0 = instant).
    #[id = "attack"]
    pub attack: FloatParam,
    /// How fast the cancellation falls away, letting the 808 back in.
    #[id = "decay"]
    pub decay: FloatParam,
    /// Shape of the kick-level → cancellation curve. 50% = linear,
    /// higher = cancels harder at lower kick levels (like PhaseChain's TENSION).
    #[id = "tension"]
    pub tension: FloatParam,
    /// Maximum cancellation (PhaseChain's VOL). 100% = full null.
    #[id = "amount"]
    pub amount: FloatParam,
    /// Only cancel the 808 below this frequency. Fully right = "Full" (whole 808, like PhaseChain).
    #[id = "freq"]
    pub freq: FloatParam,
    #[id = "output"]
    pub output: FloatParam,
    /// Delays the 808 so the cut can start just before the kick lands (FL compensates).
    #[id = "look"]
    pub lookahead: FloatParam,
    /// PHASE stage key: listen only to the kick's click (300 Hz - 3 kHz).
    #[id = "click"]
    pub click: BoolParam,

    /// CURVE (Kickstart-style shape fired by each kick) or FOLLOW (PhaseChain envelope).
    #[id = "mode"]
    pub curve_mode: BoolParam,
    /// CURVE mode: how long the dip lasts, as a note value at the host tempo.
    #[id = "len"]
    pub length: FloatParam,
    /// CURVE mode: how fast the 808 comes back. Higher = snaps back sooner.
    #[id = "shape"]
    pub shape: FloatParam,

    /// GATE stage (copies the Pro-MB on BASS OUT): kick click level that starts the duck.
    #[id = "g_thr"]
    pub gate_thresh: FloatParam,
    /// GATE stage: maximum dip in dB (Pro-MB Range). 0 dB = gate stage off.
    #[id = "g_range"]
    pub gate_range: FloatParam,
    /// GATE stage: how fast the 808 comes back after the click.
    #[id = "g_rel"]
    pub gate_release: FloatParam,

    /// KICK COPY (PhaseChain's inner Pro-MB): the sidechain kick is added to the
    /// output through a lookahead brickwall limiter (compress, 100:1, hard knee,
    /// 0% attack/release, lookahead = the Lookahead knob, output 0 dB).
    #[id = "k_on"]
    pub kick_on: BoolParam,
    #[id = "k_thr"]
    pub kick_thresh: FloatParam,
    #[id = "k_rng"]
    pub kick_range: FloatParam,
    #[id = "k_lvl"]
    pub kick_level: FloatParam,

    // ---- v0.18 ----
    /// PHASE / VOLUME / COMPRESS / RINGMOD / SHAPE.
    #[id = "dmode"]
    pub duck_mode: IntParam,
    /// VOLUME: how far the 808 is turned down at full duck.
    #[id = "vdepth"]
    pub vol_depth: FloatParam,
    #[id = "c_thr"]
    pub comp_thresh: FloatParam,
    #[id = "c_ratio"]
    pub comp_ratio: FloatParam,
    #[id = "c_att"]
    pub comp_attack: FloatParam,
    #[id = "c_rel"]
    pub comp_release: FloatParam,
    #[id = "c_knee"]
    pub comp_knee: FloatParam,
    /// SHAPE: which LFO shape.
    #[id = "skind"]
    pub shape_kind: FloatParam,
    /// SHAPE: fire on each kick (off) or loop on the host beat (on).
    #[id = "sync"]
    pub beat_sync: BoolParam,
    /// KICK COPY tone.
    #[id = "k_low"]
    pub kick_low: FloatParam,
    #[id = "k_high"]
    pub kick_high: FloatParam,
    #[id = "k_drv"]
    pub kick_drive: FloatParam,
    #[id = "k_len"]
    pub kick_length: FloatParam,
    /// Pass the dry kick (from the sidechain) through to the output too, so the
    /// shaper works on the whole kick. Unroute the kick's own track when on.
    #[id = "k_thru"]
    pub kick_thru: BoolParam,
    /// TRANSIENT SHAPER on the kick output (copy, or dry + copy with KICK THRU).
    #[id = "ts_on"]
    pub ts_on: BoolParam,
    #[id = "ts_lx"]
    pub ts_low_x: FloatParam,
    #[id = "ts_hx"]
    pub ts_high_x: FloatParam,
    #[id = "ts_la"]
    pub ts_low_atk: FloatParam,
    #[id = "ts_ls"]
    pub ts_low_sus: FloatParam,
    #[id = "ts_ma"]
    pub ts_mid_atk: FloatParam,
    #[id = "ts_ms"]
    pub ts_mid_sus: FloatParam,
    #[id = "ts_ha"]
    pub ts_high_atk: FloatParam,
    #[id = "ts_hs"]
    pub ts_high_sus: FloatParam,

    // ---- v0.22 (awk studios) ----
    /// Header MIX: dry 808 against the processed output.
    #[id = "gmix"]
    pub mix: FloatParam,
    /// Header power button (host bypass).
    #[id = "bypass"]
    pub bypass: BoolParam,
    /// SAFE: output limiter at -0.1 dBFS (footer switch, on by default).
    #[id = "safe"]
    pub safe: BoolParam,
    /// Oversampling for the nonlinear stages (RINGMOD product, KICK DRIVE).
    #[id = "os"]
    pub os: IntParam,
}

impl Default for KickNull {
    fn default() -> Self {
        Self {
            params: Arc::new(KickNullParams::default()),
            sample_rate: 44100.0,
            env: 0.0,
            hold_left: 0,
            wf: WindowFollower::default(),
            cancel_s: 0.0,
            split: [Crossover::default(); 2],
            split_freq: 0.0,
            scope: Arc::new(ArrayQueue::new(SCOPE_QUEUE)),
            slice: ScopeFrame::default(),
            slice_pos: 0,
            slice_len: 176,
            delay: [Vec::new(), Vec::new()],
            kdelay: [Vec::new(), Vec::new()],
            klim: KickLimiter::default(),
            comp: Comp::default(),
            kshelf: [[Shelf::default(); 2]; 2],
            konset: Onset::default(),
            ts_split: [[Crossover::default(); 2]; 2],
            ts_freqs: (0.0, 0.0),
            ts_band: [BandShaper::default(); 3],
            delay_pos: 0,
            latency: 0,
            key_hp: [Biquad::default(); 2],
            key_lp: [Biquad::default(); 2],
            gate_key: 0.0,
            gate_hold: 0,
            gate_gr: 0.0,
            trig: CurveTrigger::default(),
            bpm: Arc::new(AtomicU32::new(120.0f32.to_bits())),
            meters: Arc::new(Meters::default()),
            peak: PeakFollower::default(),
            ceil: awk_ui::Ceiling::default(),
            byp: BypassRamp::default(),
            osf: 1,
            os_lat: 0,
            os_ring: [Oversampler::new(1), Oversampler::new(1)],
            os_car: Oversampler::new(1),
            os_drv: [Oversampler::new(1), Oversampler::new(1)],
            d_lin: [DryDelay::new(OS_DELAY_MAX), DryDelay::new(OS_DELAY_MAX)],
            d_cancel: DryDelay::new(OS_DELAY_MAX),
            d_gate: DryDelay::new(OS_DELAY_MAX),
            d_thru: [DryDelay::new(OS_DELAY_MAX), DryDelay::new(OS_DELAY_MAX)],
            d_dry: [DryDelay::new(OS_DELAY_MAX), DryDelay::new(OS_DELAY_MAX)],
        }
    }
}

fn ms_param(name: &str, default: f32, min: f32, max: f32) -> FloatParam {
    FloatParam::new(name, default, FloatRange::Skewed { min, max, factor: FloatRange::skew_factor(-2.0) })
        .with_unit(" ms")
        .with_value_to_string(Arc::new(|v| if v < 10.0 { format!("{v:.2}") } else { format!("{v:.0}") }))
}

fn db_param(name: &str, default_db: f32, min_db: f32, max_db: f32) -> FloatParam {
    FloatParam::new(
        name,
        util::db_to_gain(default_db),
        FloatRange::Skewed {
            min: util::db_to_gain(min_db),
            max: util::db_to_gain(max_db),
            factor: FloatRange::gain_skew_factor(min_db, max_db),
        },
    )
    .with_unit(" dB")
    .with_value_to_string(formatters::v2s_f32_gain_to_db(1))
    .with_string_to_value(formatters::s2v_f32_gain_to_db())
}

/// A cut in dB shown as "-24.0 dB" (value is positive).
fn plain_db_cut(name: &str, default: f32, max: f32) -> FloatParam {
    FloatParam::new(name, default, FloatRange::Linear { min: 0.0, max })
        .with_step_size(0.1)
        .with_value_to_string(Arc::new(|v| format!("-{v:.1} dB")))
}

fn hz_param(name: &str, default: f32, min: f32, max: f32) -> FloatParam {
    FloatParam::new(name, default, FloatRange::Skewed { min, max, factor: FloatRange::skew_factor(-1.0) })
        .with_value_to_string(Arc::new(|v| if v >= 1000.0 { format!("{:.2}k", v / 1000.0) } else { format!("{v:.0} Hz") }))
}

/// Transient shaper amount, -10..+10 like Neutron.
fn ts_param(name: &str, default: f32) -> FloatParam {
    FloatParam::new(name, default, FloatRange::Linear { min: -10.0, max: 10.0 })
        .with_step_size(0.1)
        .with_value_to_string(Arc::new(|v| format!("{v:+.1}")))
}

/// ±12 dB shelf.
fn shelf_param(name: &str) -> FloatParam {
    FloatParam::new(name, 0.0, FloatRange::Linear { min: -12.0, max: 12.0 })
        .with_step_size(0.1)
        .with_value_to_string(Arc::new(|v| format!("{v:+.1} dB")))
}

fn pct_param(name: &str, default: f32) -> FloatParam {
    pct_param_range(name, default, 0.0)
}

fn pct_param_range(name: &str, default: f32, min: f32) -> FloatParam {
    FloatParam::new(name, default, FloatRange::Linear { min, max: 1.0 })
        .with_unit("%")
        .with_value_to_string(formatters::v2s_f32_percentage(0))
        .with_string_to_value(formatters::s2v_f32_percentage())
}

pub const MAX_LOOKAHEAD_MS: f32 = 10.0;
/// Longest oversampling delay (8x = 29 samples) with room to spare.
const OS_DELAY_MAX: usize = 64;

/// Peak hold for FOLLOW mode: 10 ms, but shortened with very short Decay
/// (5x Decay) so Decay below 2 ms actually makes the cut let go sooner.
#[inline]
pub fn follow_hold_ms(decay_ms: f32) -> f32 {
    PEAK_HOLD_MS.min(decay_ms * 5.0)
}

/// Freq at (or above) which the whole 808 is cancelled instead of a low band.
pub const FULL_BAND_HZ: f32 = 20000.0;

impl Default for KickNullParams {
    fn default() -> Self {
        Self {
            editor_state: EguiState::from_size(editor::WIDTH, editor::HEIGHT),
            // Defaults = the owner's closest-to-PhaseChain settings (2026-11-17).
            sensitivity: db_param("Sensitivity", 9.0, -12.0, 24.0),
            attack: ms_param("Attack", 0.0, 0.0, 50.0),
            decay: ms_param("Decay", 2.0, 2.0, 1000.0),
            // Below 50% the cut only bites at the loud front of the kick (PhaseChain on
            // BASS OUT runs its Peak tension at -51%).
            tension: pct_param("Tension", 0.0),
            amount: pct_param("Amount", 1.0),
            // Defaults measured from the owner's PhaseChain bounce (untitled.mp3, 2026-11-17):
            // only <100 Hz dips, ~13 dB, starting ~5 ms before the click, back in ~35 ms.
            freq: FloatParam::new(
                "Freq",
                FULL_BAND_HZ,
                FloatRange::Skewed { min: 20.0, max: FULL_BAND_HZ, factor: FloatRange::skew_factor(-2.2) },
            )
            .with_value_to_string(Arc::new(|v| {
                if v >= FULL_BAND_HZ - 1.0 {
                    "Full".to_string()
                } else if v >= 1000.0 {
                    format!("{:.2} kHz", v / 1000.0)
                } else {
                    format!("{v:.0} Hz")
                }
            }))
            .with_string_to_value(formatters::s2v_f32_hz_then_khz()),
            output: db_param("Output", 0.0, -24.0, 12.0).with_smoother(SmoothingStyle::Logarithmic(20.0)),
            lookahead: FloatParam::new("Lookahead", 10.0, FloatRange::Linear { min: 0.0, max: MAX_LOOKAHEAD_MS })
                .with_step_size(0.1)
                .with_unit(" ms")
                .with_value_to_string(formatters::v2s_f32_rounded(1)),
            click: BoolParam::new("Click", false)
                .with_value_to_string(Arc::new(|v| if v { "Click".into() } else { "Full".into() })),
            curve_mode: BoolParam::new("Mode", false)
                .with_value_to_string(Arc::new(|v| if v { "Curve".into() } else { "Follow".into() })),
            length: FloatParam::new("Length", 3.0, FloatRange::Linear { min: 0.0, max: 5.0 })
                .with_step_size(1.0)
                .with_value_to_string(Arc::new(|v| NOTE_NAMES[(v.round() as usize).min(5)].to_string()))
                .with_string_to_value(Arc::new(|s| {
                    NOTE_NAMES.iter().position(|n| *n == s.trim()).map(|i| i as f32)
                })),
            shape: pct_param("Shape", 0.5),
            gate_thresh: db_param("Gate Threshold", -18.0, -60.0, 0.0),
            gate_range: FloatParam::new("Gate Range", 0.0, FloatRange::Linear { min: 0.0, max: 48.0 })
                .with_step_size(0.1)
                .with_unit(" dB")
                .with_value_to_string(Arc::new(|v| format!("-{v:.1}"))),
            gate_release: ms_param("Gate Release", 1.0, 1.0, 500.0),

            kick_on: BoolParam::new("Kick Copy", true),
            kick_thresh: db_param("Kick Threshold", -12.0, -60.0, 0.0),
            kick_range: FloatParam::new("Kick Range", 60.0, FloatRange::Linear { min: 0.0, max: 60.0 })
                .with_step_size(0.1)
                .with_unit(" dB")
                .with_value_to_string(Arc::new(|v| format!("-{v:.1}"))),
            kick_level: db_param("Kick Level", 0.0, -60.0, 6.0),

            duck_mode: IntParam::new("Mode", MODE_PHASE, IntRange::Linear { min: 0, max: 4 })
                .with_value_to_string(Arc::new(|v| MODE_NAMES[(v.clamp(0, 4)) as usize].to_string()))
                .with_string_to_value(Arc::new(|s| {
                    MODE_NAMES.iter().position(|n| n.eq_ignore_ascii_case(s.trim())).map(|i| i as i32)
                })),
            // Full depth by default: the kick goes as deep as it can (2026-11-20).
            vol_depth: plain_db_cut("Duck Depth", 60.0, 60.0),
            comp_thresh: db_param("Comp Threshold", -24.0, -60.0, 0.0),
            comp_ratio: FloatParam::new(
                "Comp Ratio",
                8.0,
                FloatRange::Skewed { min: 1.0, max: 100.0, factor: FloatRange::skew_factor(-2.0) },
            )
            .with_value_to_string(Arc::new(|v| format!("{v:.1}:1"))),
            comp_attack: ms_param("Comp Attack", 1.0, 0.01, 100.0),
            comp_release: ms_param("Comp Release", 80.0, 1.0, 1000.0),
            comp_knee: FloatParam::new("Comp Knee", 6.0, FloatRange::Linear { min: 0.0, max: 24.0 })
                .with_step_size(0.1)
                .with_unit(" dB"),
            shape_kind: FloatParam::new("LFO Shape", 0.0, FloatRange::Linear { min: 0.0, max: 7.0 })
                .with_step_size(1.0)
                .with_value_to_string(Arc::new(|v| SHAPE_NAMES[(v.round() as usize).min(7)].to_string()))
                .with_string_to_value(Arc::new(|s| {
                    SHAPE_NAMES.iter().position(|n| n.eq_ignore_ascii_case(s.trim())).map(|i| i as f32)
                })),
            beat_sync: BoolParam::new("Beat Sync", false),
            kick_low: shelf_param("Kick Low"),
            kick_high: shelf_param("Kick High"),
            kick_drive: FloatParam::new("Kick Drive", 0.0, FloatRange::Linear { min: 0.0, max: 24.0 })
                .with_step_size(0.1)
                .with_unit(" dB"),
            kick_length: FloatParam::new(
                "Kick Length",
                1000.0,
                FloatRange::Skewed { min: 5.0, max: 1000.0, factor: FloatRange::skew_factor(-1.5) },
            )
            .with_value_to_string(Arc::new(|v| {
                if v >= KICK_LEN_FULL_MS { "FULL".to_string() } else { format!("{v:.0} ms") }
            })),
            kick_thru: BoolParam::new("Kick Thru", false),
            // Defaults = the owner's Neutron transient settings (2026-11-18).
            ts_on: BoolParam::new("Shaper", true),
            ts_low_x: hz_param("Shaper Low/Mid", 135.0, 40.0, 500.0),
            ts_high_x: hz_param("Shaper Mid/High", 2000.0, 800.0, 8000.0),
            ts_low_atk: ts_param("Low Attack", 5.2),
            ts_low_sus: ts_param("Low Sustain", 5.9),
            ts_mid_atk: ts_param("Mid Attack", 1.0),
            ts_mid_sus: ts_param("Mid Sustain", -3.9),
            ts_high_atk: ts_param("High Attack", 0.0),
            ts_high_sus: ts_param("High Sustain", -2.5),
            mix: awk_ui::mix_param(),
            bypass: awk_ui::bypass_param(),
            safe: awk_ui::safe_param(),
            os: awk_ui::os_param(),
        }
    }
}

/// One-pole coefficient for a time constant in ms (0 ms = instant).
#[inline]
pub fn coef(ms: f32, sr: f32) -> f32 {
    let samples = ms * 0.001 * sr;
    if samples < 1.0 {
        0.0
    } else {
        (-1.0 / samples).exp()
    }
}

/// How long a kick peak is held before the envelope may fall. Bridges the gaps
/// between the half-cycles of a kick's low end (50 Hz half-cycle = 10 ms) so the
/// cut doesn't ripple with the waveform, which is what made the 808 sound squished.
pub const PEAK_HOLD_MS: f32 = 10.0;
/// FOLLOW-mode window: longer than a half-cycle of a kick down to ~20 Hz, so the
/// cut never "lets go" between waveform peaks (that wobble was the squish).
pub const WINDOW_MS: f32 = 25.0;
/// Final de-zipper on the cut amount.
pub const CANCEL_SMOOTH_MS: f32 = 1.0;

/// Peak-controller style envelope follower with peak hold: rises with `attack`,
/// holds for `hold` samples after the last peak, then falls with `decay`.
#[inline]
pub fn follow(env: f32, hold_left: &mut u32, key: f32, att: f32, dec: f32, hold: u32) -> f32 {
    if key >= env {
        *hold_left = hold;
        key + (env - key) * att
    } else if *hold_left > 0 {
        *hold_left -= 1;
        env
    } else {
        key + (env - key) * dec
    }
}

/// FOLLOW-mode envelope: the kick's peak over a sliding window (two alternating
/// blocks of WINDOW_MS), then attack/decay smoothing. On a fading kick the window
/// peak only steps down, so the cut can't wobble back up between half-cycles.
#[allow(dead_code)]
#[derive(Clone, Copy, Default)]
pub struct WindowFollower {
    pub env: f32,
    cur: f32,
    prev: f32,
    n: u32,
}

impl WindowFollower {
    #[inline]
    pub fn tick(&mut self, key: f32, att: f32, dec: f32, sr: f32) -> f32 {
        let block = ((WINDOW_MS * 0.001 * sr) as u32).max(1);
        self.cur = self.cur.max(key);
        self.n += 1;
        if self.n >= block {
            self.prev = self.cur;
            self.cur = 0.0;
            self.n = 0;
        }
        let target = self.cur.max(self.prev);
        self.env = if target >= self.env {
            target + (self.env - target) * att
        } else {
            target + (self.env - target) * dec
        };
        self.env
    }
}

/// Pro-MB "release 0%": how fast the kick limiter lets go once the peak has passed.
pub const KICK_RELEASE_MS: f32 = 1.0;
pub const KICK_RATIO: f32 = 100.0;

/// Lookahead brickwall limiter for the kick copy. The detector sees the kick
/// `look` samples before the (delayed) kick that gets output, so gain reduction is
/// in place before each peak arrives: hard knee, 100:1, instant attack.
pub struct KickLimiter {
    win: VecDeque<(u64, f32)>,
    n: u64,
    pub gain: f32,
}

impl Default for KickLimiter {
    fn default() -> Self {
        Self { win: VecDeque::new(), n: 0, gain: 1.0 }
    }
}

impl KickLimiter {
    pub fn with_capacity(cap: usize) -> Self {
        Self { win: VecDeque::with_capacity(cap + 4), n: 0, gain: 1.0 }
    }

    pub fn clear(&mut self) {
        self.win.clear();
        self.n = 0;
        self.gain = 1.0;
    }

    /// `level`: this sample's undelayed kick peak (both channels). Returns the gain
    /// for the kick sample `look` samples ago.
    #[inline]
    pub fn tick(&mut self, level: f32, look: u64, thresh: f32, range_db: f32, rel: f32) -> f32 {
        while let Some(&(_, v)) = self.win.back() {
            if v <= level { self.win.pop_back(); } else { break; }
        }
        if self.win.len() < self.win.capacity() {
            self.win.push_back((self.n, level));
        }
        while let Some(&(idx, _)) = self.win.front() {
            if idx + look < self.n { self.win.pop_front(); } else { break; }
        }
        self.n += 1;
        let pk = self.win.front().map(|w| w.1).unwrap_or(0.0);
        let target = if pk > thresh && thresh > 0.0 {
            let over = util::gain_to_db(pk / thresh);
            util::db_to_gain(-(over * (1.0 - 1.0 / KICK_RATIO)).min(range_db))
        } else {
            1.0
        };
        self.gain = if target <= self.gain { target } else { target + (self.gain - target) * rel };
        self.gain
    }
}

pub const NOTE_NAMES: [&str; 6] = ["1/32", "1/16", "1/8", "1/4", "1/2", "1/1"];
/// Beats (quarter notes) per LENGTH step.
pub const NOTE_BEATS: [f32; 6] = [0.125, 0.25, 0.5, 1.0, 2.0, 4.0];

/// Length of the dip in seconds at a tempo.
pub fn curve_seconds(length_step: f32, bpm: f32) -> f32 {
    NOTE_BEATS[(length_step.round() as usize).min(5)] * 60.0 / bpm.max(20.0)
}

/// Kickstart-style dip: how much of the 808 to remove at position `x` (0..1)
/// through the curve. A 0.5 ms fade-in avoids a click, then the 808 comes back
/// along (1-x)^k: higher `shape` = larger k = the 808 returns sooner.
pub const CURVE_FADE_S: f32 = 0.002;
#[inline]
pub fn curve_core(t_s: f32, len_s: f32, shape: f32) -> f32 {
    if t_s < 0.0 || t_s >= len_s {
        return 0.0;
    }
    let x = t_s / len_s;
    let k = 2.0f32.powf(shape * 4.0 - 1.0); // 0.5 .. 8, default 2
    (1.0 - x).powf(k)
}
/// The curve from a standing start (used by the editor preview).
#[inline]
pub fn curve_duck(t_s: f32, len_s: f32, shape: f32) -> f32 {
    (t_s / CURVE_FADE_S).clamp(0.0, 1.0) * curve_core(t_s, len_s, shape)
}

/// Fires one Kickstart-style curve per kick.
///
/// Triggering listens to a *held* kick level (10 ms hold, 30 ms release), not the
/// raw waveform: a low kick crosses zero every few ms, and re-arming on those
/// crossings restarted the curve mid-kick, which crackled. A new trigger also
/// fades from wherever the dip currently is, so the level never jumps.
#[derive(Clone, Copy)]
pub struct CurveTrigger {
    /// LFO shape (0 = the original Kickstart-style curve).
    pub kind: usize,
    level: f32,
    hold: u32,
    armed: bool,
    pos: Option<u32>,
    start: f32,
    last: f32,
}

impl Default for CurveTrigger {
    fn default() -> Self {
        Self { kind: 0, level: 0.0, hold: 0, armed: true, pos: None, start: 0.0, last: 0.0 }
    }
}

impl CurveTrigger {
    /// Returns how much of the 808 to remove this sample (0..1, before Amount).
    #[inline]
    pub fn tick(&mut self, key: f32, thresh: f32, len_s: f32, shape: f32, sr: f32) -> f32 {
        let hold = (PEAK_HOLD_MS * 0.001 * sr) as u32;
        self.level = follow(self.level, &mut self.hold, key, 0.0, coef(30.0, sr), hold);
        if self.armed && self.level >= thresh {
            self.armed = false;
            self.pos = Some(0);
            self.start = self.last;
        } else if !self.armed && self.level < thresh * 0.5 {
            self.armed = true; // kick has dropped 6 dB below the threshold
        }
        let d = match self.pos {
            Some(n) => {
                let t = n as f32 / sr;
                if t >= len_s {
                    self.pos = None;
                    0.0
                } else {
                    self.pos = Some(n + 1);
                    let fade = (t / CURVE_FADE_S).min(1.0);
                    let core = if self.kind == 0 { curve_core(t, len_s, shape) } else { shape_at(self.kind, t / len_s, shape) };
                    self.start + (core - self.start) * fade
                }
            }
            None => 0.0,
        };
        self.last = d;
        d
    }
}

/// Pro-MB style gain reduction (dB, positive) for a key level, at 100:1.
pub const GATE_RATIO: f32 = 100.0;
#[inline]
pub fn gate_target_gr(level_db: f32, thresh_db: f32, range_db: f32) -> f32 {
    let over = level_db - thresh_db;
    if over <= 0.0 {
        0.0
    } else {
        (over * (1.0 - 1.0 / GATE_RATIO)).min(range_db)
    }
}

/// One step of the gate: detector (instant peak + 5 ms hold), then gain
/// reduction smoothed in dB (attack 0.5 ms, release as set). Returns the GR in dB.
pub const GATE_HOLD_MS: f32 = 5.0;
pub const GATE_ATTACK_MS: f32 = 0.5;
#[allow(clippy::too_many_arguments)]
#[inline]
pub fn gate_step(
    key: &mut f32,
    hold: &mut u32,
    gr: &mut f32,
    input: f32,
    thresh_db: f32,
    range_db: f32,
    rel_coef: f32,
    sr: f32,
) -> f32 {
    *key = follow(*key, hold, input, 0.0, coef(5.0, sr), (GATE_HOLD_MS * 0.001 * sr) as u32);
    let target = gate_target_gr(util::gain_to_db(key.max(1e-6)), thresh_db, range_db);
    let c = if target > *gr { coef(GATE_ATTACK_MS, sr) } else { rel_coef };
    *gr = target + (*gr - target) * c;
    *gr
}

/// Kick level (0..1+) → cancellation (0..1). Tension 0.5 = linear; higher values
/// bend the curve up so quieter parts of the kick already cancel a lot.
#[inline]
pub fn curve(level: f32, tension: f32) -> f32 {
    let exponent = (2.0f32).powf((0.5 - tension) * 4.0);
    level.clamp(0.0, 1.0).powf(exponent)
}

#[derive(Clone, Copy, Default)]
pub struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
}

impl Biquad {
    fn set_highpass(&mut self, freq: f32, sr: f32) {
        let w0 = 2.0 * std::f32::consts::PI * (freq / sr).min(0.49);
        let (sin, cos) = w0.sin_cos();
        let alpha = sin / (2.0 * std::f32::consts::FRAC_1_SQRT_2);
        let a0 = 1.0 + alpha;
        self.b0 = (1.0 + cos) / 2.0 / a0;
        self.b1 = -(1.0 + cos) / a0;
        self.b2 = self.b0;
        self.a1 = -2.0 * cos / a0;
        self.a2 = (1.0 - alpha) / a0;
    }
    fn set_lowpass(&mut self, freq: f32, sr: f32) {
        let w0 = 2.0 * std::f32::consts::PI * (freq / sr).min(0.49);
        let (sin, cos) = w0.sin_cos();
        let alpha = sin / (2.0 * std::f32::consts::FRAC_1_SQRT_2);
        let a0 = 1.0 + alpha;
        self.b0 = (1.0 - cos) / 2.0 / a0;
        self.b1 = (1.0 - cos) / a0;
        self.b2 = self.b0;
        self.a1 = -2.0 * cos / a0;
        self.a2 = (1.0 - alpha) / a0;
    }
    #[inline]
    fn tick(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        y
    }
}

/// Linkwitz-Riley 24 dB/oct crossover. Low + high sum to a flat (all-pass)
/// response, so turning only the low band down never smears the phase between
/// the bands — that smear is what made the 808 sound squished.
#[derive(Clone, Copy, Default)]
pub struct Crossover {
    lp: [Biquad; 2],
    hp: [Biquad; 2],
}

impl Crossover {
    pub fn set(&mut self, freq: f32, sr: f32) {
        for b in &mut self.lp {
            b.set_lowpass(freq, sr);
        }
        for b in &mut self.hp {
            b.set_highpass(freq, sr);
        }
    }
    fn clear(&mut self) {
        for b in self.lp.iter_mut().chain(self.hp.iter_mut()) {
            b.z1 = 0.0;
            b.z2 = 0.0;
        }
    }
    /// Returns (low, high).
    #[inline]
    pub fn tick(&mut self, x: f32) -> (f32, f32) {
        let l0 = self.lp[0].tick(x);
        let low = self.lp[1].tick(l0);
        let h0 = self.hp[0].tick(x);
        let high = self.hp[1].tick(h0);
        // LR4 bands come out in phase; sum = all-pass of x.
        (low, high)
    }
}

impl KickNull {
    /// (Re)builds the oversamplers for `factor` and clears the alignment delays.
    fn set_os(&mut self, factor: usize) {
        self.osf = factor;
        self.os_lat = oversample::latency(factor);
        self.os_ring = [Oversampler::new(factor), Oversampler::new(factor)];
        self.os_car = Oversampler::new(factor);
        self.os_drv = [Oversampler::new(factor), Oversampler::new(factor)];
        for d in self.d_lin.iter_mut().chain(self.d_thru.iter_mut()).chain(self.d_dry.iter_mut()) {
            d.clear();
        }
        self.d_cancel.clear();
        self.d_gate.clear();
    }

    /// Total delay reported to the host: lookahead + oversampling.
    fn total_latency(&self) -> u32 {
        self.lookahead_samples() + self.os_lat as u32
    }

    fn lookahead_samples(&self) -> u32 {
        let max = self.delay[0].len().saturating_sub(2) as u32;
        ((self.params.lookahead.value() * 0.001 * self.sample_rate).round() as u32).min(max)
    }
}

impl Plugin for KickNull {
    const NAME: &'static str = "KickNull";
    const VENDOR: &'static str = "awk studios";
    const URL: &'static str = "";
    const EMAIL: &'static str = "";
    const VERSION: &'static str = env!("CARGO_PKG_VERSION");

    const AUDIO_IO_LAYOUTS: &'static [AudioIOLayout] = &[
        AudioIOLayout {
            main_input_channels: NonZeroU32::new(2),
            main_output_channels: NonZeroU32::new(2),
            aux_input_ports: &[new_nonzero_u32(2)],
            names: PortNames {
                layout: Some("Stereo"),
                main_input: Some("808"),
                main_output: Some("Output"),
                aux_inputs: &["Kick (sidechain)"],
                aux_outputs: &[],
            },
            ..AudioIOLayout::const_default()
        },
        AudioIOLayout {
            main_input_channels: NonZeroU32::new(1),
            main_output_channels: NonZeroU32::new(1),
            aux_input_ports: &[new_nonzero_u32(1)],
            names: PortNames {
                layout: Some("Mono"),
                main_input: Some("808"),
                main_output: Some("Output"),
                aux_inputs: &["Kick (sidechain)"],
                aux_outputs: &[],
            },
            ..AudioIOLayout::const_default()
        },
    ];

    const SAMPLE_ACCURATE_AUTOMATION: bool = true;
    type SysExMessage = ();
    type BackgroundTask = ();

    fn params(&self) -> Arc<dyn Params> {
        self.params.clone()
    }

    fn editor(&mut self, _async_executor: AsyncExecutor<Self>) -> Option<Box<dyn Editor>> {
        editor::create(self.params.clone(), self.scope.clone(), self.bpm.clone(), self.meters.clone())
    }

    fn initialize(
        &mut self,
        _audio_io_layout: &AudioIOLayout,
        buffer_config: &BufferConfig,
        context: &mut impl InitContext<Self>,
    ) -> bool {
        self.sample_rate = buffer_config.sample_rate;
        self.slice_len = ((SCOPE_SLICE_SECONDS * self.sample_rate) as usize).max(1);
        let max = (MAX_LOOKAHEAD_MS * 0.001 * self.sample_rate) as usize + 2;
        self.delay = [vec![0.0; max], vec![0.0; max]];
        self.kdelay = [vec![0.0; max], vec![0.0; max]];
        self.klim = KickLimiter::with_capacity(max);
        self.delay_pos = 0;
        for k in 0..2 {
            self.key_hp[k].set_highpass(300.0, self.sample_rate);
            self.key_lp[k].set_lowpass(3000.0, self.sample_rate);
        }
        self.peak.set_sr(self.sample_rate);
        self.ceil.set_sr(self.sample_rate);
        self.byp.set_sr(self.sample_rate);
        self.set_os(oversample::factor_for(self.params.os.value()));
        self.latency = self.total_latency();
        context.set_latency_samples(self.latency);
        true
    }

    fn reset(&mut self) {
        let f = self.osf;
        self.set_os(f);
        self.peak = PeakFollower::default();
        self.peak.set_sr(self.sample_rate);
        self.klim.clear();
        self.comp = Comp::default();
        self.konset = Onset::default();
        self.ts_band = [BandShaper::default(); 3];
        for ch in &mut self.ts_split {
            for x in ch.iter_mut() {
                x.clear();
            }
        }
        for ch in &mut self.kshelf {
            for f in ch.iter_mut() {
                f.clear();
            }
        }
        for d in &mut self.kdelay {
            d.iter_mut().for_each(|s| *s = 0.0);
        }
        self.env = 0.0;
        self.hold_left = 0;
        self.wf = WindowFollower::default();
        self.cancel_s = 0.0;
        for f in &mut self.split {
            f.clear();
        }
        self.split_freq = 0.0;
        self.slice = ScopeFrame::default();
        self.slice_pos = 0;
        for d in &mut self.delay {
            d.iter_mut().for_each(|s| *s = 0.0);
        }
        self.trig = CurveTrigger::default();
        self.gate_key = 0.0;
        self.gate_hold = 0;
        self.gate_gr = 0.0;
        for f in self.key_hp.iter_mut().chain(self.key_lp.iter_mut()) {
            f.z1 = 0.0;
            f.z2 = 0.0;
        }
    }

    fn process(
        &mut self,
        buffer: &mut Buffer,
        aux: &mut AuxiliaryBuffers,
        context: &mut impl ProcessContext<Self>,
    ) -> ProcessStatus {
        let transport = context.transport();
        let bpm = transport.tempo.unwrap_or(120.0) as f32;
        let (playing, pos_beats) = (transport.playing, transport.pos_beats().unwrap_or(0.0));
        let before = self.latency;
        let sidechain: Option<&[&mut [f32]]> = aux.inputs.first().map(|b| b.as_slice_immutable());
        self.render(buffer, sidechain, bpm, playing, pos_beats);
        if self.latency != before {
            context.set_latency_samples(self.latency);
        }
        ProcessStatus::Normal
    }
}

impl KickNull {
    /// The whole audio path (`process` minus the host context, so tests can drive it).
    fn render(&mut self, buffer: &mut Buffer, sidechain: Option<&[&mut [f32]]>, bpm: f32, playing: bool, pos_beats: f64) {
        let sr = self.sample_rate;
        let look = self.lookahead_samples();
        let f = oversample::factor_for(self.params.os.value());
        if f != self.osf {
            self.set_os(f);
        }
        let osf = self.osf;
        let os_lat = self.os_lat;
        self.latency = look + os_lat as u32;
        let bypassed = self.params.bypass.value();
        let mut ob = [0.0f32; MAX_FACTOR];
        let mut oc = [0.0f32; MAX_FACTOR];
        let click = self.params.click.value();
        let curve_mode = self.params.curve_mode.value();
        self.bpm.store(bpm.to_bits(), Ordering::Relaxed);
        let dlen = self.delay[0].len().max(1);
        let hold = (PEAK_HOLD_MS * 0.001 * sr) as u32;

        let freq = self.params.freq.value();
        let full_band = freq >= FULL_BAND_HZ - 1.0;
        if !full_band && freq != self.split_freq {
            for f in &mut self.split {
                f.set(freq, sr);
            }
            self.split_freq = freq;
        }

        let c_smooth = coef(CANCEL_SMOOTH_MS, sr);
        let mode = self.params.duck_mode.value();
        let len_s = curve_seconds(self.params.length.value(), bpm);
        self.trig.kind = (self.params.shape_kind.value().round() as usize).min(7);
        let sync = self.params.beat_sync.value();
        let len_beats = NOTE_BEATS[(self.params.length.value().round() as usize).min(5)] as f64;
        // Beat position of the *output* sample (the 808 is delayed by the lookahead).
        let mut beat = pos_beats - look as f64 * bpm as f64 / 60.0 / sr as f64;
        let beat_step = bpm as f64 / 60.0 / sr as f64;
        let (c_thr, c_ratio, c_knee) = (
            util::gain_to_db(self.params.comp_thresh.value()),
            self.params.comp_ratio.value(),
            self.params.comp_knee.value(),
        );
        let (c_att, c_rel) = (coef(self.params.comp_attack.value(), sr), coef(self.params.comp_release.value(), sr));
        let vol_depth = self.params.vol_depth.value();
        for ch in 0..2 {
            self.kshelf[ch][0].set(false, 100.0, self.params.kick_low.value(), sr);
            self.kshelf[ch][1].set(true, 3000.0, self.params.kick_high.value(), sr);
        }
        let (k_drive, k_len) = (self.params.kick_drive.value(), self.params.kick_length.value());
        let kick_thru = self.params.kick_thru.value();
        let ts = [
            (self.params.ts_low_atk.value(), self.params.ts_low_sus.value()),
            (self.params.ts_mid_atk.value(), self.params.ts_mid_sus.value()),
            (self.params.ts_high_atk.value(), self.params.ts_high_sus.value()),
        ];
        let ts_active = self.params.ts_on.value() && ts.iter().any(|(a, s)| *a != 0.0 || *s != 0.0);
        let ts_f = (self.params.ts_low_x.value(), self.params.ts_high_x.value());
        if ts_f != self.ts_freqs {
            for ch in &mut self.ts_split {
                ch[0].set(ts_f.0, sr);
                ch[1].set(ts_f.1, sr);
            }
            self.ts_freqs = ts_f;
        }
        let ts_c = ShaperCoefs::new(sr);
        let _ = curve_mode;
        let k_rel = coef(KICK_RELEASE_MS, sr);
        let kick_on = self.params.kick_on.value();
        let kick_level = self.params.kick_level.value();
        for (i, mut channel_samples) in buffer.iter_samples().enumerate() {
            let sens = self.params.sensitivity.value();
            let att = coef(self.params.attack.value(), sr);
            let dec = coef(self.params.decay.value(), sr);
            let tension = self.params.tension.value();
            let amount = self.params.amount.value();
            let out_gain = self.params.output.smoothed.next();
            let mix = self.params.mix.smoothed.next();
            let on = self.byp.next(bypassed);

            let g_thr = util::gain_to_db(self.params.gate_thresh.value());
            let g_range = self.params.gate_range.value();
            let g_rel = coef(self.params.gate_release.value(), sr);

            let mut key = 0.0f32;
            let mut click_key = 0.0f32;
            let mut kraw = [0.0f32; 2];
            if let Some(sc) = sidechain {
                for (k, ch) in sc.iter().enumerate().take(2) {
                    let raw = ch.get(i).copied().unwrap_or(0.0);
                    kraw[k] = raw;
                    // Always run the click filter so it's settled when switched on.
                    let band = self.key_lp[k].tick(self.key_hp[k].tick(raw));
                    // The band is quieter than the full kick; +12 dB keeps the knobs in a similar range.
                    let v = if click { band * 4.0 } else { raw };
                    key = key.max(v.abs());
                    click_key = click_key.max(band.abs());
                }
            }
            let key = key * sens;

            // v0.14: back to the v0.12 peak-hold follower; v0.13's window follower made the release lurch.
            let f_hold = (follow_hold_ms(self.params.decay.value()) * 0.001 * sr) as u32;
            self.env = follow(self.env, &mut self.hold_left, key, att, dec, f_hold);
            // How much of the 808 to take away this sample (0..1), per mode.
            let comp_gain = if mode == MODE_COMPRESS {
                self.comp.tick(key, c_thr, c_ratio, c_knee, c_att, c_rel)
            } else {
                1.0
            };
            let cancel = match mode {
                MODE_SHAPE => {
                    if sync {
                        if playing && len_beats > 0.0 {
                            let x = (beat / len_beats).rem_euclid(1.0) as f32;
                            amount * shape_at(self.trig.kind, x, self.params.shape.value())
                        } else {
                            0.0
                        }
                    } else {
                        amount * self.trig.tick(key, self.params.gate_thresh.value(), len_s, self.params.shape.value(), sr)
                    }
                }
                MODE_VOLUME => curve(self.env, tension),
                MODE_COMPRESS => 1.0 - comp_gain,
                _ => amount * curve(self.env, tension), // PHASE, RINGMOD
            };
            beat += beat_step;
            let cancel = if mode == MODE_COMPRESS {
                self.cancel_s = cancel;
                cancel
            } else {
                self.cancel_s = cancel + (self.cancel_s - cancel) * c_smooth;
                if cancel == 0.0 && self.cancel_s < 1e-6 { 0.0 } else { self.cancel_s }
            };
            // Gain on the 808 (or its low band) for the gain-type modes.
            let duck_gain = match mode {
                MODE_VOLUME => util::db_to_gain(-cancel * vol_depth),
                _ => 1.0 - cancel,
            };
            // RINGMOD carrier: the kick waveform, normalised by its own envelope.
            let carrier = if mode == MODE_RING {
                let mono_sc = sidechain.map(|sc| sc.len() == 1).unwrap_or(false);
                let km = if mono_sc { kraw[0] } else { 0.5 * (kraw[0] + kraw[1]) };
                (km * sens / self.env.max(1e-4)).clamp(-1.0, 1.0)
            } else {
                0.0
            };

            // GATE stage, keyed on the kick's click like Pro-MB's 300 Hz - 3 kHz sidechain.
            let gr_db = if g_range > 0.0 {
                gate_step(&mut self.gate_key, &mut self.gate_hold, &mut self.gate_gr, click_key, g_thr, g_range, g_rel, sr)
            } else {
                self.gate_gr = 0.0;
                0.0
            };
            let gate_gain = util::db_to_gain(-gr_db);

            // PHASE stage per channel (on the lookahead-delayed 808).
            let nch = channel_samples.len().min(2);
            // Oversampling delay for the control signals that meet the product / drive.
            let (cancel_d, gate_gain) = if osf == 1 {
                (cancel, gate_gain)
            } else {
                (self.d_cancel.tick(cancel, os_lat), self.d_gate.tick(gate_gain, os_lat))
            };
            let mut dry = [0.0f32; 2];
            let mut post = [0.0f32; 2];
            let rd = (self.delay_pos + dlen - look as usize) % dlen;
            for (ch, sample) in channel_samples.iter_mut().enumerate().take(2) {
                self.delay[ch][self.delay_pos] = *sample;
                let x = if look == 0 { *sample } else { self.delay[ch][rd] };
                let (low, high) = self.split[ch].tick(x);
                // The band the mode works on: the whole 808, or just below FREQ.
                // (RINGMOD's `v * carrier * cancel` product is added below, oversampled.)
                let shaped = |v: f32| -> f32 {
                    match mode {
                        // The phase-inverted 808 mixed in at `cancel` (PhaseChain).
                        MODE_PHASE | MODE_SHAPE => v - cancel * v,
                        MODE_RING => v * (1.0 - cancel),
                        _ => v * duck_gain,
                    }
                };
                let band = if full_band { x } else { low };
                let lin = if full_band { shaped(x) } else { high + shaped(low) };
                post[ch] = if osf == 1 {
                    // OFF: same arithmetic, same order as v0.21 (bit-identical).
                    match (mode == MODE_RING, full_band) {
                        (true, true) => x * (1.0 - cancel) + x * carrier * cancel,
                        (true, false) => high + (low * (1.0 - cancel) + low * carrier * cancel),
                        _ => lin,
                    }
                } else {
                    // Oversampled: everything linear is delayed to meet the product.
                    let lin_d = self.d_lin[ch].tick(lin, os_lat);
                    self.os_ring[ch].up(band, &mut ob);
                    if ch == 0 {
                        self.os_car.up(carrier, &mut oc);
                    }
                    for k in 0..osf {
                        ob[k] *= oc[k];
                    }
                    let prod = self.os_ring[ch].down(&mut ob);
                    lin_d + if mode == MODE_RING { prod * cancel_d } else { 0.0 }
                };
                dry[ch] = if osf == 1 { x } else { self.d_dry[ch].tick(x, os_lat) };
            }
            // GATE gain on the mid only (Pro-MB: Stereo Link "Mid-only"); sides untouched.
            if nch == 2 {
                let m = 0.5 * (post[0] + post[1]) * gate_gain;
                let sd = 0.5 * (post[0] - post[1]);
                post = [m + sd, m - sd];
            } else {
                post[0] *= gate_gain;
            }
            // KICK COPY: the kick, delayed to line up with the 808, through the limiter.
            if sidechain.map(|sc| sc.len() == 1).unwrap_or(false) {
                kraw[1] = kraw[0];
            }
            let mut kick_out = [0.0f32; 2];
            {
                let lvl = kraw[0].abs().max(kraw[1].abs());
                let g = self.klim.tick(
                    lvl,
                    look as u64,
                    self.params.kick_thresh.value(),
                    self.params.kick_range.value(),
                    k_rel,
                );
                // LENGTH: time since this hit, for the (delayed) sample being output.
                let since = self.konset.tick(lvl, sr) as f32 - look as f32;
                let len_g = length_gain(since.max(0.0) / sr * 1000.0, k_len);
                for ch in 0..2 {
                    self.kdelay[ch][self.delay_pos] = kraw[ch];
                    let kd = if look == 0 { kraw[ch] } else { self.kdelay[ch][rd] };
                    let v = kd * g * len_g;
                    let v = self.kshelf[ch][0].tick(v);
                    let v = self.kshelf[ch][1].tick(v);
                    let (v, kd) = if osf == 1 {
                        (drive(v, k_drive), kd)
                    } else {
                        let mut kb = [0.0f32; MAX_FACTOR];
                        self.os_drv[ch].up(v, &mut kb);
                        for s in kb.iter_mut().take(osf) {
                            *s = drive(*s, k_drive);
                        }
                        (self.os_drv[ch].down(&mut kb), self.d_thru[ch].tick(kd, os_lat))
                    };
                    let copy = if kick_on { v * kick_level } else { 0.0 };
                    kick_out[ch] = copy + if kick_thru { kd } else { 0.0 };
                }
                // TRANSIENT SHAPER on the whole kick output, 3 bands.
                if ts_active && (kick_on || kick_thru) {
                    let mut bands = [[0.0f32; 3]; 2];
                    for ch in 0..2 {
                        let (low, rest) = self.ts_split[ch][0].tick(kick_out[ch]);
                        let (mid, high) = self.ts_split[ch][1].tick(rest);
                        bands[ch] = [low, mid, high];
                    }
                    for b in 0..3 {
                        let lvl = bands[0][b].abs().max(bands[1][b].abs());
                        let g = self.ts_band[b].tick(lvl, ts[b].0, ts[b].1, &ts_c);
                        for ch in 0..2 {
                            bands[ch][b] *= g;
                        }
                    }
                    for ch in 0..2 {
                        kick_out[ch] = bands[ch][0] + bands[ch][1] + bands[ch][2];
                    }
                }
            }
            let (mut dry_pk, mut wet_pk) = (0.0f32, 0.0f32);
            let mut io = [0.0f32; 4];
            for (ch, sample) in channel_samples.iter_mut().enumerate().take(2) {
                io[ch] = *sample;
                let wet = (post[ch] + kick_out[ch]) * out_gain;
                // MIX against the (latency-aligned) dry 808, then the bypass ramp.
                // At MIX 100% and power on this is exactly `wet`.
                let mixed = wet * mix + dry[ch] * (1.0 - mix);
                io[2 + ch] = mixed * on + dry[ch] * (1.0 - on);
                dry_pk = dry_pk.max(dry[ch].abs());
                wet_pk = wet_pk.max(post[ch].abs());
            }
            if nch < 2 {
                io[1] = io[0];
                io[3] = io[2];
            }
            // SAFE: never above -0.1 dBFS.
            let (cl, cr) = self.ceil.tick(io[2], io[3], self.params.safe.value());
            io[2] = cl;
            io[3] = cr;
            for (ch, sample) in channel_samples.iter_mut().enumerate().take(2) {
                *sample = io[2 + ch];
            }
            self.peak.feed(io[0], io[1], io[2], io[3]);

            self.delay_pos = (self.delay_pos + 1) % dlen;

            // Scope history for the editor (dropped silently if nobody is reading).
            let s = &mut self.slice;
            s.dry = s.dry.max(dry_pk);
            s.wet = s.wet.max(wet_pk);
            s.key = s.key.max(key.max(click_key * 4.0));
            let shown = match mode {
                MODE_VOLUME | MODE_COMPRESS => 1.0 - duck_gain,
                _ => cancel,
            };
            s.cancel = s.cancel.max(1.0 - (1.0 - shown) * gate_gain);
            self.slice_pos += 1;
            if self.slice_pos >= self.slice_len {
                let _ = self.scope.push(*s);
                self.slice = ScopeFrame::default();
                self.slice_pos = 0;
            }
        }

        self.peak.store(&self.meters);
        self.ceil.store(&self.meters);
    }
}

impl ClapPlugin for KickNull {
    const CLAP_ID: &'static str = "com.awknode.kicknull";
    const CLAP_DESCRIPTION: Option<&'static str> = Some("Phase-inverted sidechain: the kick cancels the 808");
    const CLAP_MANUAL_URL: Option<&'static str> = None;
    const CLAP_SUPPORT_URL: Option<&'static str> = None;
    const CLAP_FEATURES: &'static [ClapFeature] =
        &[ClapFeature::AudioEffect, ClapFeature::Stereo, ClapFeature::Mono, ClapFeature::Utility];
}

impl Vst3Plugin for KickNull {
    const VST3_CLASS_ID: [u8; 16] = *b"AwknodeKickNull1";
    const VST3_SUBCATEGORIES: &'static [Vst3SubCategory] = &[Vst3SubCategory::Fx, Vst3SubCategory::Dynamics];
}

nih_export_clap!(KickNull);
nih_export_vst3!(KickNull);

#[cfg(test)]
mod tests {
    use super::*;

    /// Simulates the chain on one sample stream: 808 = 50 Hz sine, kick = decaying burst.
    fn run(amount: f32, tension: f32, decay_ms: f32) -> Vec<(f32, f32)> {
        let sr = 48000.0;
        let (att, dec) = (coef(0.0, sr), coef(decay_ms, sr));
        let hold = (PEAK_HOLD_MS * 0.001 * sr) as u32;
        let (mut env, mut hl) = (0.0, 0u32);
        (0..(sr as usize / 2))
            .map(|n| {
                let t = n as f32 / sr;
                let x = (2.0 * std::f32::consts::PI * 50.0 * t).sin();
                let key = if t < 0.2 { (-t / 0.05).exp() } else { 0.0 };
                env = follow(env, &mut hl, key, att, dec, hold);
                let cancel = amount * curve(env, tension);
                (x, x - cancel * x)
            })
            .collect()
    }

    #[test]
    fn full_kick_fully_nulls_808() {
        let out = run(1.0, 0.62, 150.0);
        // First sample of the kick at full level: 808 fully cancelled.
        assert!(out[1].1.abs() < 1e-3, "{:?}", out[1]);
    }

    #[test]
    fn eight_oh_eight_comes_back_after_kick() {
        let out = run(1.0, 0.62, 150.0);
        let sr = 48000.0;
        let ratio = |ms: f32| {
            let i = (ms * 0.001 * sr) as usize;
            let w = 960; // one 50 Hz cycle
            let dry = out[i..i + w].iter().map(|s| s.0.abs()).fold(0.0f32, f32::max);
            let wet = out[i..i + w].iter().map(|s| s.1.abs()).fold(0.0f32, f32::max);
            wet / dry
        };
        assert!(ratio(0.0) < 0.2, "early: {}", ratio(0.0));
        assert!(ratio(120.0) > ratio(30.0), "808 should be returning");
        assert!(ratio(450.0) > 0.85, "808 should be back: {}", ratio(450.0));
    }

    /// A 50 Hz kick: the cut must not ripple with each half-cycle (the "squish").
    #[test]
    fn no_ripple_on_low_kick() {
        let sr = 48000.0;
        let (att, dec) = (coef(0.0, sr), coef(35.0, sr));
        let hold = (PEAK_HOLD_MS * 0.001 * sr) as u32;
        let (mut env, mut hl) = (0.0f32, 0u32);
        let mut envs = Vec::new();
        for n in 0..(sr as usize / 10) {
            let t = n as f32 / sr;
            let key = (2.0 * std::f32::consts::PI * 50.0 * t).sin().abs() * 0.9;
            env = follow(env, &mut hl, key, att, dec, hold);
            envs.push(env);
        }
        let steady = &envs[960..];
        let (lo, hi) = steady.iter().fold((1.0f32, 0.0f32), |(a, b), &e| (a.min(e), b.max(e)));
        assert!(hi - lo < 0.02, "envelope ripples {lo}..{hi}");
    }

    /// A deep, pitch-dropping, decaying kick with Sensitivity maxed (+24 dB):
    /// once the cut starts letting go it must not wobble back up.
    #[test]
    fn no_squish_wobble_at_max_sensitivity() {
        let sr = 48000.0;
        let (att, dec) = (coef(0.0, sr), coef(30.0, sr));
        let hold = (PEAK_HOLD_MS * 0.001 * sr) as u32;
        let sens = util::db_to_gain(24.0);
        let _ = hold;
        let (mut wf, mut ph) = (WindowFollower::default(), 0.0f32);
        let mut cuts = Vec::new();
        for n in 0..(sr as usize * 6 / 10) {
            let t = n as f32 / sr;
            let f = 35.0 + 25.0 * (-t / 0.03).exp();
            ph += 2.0 * std::f32::consts::PI * f / sr;
            let key = ph.sin().abs() * (-t / 0.12).exp() * sens;
            let env = wf.tick(key, att, dec, sr);
            cuts.push(0.8 * curve(env, 0.5));
        }
        let full = cuts.iter().position(|&c| c >= 0.79).unwrap();
        let top = full + cuts[full..].iter().position(|&c| c < 0.79).unwrap();
        let wobble: f32 = cuts[top..].windows(2).map(|w| (w[1] - w[0]).max(0.0)).sum();
        assert!(wobble < 0.02, "cut wobbles back up by {wobble} in total");
    }

    /// PhaseChain's inner Pro-MB: with lookahead, a loud kick comes out pinned at
    /// the threshold (100:1 → within ~0.2 dB) with no overshoot; quiet parts untouched.
    #[test]
    fn kick_limiter_pins_at_threshold() {
        let sr = 48000.0;
        let look = (0.010 * sr) as usize;
        let thr = util::db_to_gain(-12.0);
        let mut lim = KickLimiter::with_capacity(look + 2);
        let rel = coef(KICK_RELEASE_MS, sr);
        let x: Vec<f32> = (0..(sr as usize / 2)).map(|n| {
            let t = n as f32 / sr;
            (2.0 * std::f32::consts::PI * 55.0 * t).sin() * (-t / 0.15).exp()
        }).collect();
        let mut out = Vec::new();
        for n in 0..x.len() {
            let g = lim.tick(x[n].abs(), look as u64, thr, 60.0, rel);
            let delayed = if n >= look { x[n - look] } else { 0.0 };
            out.push(delayed * g);
        }
        let pk = out.iter().fold(0.0f32, |a, v| a.max(v.abs()));
        let pk_db = util::gain_to_db(pk);
        assert!(pk_db <= -11.8 && pk_db > -12.5, "kick copy peak {pk_db} dB, want -12");
        // well after the kick has dropped below threshold the copy is untouched
        let n = (0.45 * sr) as usize;
        assert!((out[n] - x[n - look]).abs() < 1e-6);
    }

    #[test]
    fn no_kick_is_bit_transparent() {
        let x = 0.3721f32;
        let cancel = 1.0 * curve(0.0, 0.62);
        assert_eq!(x - cancel * x, x);
    }

    #[test]
    fn lookahead_delay_line_delays_exactly() {
        let (dlen, look) = (16usize, 5usize);
        let mut buf = vec![0.0f32; dlen];
        let mut pos = 0;
        let mut out = Vec::new();
        for n in 0..12 {
            buf[pos] = n as f32;
            let rd = (pos + dlen - look) % dlen;
            out.push(buf[rd]);
            pos = (pos + 1) % dlen;
        }
        assert_eq!(&out[5..], &[0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn gate_matches_pro_mb_settings() {
        // Below threshold: nothing. 6 dB over at 100:1: ~5.94 dB. Way over: capped at range.
        assert_eq!(gate_target_gr(-24.0, -18.0, 30.0), 0.0);
        assert!((gate_target_gr(-12.0, -18.0, 30.0) - 5.94).abs() < 0.01);
        assert_eq!(gate_target_gr(0.0, -60.0, 30.0), 30.0);
        // A -6 dB click (12 dB over) settles at ~11.9 dB of GR within ~3 ms, then releases.
        let sr = 48000.0;
        let (mut k, mut h, mut gr) = (0.0, 0u32, 0.0);
        let rel = coef(15.0, sr);
        for _ in 0..(0.003 * sr) as usize {
            gate_step(&mut k, &mut h, &mut gr, 0.5, -18.0, 30.0, rel, sr);
        }
        assert!((gr - 11.88).abs() < 0.3, "gr {gr}");
        for _ in 0..(0.1 * sr) as usize {
            gate_step(&mut k, &mut h, &mut gr, 0.0, -18.0, 30.0, rel, sr);
        }
        assert!(gr < 0.5, "released gr {gr}");
    }

    #[test]
    fn kickstart_curve_shape() {
        let len = curve_seconds(3.0, 120.0); // 1/4 at 120 bpm
        assert!((len - 0.5).abs() < 1e-6);
        assert_eq!(curve_duck(-0.001, len, 0.5), 0.0);
        assert!(curve_duck(0.0001, len, 0.5) < 0.25, "fade-in avoids a click");
        assert!(curve_duck(0.003, len, 0.5) > 0.98, "full dip 3 ms in");
        let (early, late) = (curve_duck(0.1, len, 0.5), curve_duck(0.4, len, 0.5));
        assert!(early > late && late > 0.0);
        assert_eq!(curve_duck(len, len, 0.5), 0.0);
        // Higher shape = 808 back sooner.
        assert!(curve_duck(0.1, len, 0.9) < curve_duck(0.1, len, 0.3));
    }

    /// A real low kick (55 Hz body, decaying) must fire ONE curve, with no jumps.
    #[test]
    fn curve_trigger_no_crackle() {
        let sr = 48000.0;
        let len_s = curve_seconds(3.0, 140.0);
        let mut trig = CurveTrigger::default();
        let mut prev = 0.0f32;
        let mut triggers = 0;
        let mut max_jump = 0.0f32;
        for n in 0..(sr as usize) {
            let t = n as f32 / sr;
            // Two kicks, 0 s and 0.43 s (one beat at 140 bpm).
            let kick = |t0: f32| {
                let tt = t - t0;
                if tt < 0.0 { 0.0 } else { (2.0 * std::f32::consts::PI * 55.0 * tt).sin() * (-tt / 0.15).exp() * 0.9 }
            };
            let key = (kick(0.0) + kick(0.4286)).abs();
            let was = trig.pos;
            let d = trig.tick(key, util::db_to_gain(-18.0), len_s, 0.5, sr);
            if trig.pos == Some(1) && was != Some(0) { triggers += 1; }
            // Ignore the deliberate 0.5 ms fade-in steps (max 1/24 per sample at 48 kHz).
            max_jump = max_jump.max((d - prev).abs());
            prev = d;
        }
        assert_eq!(triggers, 2, "one curve per kick");
        assert!(max_jump < 0.05, "level jumped by {max_jump}");
    }

    /// Crossover: bands sum flat with no duck; ducking the low band removes 50 Hz
    /// and leaves 2 kHz alone.
    #[test]
    fn crossover_band_duck() {
        let sr = 48000.0;
        let rms = |hz: f32, cancel: f32| {
            let mut xo = Crossover::default();
            xo.set(150.0, sr);
            let (mut acc, mut n) = (0.0f32, 0);
            for k in 0..(sr as usize) {
                let x = (2.0 * std::f32::consts::PI * hz * k as f32 / sr).sin();
                let (lo, hi) = xo.tick(x);
                let y = hi + (1.0 - cancel) * lo;
                if k > sr as usize / 2 {
                    acc += y * y;
                    n += 1;
                }
            }
            (acc / n as f32).sqrt() / std::f32::consts::FRAC_1_SQRT_2
        };
        for hz in [40.0, 150.0, 1000.0] {
            assert!((rms(hz, 0.0) - 1.0).abs() < 0.02, "not flat at {hz}: {}", rms(hz, 0.0));
        }
        assert!(rms(50.0, 1.0) < 0.15, "50 Hz left: {}", rms(50.0, 1.0));
        assert!((rms(2000.0, 1.0) - 1.0).abs() < 0.02, "2 kHz changed: {}", rms(2000.0, 1.0));
    }

    #[test]
    fn tension_bends_curve() {
        assert!((curve(0.5, 0.5) - 0.5).abs() < 1e-6);
        assert!(curve(0.5, 0.8) > 0.5);
        assert!(curve(0.5, 0.2) < 0.5);
    }
}

#[cfg(test)]
mod dump_defaults;

/// v0.22: oversampling alignment, MIX, bypass and the new defaults, run through the real audio path.
#[cfg(test)]
mod os_tests {
    use super::*;

    struct Ctx;
    impl InitContext<KickNull> for Ctx {
        fn plugin_api(&self) -> PluginApi {
            PluginApi::Clap
        }
        fn execute(&self, _: ()) {}
        fn set_latency_samples(&self, _: u32) {}
        fn set_current_voice_capacity(&self, _: u32) {}
    }

    const SR: f32 = 48000.0;
    const N: usize = 24000;

    fn plugin(edit: impl FnOnce(&mut KickNullParams)) -> KickNull {
        let mut p = KickNullParams::default();
        p.lookahead = FloatParam::new("Lookahead", 0.0, FloatRange::Linear { min: 0.0, max: MAX_LOOKAHEAD_MS });
        p.ts_on = BoolParam::new("Shaper", false);
        edit(&mut p);
        // No host here to prime the smoothers.
        p.output.smoothed.reset(p.output.value());
        p.mix.smoothed.reset(p.mix.value());
        let mut k = KickNull { params: Arc::new(p), ..Default::default() };
        let cfg = BufferConfig { sample_rate: SR, min_buffer_size: None, max_buffer_size: 512, process_mode: ProcessMode::Realtime };
        assert!(k.initialize(&KickNull::AUDIO_IO_LAYOUTS[0], &cfg, &mut Ctx));
        k
    }

    fn os_param_at(v: i32) -> IntParam {
        IntParam::new("Oversampling", v, IntRange::Linear { min: 0, max: 3 })
    }

    /// 808 = 45 Hz sine; kick = 55 Hz sine burst every 250 ms (low, so the nonlinear
    /// products stay far below Nyquist and OFF vs oversampled should match closely).
    fn signals() -> ([Vec<f32>; 2], [Vec<f32>; 2]) {
        let tau = 2.0 * std::f32::consts::PI;
        let b: Vec<f32> = (0..N).map(|n| 0.6 * (tau * 45.0 * n as f32 / SR).sin()).collect();
        let k: Vec<f32> = (0..N)
            .map(|n| {
                let t = (n % 12000) as f32 / SR;
                0.9 * (-t / 0.06).exp() * (tau * 55.0 * t).sin()
            })
            .collect();
        ([b.clone(), b], [k.clone(), k])
    }

    fn run(k: &mut KickNull) -> Vec<f32> {
        let (mut main, mut kick) = signals();
        for start in (0..N).step_by(512) {
            let end = (start + 512).min(N);
            let [ml, mr] = &mut main;
            let [kl, kr] = &mut kick;
            let mut buf = Buffer::default();
            let mut sc = Buffer::default();
            unsafe {
                buf.set_slices(end - start, |v| {
                    v.clear();
                    v.push(&mut ml[start..end]);
                    v.push(&mut mr[start..end]);
                });
                sc.set_slices(end - start, |v| {
                    v.clear();
                    v.push(&mut kl[start..end]);
                    v.push(&mut kr[start..end]);
                });
            }
            k.render(&mut buf, Some(sc.as_slice_immutable()), 120.0, false, 0.0);
        }
        main[0].clone()
    }

    fn max_err(a: &[f32], b: &[f32], shift: usize) -> f32 {
        (2000..N).map(|n| (a[n] - b[n - shift]).abs()).fold(0.0, f32::max)
    }

    #[test]
    fn new_defaults() {
        let p = KickNullParams::default();
        assert_eq!(p.amount.default_plain_value(), 1.0);
        assert_eq!(p.vol_depth.default_plain_value(), 60.0);
        assert_eq!(p.mix.default_plain_value(), 1.0);
        assert!(!p.bypass.default_plain_value());
        assert_eq!(p.os.default_plain_value(), 0);
        assert_eq!(KickNull::VENDOR, "awk studios");
    }

    #[test]
    fn reported_latency_is_lookahead_plus_os() {
        for os in 0..4 {
            let k = plugin(|p| {
                p.os = os_param_at(os);
                p.lookahead = FloatParam::new("Lookahead", 5.0, FloatRange::Linear { min: 0.0, max: MAX_LOOKAHEAD_MS });
            });
            assert_eq!(k.latency as usize, 240 + oversample::latency(oversample::factor_for(os)), "os {os}");
        }
    }

    /// RINGMOD with oversampling = the OFF result, delayed by exactly the reported extra latency.
    #[test]
    fn ringmod_oversampled_lines_up() {
        let mk = |os| {
            plugin(|p| {
                p.os = os_param_at(os);
                p.duck_mode = IntParam::new("Mode", MODE_RING, IntRange::Linear { min: 0, max: 4 });
                p.kick_on = BoolParam::new("Kick Copy", false);
            })
        };
        let base = run(&mut mk(0));
        for os in 1..4 {
            let d = oversample::latency(oversample::factor_for(os));
            let out = run(&mut mk(os));
            let e = max_err(&out, &base, d);
            assert!(e < 0.02, "os {os}: max error {e}");
            // One sample off would be far worse: proves the alignment is exact.
            let off = max_err(&out, &base, d + 1);
            assert!(off > 5.0 * e, "os {os}: misaligned? {e} vs {off}");
        }
    }

    /// KICK COPY drive (and KICK THRU) with oversampling lines up the same way.
    #[test]
    fn drive_oversampled_lines_up() {
        let mk = |os| {
            plugin(|p| {
                p.os = os_param_at(os);
                p.kick_drive = FloatParam::new("Kick Drive", 12.0, FloatRange::Linear { min: 0.0, max: 24.0 });
                p.kick_thru = BoolParam::new("Kick Thru", true);
            })
        };
        let base = run(&mut mk(0));
        for os in 1..4 {
            let d = oversample::latency(oversample::factor_for(os));
            let out = run(&mut mk(os));
            let e = max_err(&out, &base, d);
            assert!(e < 0.03, "os {os}: max error {e}");
            let off = max_err(&out, &base, d + 1);
            assert!(off > 5.0 * e, "os {os}: misaligned? {e} vs {off}");
        }
    }

    /// MIX 0% = the untouched 808, delayed by exactly the reported latency.
    #[test]
    fn mix_zero_is_aligned_dry() {
        for os in 0..4 {
            let mut k = plugin(|p| {
                p.os = os_param_at(os);
                p.lookahead = FloatParam::new("Lookahead", 2.0, FloatRange::Linear { min: 0.0, max: MAX_LOOKAHEAD_MS });
                p.mix = FloatParam::new("Mix", 0.0, FloatRange::Linear { min: 0.0, max: 1.0 });
            });
            let lat = k.latency as usize;
            let out = run(&mut k);
            let (dry, _) = signals();
            assert_eq!(max_err(&out, &dry[0], lat), 0.0, "os {os}");
        }
    }

    /// Power off (bypass) settles to the dry 808, latency-aligned, with no jump.
    #[test]
    fn bypass_passes_aligned_dry() {
        for os in [0, 3] {
            let mut k = plugin(|p| {
                p.os = os_param_at(os);
                p.bypass = BoolParam::new("Bypass", true).make_bypass();
            });
            let lat = k.latency as usize;
            let out = run(&mut k);
            let (dry, _) = signals();
            let e = (12000..N).map(|n| (out[n] - dry[0][n - lat]).abs()).fold(0.0f32, f32::max);
            assert!(e < 1e-6, "os {os}: {e}");
            let jump = out.windows(2).map(|w| (w[1] - w[0]).abs()).fold(0.0f32, f32::max);
            assert!(jump < 0.1, "os {os}: click {jump}");
        }
    }
}
