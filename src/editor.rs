//! KICKNULL editor, awk studios "Machined" look (red accent).
//! Header: presets, A/B, undo/redo, power (bypass), MIX.
//! - LIVE scope: the 808 over the last ~2.5 s on a dB scale. Red = what the kick
//!   takes away, grey = what's left, thin line = the kick (sidechain).
//! - THE DIP: what one kick hit does to the 808 with the current settings (ms / dB axes).
//!   The mode buttons on top pick the SIDECHAIN knobs.
//! - Rows: SIDECHAIN, KICK COPY, TRANSIENT. Right: IN / OUT meters with dB scale.
//!   Footer: OVERSAMPLING (RINGMOD product + KICK DRIVE) and the total latency.
//! Knobs: drag (Ctrl or Shift = fine), scroll to nudge, double-click to reset.

use crate::awk_ui::*;
use crate::oversample;
use crate::presets::PresetBar;
use crate::{
    coef, comp_gr_db, curve, curve_duck, curve_seconds, follow, gate_step, shape_at, KickNullParams, ScopeFrame,
    FULL_BAND_HZ, MODE_COMPRESS, MODE_NAMES, MODE_RING, MODE_SHAPE, MODE_VOLUME, NOTE_BEATS, NOTE_NAMES,
};
use crossbeam::queue::ArrayQueue;
use nih_plug::prelude::*;
use nih_plug_egui::create_egui_editor;
use nih_plug_egui::egui::{self, pos2, vec2, Align2, Color32, Painter, Pos2, Rect};
use std::collections::VecDeque;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;

pub const WIDTH: u32 = 960;
pub const HEIGHT: u32 = 770;
const ACC: Color32 = rgb(0xff, 0x4b, 0x4b);
/// Scope history: 640 slices of 4 ms = 2.56 s.
const HISTORY: usize = 640;
const SLICE_S: f32 = 0.004;
/// Scope dB range (centre line = this, edges = 0 dBFS).
const SCOPE_FLOOR: f32 = -48.0;
/// THE DIP dB range.
const DIP_FLOOR: f32 = -48.0;

struct St {
    history: VecDeque<ScopeFrame>,
    presets: PresetBar,
    skin: Skin,
}

pub fn create(
    params: Arc<KickNullParams>,
    scope: Arc<ArrayQueue<ScopeFrame>>,
    bpm: Arc<AtomicU32>,
    meters: Arc<Meters>,
) -> Option<Box<dyn Editor>> {
    let state = params.editor_state.clone();
    let st = St {
        history: VecDeque::from(vec![ScopeFrame::default(); HISTORY]),
        presets: PresetBar::new("KickNull"),
        skin: Skin::new(Look::Machined, ACC, "KICKNULL", env!("CARGO_PKG_VERSION"), ""),
    };
    create_egui_editor(
        state,
        st,
        |ctx, _| install_fonts(ctx),
        move |ctx, setter, st| frame(ctx, setter, st, &params, &scope, &bpm, &meters),
    )
}

/// A row of knobs spread across a panel. `dim(i)` greys out knob i.
#[allow(clippy::too_many_arguments)]
fn knob_row(
    ui: &mut egui::Ui,
    setter: &ParamSetter,
    skin: &Skin,
    r: Rect,
    y: f32,
    rad: f32,
    knobs: &[(&str, &FloatParam, bool)],
    dim: impl Fn(usize) -> bool,
) {
    let n = knobs.len() as f32;
    let cell = (r.width() - 16.0) / n;
    for (i, (label, param, bip)) in knobs.iter().enumerate() {
        let x = r.left() + 8.0 + cell * (i as f32 + 0.5);
        skin.knob(ui, setter, pos2(x, y), rad, label, *param, false, *bip, dim(i));
    }
}

/// dBFS of a linear peak, floored.
fn db_of(g: f32, floor: f32) -> f32 {
    if g <= 0.0 {
        floor
    } else {
        (20.0 * g.log10()).max(floor)
    }
}

fn draw_scope(painter: &Painter, skin: &Skin, r: Rect, history: &VecDeque<ScopeFrame>, params: &KickNullParams) {
    let (sp, pal) = skin.screen(painter, r);
    let (gl, gr, top, bot) = (r.left() + 46.0, r.right() - 12.0, r.top() + 26.0, r.bottom() - 18.0);
    let mid = (top + bot) / 2.0;
    let half = (bot - top) / 2.0;
    // Distance from the centre line for a level in dBFS (centre = SCOPE_FLOOR, edges = 0 dB).
    let off = |db: f32| (db.clamp(SCOPE_FLOOR, 0.0) - SCOPE_FLOOR) / -SCOPE_FLOOR * half;

    // dB grid, labelled on the upper half (the lower half mirrors it).
    for d in [0.0f32, -12.0, -24.0, -36.0] {
        let o = off(d);
        for (j, y) in [mid - o, mid + o].into_iter().enumerate() {
            let yy = y.round() + 0.5;
            line(&sp, pos2(gl, yy), pos2(gr, yy), if d == 0.0 { pal.grid_hi } else { pal.grid }, 1.0);
            if j == 0 {
                txt(&sp, pos2(gl - 6.0, yy), Align2::RIGHT_CENTER, format!("{d:.0}"), mono(8.5), pal.axis, 0.0);
            }
        }
    }
    line(&sp, pos2(gl, mid.round() + 0.5), pos2(gr, mid.round() + 0.5), pal.grid_hi, 1.0);
    txt(&sp, pos2(gl - 6.0, mid), Align2::RIGHT_CENTER, format!("{SCOPE_FLOOR:.0}"), mono(8.5), pal.axis, 0.0);
    txt(&sp, pos2(gl - 6.0, top - 9.0), Align2::RIGHT_CENTER, "dBFS", mono(8.0), pal.axis, 0.0);

    // Time grid: history spans HISTORY * 4 ms; "NOW" at the right edge.
    let span_s = HISTORY as f32 * SLICE_S;
    let mut s = 0.0f32;
    while s <= span_s + 1e-3 {
        let x = gr - s / span_s * (gr - gl);
        line(&sp, pos2(x.round() + 0.5, top), pos2(x.round() + 0.5, bot), pal.grid, 1.0);
        let lab_s = if s == 0.0 { "NOW".to_string() } else { format!("-{s:.1} s") };
        let a = if s == 0.0 { Align2::RIGHT_TOP } else { Align2::CENTER_TOP };
        txt(&sp, pos2(x, bot + 3.0), a, lab_s, mono(8.5), pal.axis, 0.0);
        s += 0.5;
    }

    // Bars: red = removed (dry beyond what's left), grey = what's left of the 808.
    let n = history.len().max(1);
    let cols = (gr - gl).floor() as usize;
    let mut kick = Vec::with_capacity(cols);
    for c in 0..cols {
        let f = history[(c * n / cols.max(1)).min(n - 1)];
        let x = gl + c as f32 + 0.5;
        let dry = off(db_of(f.dry, SCOPE_FLOOR));
        let wet = off(db_of(f.wet, SCOPE_FLOOR));
        if dry > wet + 0.5 {
            line(&sp, pos2(x, mid - dry), pos2(x, mid - wet), al(ACC, 0.8), 1.0);
            line(&sp, pos2(x, mid + wet), pos2(x, mid + dry), al(ACC, 0.8), 1.0);
        }
        if wet > 0.5 {
            line(&sp, pos2(x, mid - wet), pos2(x, mid + wet), Color32::from_white_alpha(74), 1.0);
        }
        // The kick: its level on the same dB scale, upper half.
        kick.push(pos2(x, mid - off(db_of(f.key, SCOPE_FLOOR))));
    }
    polyline(&sp, kick, al(pal.trace, 0.85), 1.2, false);

    // Header line + live readouts.
    let (mut cancel, mut key) = (0.0f32, 0.0f32);
    for f in history.iter().rev().take(30) {
        cancel = cancel.max(f.cancel);
        key = key.max(f.key);
    }
    let cut = if cancel >= 0.999 {
        "-inf".to_string()
    } else if cancel < 0.001 {
        "0.0".to_string()
    } else {
        format!("{:.1}", 20.0 * (1.0 - cancel).log10())
    };
    txt(&sp, pos2(r.left() + 22.0, r.top() + 15.0), Align2::LEFT_CENTER, "LIVE · 808", labb(12.5), rgb(0xc9, 0xce, 0xd5), 1.5);
    let lg = txt(&sp, pos2(r.left() + 110.0, r.top() + 15.0), Align2::LEFT_CENTER, "RED = TAKEN BY THE KICK", mono(9.0), al(ACC, 0.9), 0.5);
    let lg = txt(&sp, pos2(lg.right() + 14.0, r.top() + 15.0), Align2::LEFT_CENTER, "GREY = 808 LEFT", mono(9.0), Color32::from_white_alpha(140), 0.5);
    txt(&sp, pos2(lg.right() + 14.0, r.top() + 15.0), Align2::LEFT_CENTER, "LINE = KICK", mono(9.0), pal.trace, 0.5);
    let key_s = if params.click.value() { "CLICK 300-3K" } else { "FULL KICK" };
    let lit = key > 0.02;
    let rr = txt(&sp, pos2(gr, r.top() + 15.0), Align2::RIGHT_CENTER, format!("CUT {cut} dB  ·  KEY {key_s}"), mono(10.0), ACC, 0.5);
    let lamp = pos2(rr.left() - 12.0, r.top() + 15.0);
    if lit {
        sp.circle_filled(lamp, 7.0, al(ACC, 0.2));
    }
    sp.circle_filled(lamp, 3.5, if lit { ACC } else { rgb(0x3a, 0x1c, 0x1c) });
    skin.screen_end(painter, r);
}

/// What one kick does to the 808 with the current settings: (span in s, 808 level, kick level), both linear.
fn simulate(params: &KickNullParams, bpm: f32, cols: usize) -> (f32, Vec<f32>, Vec<f32>) {
    let sim_sr = 4000.0;
    let mode = params.duck_mode.value();
    let curve_mode = mode == MODE_SHAPE;
    let sync = params.beat_sync.value();
    let kind = (params.shape_kind.value().round() as usize).min(7);
    let len_s = curve_seconds(params.length.value(), bpm);
    let span = if curve_mode { (len_s + 0.08).max(0.6) } else { 0.6 };
    let (c_thr, c_ratio, c_knee) =
        (util::gain_to_db(params.comp_thresh.value()), params.comp_ratio.value(), params.comp_knee.value());
    let (c_att, c_rel) = (coef(params.comp_attack.value(), sim_sr), coef(params.comp_release.value(), sim_sr));
    let mut c_gr = 0.0f32;
    let steps = (sim_sr * span) as usize;
    let sens = params.sensitivity.value();
    let (att, dec) = (coef(params.attack.value(), sim_sr), coef(params.decay.value(), sim_sr));
    let (tension, amount) = (params.tension.value(), params.amount.value());
    let g_thr = util::gain_to_db(params.gate_thresh.value());
    let g_range = params.gate_range.value();
    let g_rel = coef(params.gate_release.value(), sim_sr);
    let (mut gk, mut gh, mut ggr) = (0.0f32, 0u32, 0.0f32);
    let hold = (crate::follow_hold_ms(params.decay.value()) * 0.001 * sim_sr) as u32;
    // With lookahead the cut starts before the kick lands: shift the kick earlier.
    let look = params.lookahead.value() * 0.001;
    let (mut env, mut hl) = (0.0f32, 0u32);
    let mut level = Vec::with_capacity(steps);
    let mut kick = Vec::with_capacity(steps);
    // Kick lands at 30 ms on this graph; it hits 0 dBFS and dies away over ~70 ms.
    let kick_at = |t: f32| -> f32 {
        let t = t - 0.03;
        if t < 0.0 {
            0.0
        } else if t < 0.0015 {
            t / 0.0015
        } else {
            (-(t - 0.0015) / 0.07).exp()
        }
    };
    for n in 0..steps {
        let t = n as f32 / sim_sr;
        let k = kick_at(t + look) * sens;
        env = follow(env, &mut hl, k, att, dec, hold);
        // The click part of a kick: short and about 6 dB under the body.
        let click = {
            let tc = t + look - 0.03;
            if tc < 0.0 {
                0.0
            } else {
                0.5 * (-tc / 0.012).exp()
            }
        };
        let duck = if curve_mode && sync {
            shape_at(kind, t / len_s, params.shape.value())
        } else if curve_mode {
            let fire = 0.03 - look;
            let d = if kind == 0 {
                curve_duck(t - fire, len_s, params.shape.value())
            } else {
                ((t - fire) / crate::CURVE_FADE_S).clamp(0.0, 1.0) * shape_at(kind, (t - fire) / len_s, params.shape.value())
            };
            if sens >= params.gate_thresh.value() {
                d
            } else {
                0.0
            }
        } else if mode == MODE_COMPRESS {
            let target = comp_gr_db(util::gain_to_db(k.max(1e-6)), c_thr, c_ratio, c_knee);
            let c = if target > c_gr { c_att } else { c_rel };
            c_gr = target + (c_gr - target) * c;
            1.0 - util::db_to_gain(-c_gr)
        } else {
            curve(env, tension)
        };
        let gr = if g_range > 0.0 { gate_step(&mut gk, &mut gh, &mut ggr, click, g_thr, g_range, g_rel, sim_sr) } else { 0.0 };
        let remain = match mode {
            MODE_VOLUME => util::db_to_gain(-duck * params.vol_depth.value()),
            MODE_COMPRESS => 1.0 - duck,
            MODE_RING => 1.0 - amount * duck * 0.5, // half the ducked part comes back as ring tone
            _ => 1.0 - amount * duck,
        };
        level.push(remain * util::db_to_gain(-gr));
        kick.push((kick_at(t) * sens).min(1.0));
    }
    let at = |c: usize| (c * steps / cols.max(1)).min(steps.saturating_sub(1));
    let lv = (0..cols).map(|c| level[at(c)]).collect();
    let kv = (0..cols).map(|c| kick[at(c)]).collect();
    (span, lv, kv)
}

/// THE DIP: 808 level after one kick, in dB against ms.
fn draw_dip(painter: &Painter, skin: &Skin, r: Rect, params: &KickNullParams, bpm: f32) {
    let (sp, pal) = skin.screen(painter, r);
    let (gl, gr, top, bot) = (r.left() + 36.0, r.right() - 10.0, r.top() + 46.0, r.bottom() - 22.0);
    let py = |db: f32| top + (db.clamp(DIP_FLOOR, 0.0) / DIP_FLOOR) * (bot - top);
    let cols = (gr - gl).max(1.0) as usize;
    let (span, level, kick) = simulate(params, bpm, cols);
    let px = |s: f32| gl + s / span * (gr - gl);

    for d in [0.0f32, -12.0, -24.0, -36.0, -48.0] {
        let yy = py(d).round() + 0.5;
        line(&sp, pos2(gl, yy), pos2(gr, yy), if d == 0.0 { pal.grid_hi } else { pal.grid }, 1.0);
        txt(&sp, pos2(gl - 6.0, yy), Align2::RIGHT_CENTER, format!("{d:.0}"), mono(8.5), pal.axis, 0.0);
    }
    txt(&sp, pos2(gl - 6.0, top - 11.0), Align2::RIGHT_CENTER, "dB", mono(8.5), pal.axis, 0.0);
    let span_ms = span * 1000.0;
    let step = [25.0f32, 50.0, 100.0, 200.0, 250.0, 500.0, 1000.0].into_iter().find(|s| span_ms / s <= 6.0).unwrap_or(2000.0);
    let mut m = 0.0f32;
    while m <= span_ms + 0.5 {
        let xx = px(m / 1000.0).round() + 0.5;
        line(&sp, pos2(xx, top), pos2(xx, bot), pal.grid, 1.0);
        if xx < gr - 26.0 {
            txt(&sp, pos2(xx, bot + 4.0), Align2::CENTER_TOP, format!("{m:.0}"), mono(8.5), pal.axis, 0.0);
        }
        m += step;
    }
    txt(&sp, pos2(gr, bot + 4.0), Align2::RIGHT_TOP, "ms", mono(8.5), pal.axis, 0.0);

    // Kick landing mark (30 ms on this graph).
    let kx = px(0.03).round() + 0.5;
    dashed(&sp, pos2(kx, top), pos2(kx, bot), al(pal.trace, 0.3), 1.0, 3.0, 3.0);

    // Kick level (dBFS) and the 808 level after the cut.
    let kpts: Vec<Pos2> = kick.iter().enumerate().map(|(c, &k)| pos2(gl + c as f32, py(db_of(k, DIP_FLOOR)))).collect();
    polyline(&sp, kpts, al(pal.trace, 0.35), 1.0, false);
    let pts: Vec<Pos2> = level.iter().enumerate().map(|(c, &v)| pos2(gl + c as f32, py(db_of(v, DIP_FLOOR)))).collect();
    // Shade what's taken: between 0 dB and the curve.
    for p in &pts {
        if p.y > top + 0.5 {
            line(&sp, pos2(p.x + 0.5, top), pos2(p.x + 0.5, p.y), al(ACC, 0.16), 1.0);
        }
    }
    polyline(&sp, pts, ACC, 2.0, false);

    let min = level.iter().copied().fold(1.0f32, f32::min);
    let max_cut = if min <= 1e-5 { "-inf".to_string() } else { format!("{:.1}", 20.0 * min.log10()) };
    txt(&sp, pos2(r.left() + 22.0, r.top() + 15.0), Align2::LEFT_CENTER, "THE DIP", labb(13.0), rgb(0xe6, 0xe9, 0xec), 2.5);
    let band = if params.freq.value() >= FULL_BAND_HZ - 1.0 {
        "FULL BAND".to_string()
    } else {
        format!("BELOW {}", params.freq.normalized_value_to_string(params.freq.unmodulated_normalized_value(), true))
    };
    txt(&sp, pos2(gr, r.top() + 15.0), Align2::RIGHT_CENTER, band, mono(9.0), pal.axis, 0.5);
    let step_i = (params.length.value().round() as usize).min(5);
    let mut info = format!("MAX {max_cut} dB  ·  {bpm:.0} BPM");
    if params.duck_mode.value() == MODE_SHAPE {
        let ms = NOTE_BEATS[step_i] * 60.0 / bpm.max(20.0) * 1000.0;
        info.push_str(&format!("  ·  {} = {ms:.0} ms", NOTE_NAMES[step_i]));
    }
    txt(&sp, pos2(r.left() + 22.0, r.top() + 32.0), Align2::LEFT_CENTER, info, mono(9.5), ACC, 0.3);
    skin.screen_end(painter, r);
}

/// One editor frame (pulled out of the update closure so tests can render it).
fn frame(ctx: &egui::Context, setter: &ParamSetter, st: &mut St, params: &Arc<KickNullParams>, scope: &Arc<ArrayQueue<ScopeFrame>>, bpm: &Arc<AtomicU32>, meters: &Arc<Meters>) {
            while let Some(f) = scope.pop() {
                st.history.push_back(f);
            }
            while st.history.len() > HISTORY {
                st.history.pop_front();
            }
            ctx.request_repaint_after(std::time::Duration::from_millis(33));
            st.skin.begin(ctx);
            let p = &params;
            let tempo = f32::from_bits(bpm.load(Ordering::Relaxed));
            egui::CentralPanel::default().frame(egui::Frame::NONE.fill(rgb(0x1c, 0x1f, 0x23))).show(ctx, |ui| {
                let whole = ui.max_rect();
                st.skin.background(ui.painter(), whole);
                let full = st.presets.strip(ui, setter, params.as_ref(), whole, &st.skin, &p.mix, &p.bypass);
                let painter = ui.painter().clone();
                let (l, t) = (full.left(), full.top());
                let foot = Rect::from_min_max(pos2(l, full.bottom() - 30.0), full.right_bottom());
                let bottom = foot.top() - 14.0;
                let skin = &mut st.skin;

                // ---- LIVE scope
                let scope_r = Rect::from_min_max(pos2(l + 14.0, t + 14.0), pos2(l + 836.0, t + 174.0));
                draw_scope(&painter, skin, scope_r, &st.history, p);

                // ---- THE DIP (with the mode buttons)
                let dip_card = Rect::from_min_max(pos2(l + 14.0, scope_r.bottom() + 12.0), pos2(l + 350.0, bottom));
                skin.panel(&painter, dip_card, "", "");
                let mode = p.duck_mode.value();
                {
                    let inner_w = dip_card.width() - 24.0;
                    let font = lab(11.5);
                    let widths: Vec<f32> = MODE_NAMES.iter().map(|m| txt_w(&painter, m, font.clone(), 1.2)).collect();
                    let gap = 5.0;
                    let pad = ((inner_w - widths.iter().sum::<f32>() - gap * 4.0) / 5.0).max(6.0);
                    let mut x = dip_card.left() + 12.0;
                    for (i, m) in MODE_NAMES.iter().enumerate() {
                        let r = Rect::from_min_size(pos2(x, dip_card.top() + 10.0), vec2(widths[i] + pad, 20.0));
                        if skin.chip(ui, r, m, mode == i as i32, ("mode", i)).clicked() {
                            setter.begin_set_parameter(&p.duck_mode);
                            setter.set_parameter(&p.duck_mode, i as i32);
                            setter.end_set_parameter(&p.duck_mode);
                        }
                        x = r.right() + gap;
                    }
                }
                let dip_r = Rect::from_min_max(dip_card.left_top() + vec2(10.0, 40.0), dip_card.right_bottom() - vec2(10.0, 10.0));
                draw_dip(&painter, skin, dip_r, p, tempo);

                // ---- rows
                let rx0 = dip_card.right() + 12.0;
                let rx1 = scope_r.right();
                let gap = 12.0;
                let rh = (bottom - dip_card.top() - 2.0 * gap) / 3.0;
                let row = |i: usize| {
                    let y = dip_card.top() + i as f32 * (rh + gap);
                    Rect::from_min_max(pos2(rx0, y), pos2(rx1, y + rh))
                };
                let knob_y = |r: Rect| r.top() + 30.0 + (r.height() - 30.0) * 0.36;

                // SIDECHAIN: 5 knobs for the mode, then FREQ and OUTPUT.
                let sc = row(0);
                skin.panel(&painter, sc, "SIDECHAIN", "");
                let mode_knobs: [(&str, &FloatParam, bool); 5] = match mode {
                    MODE_VOLUME => [
                        ("SENS", &p.sensitivity, false),
                        ("ATTACK", &p.attack, false),
                        ("DECAY", &p.decay, false),
                        ("TENSION", &p.tension, false),
                        ("DEPTH", &p.vol_depth, false),
                    ],
                    MODE_COMPRESS => [
                        ("THRESH", &p.comp_thresh, false),
                        ("RATIO", &p.comp_ratio, false),
                        ("ATTACK", &p.comp_attack, false),
                        ("RELEASE", &p.comp_release, false),
                        ("KNEE", &p.comp_knee, false),
                    ],
                    MODE_SHAPE => [
                        ("SHAPE", &p.shape_kind, false),
                        ("LENGTH", &p.length, false),
                        ("BEND", &p.shape, false),
                        ("SENS", &p.sensitivity, false),
                        ("AMOUNT", &p.amount, false),
                    ],
                    // PHASE and RINGMOD
                    _ => [
                        ("SENS", &p.sensitivity, false),
                        ("ATTACK", &p.attack, false),
                        ("DECAY", &p.decay, false),
                        ("TENSION", &p.tension, false),
                        ("AMOUNT", &p.amount, false),
                    ],
                };
                let sc_knobs: Vec<(&str, &FloatParam, bool)> =
                    mode_knobs.iter().copied().chain([("FREQ", &p.freq, false), ("OUTPUT", &p.output, false)]).collect();
                knob_row(ui, setter, skin, sc, knob_y(sc), 22.0, &sc_knobs, |_| false);
                let mut cx = sc.left() + 128.0;
                if mode == MODE_SHAPE {
                    let r = Rect::from_min_size(pos2(cx, sc.top() + 7.0), vec2(128.0, 18.0));
                    skin.chip_param(ui, setter, r, &p.beat_sync, ("TRIG: BEAT SYNC", "TRIG: EVERY KICK"));
                    cx = r.right() + 6.0;
                }
                let r = Rect::from_min_size(pos2(cx, sc.top() + 7.0), vec2(132.0, 18.0));
                skin.chip_param(ui, setter, r, &p.click, ("KEY: CLICK 300-3K", "KEY: FULL KICK"));

                // KICK COPY (PhaseChain's inner Pro-MB). LOOKAHEAD moves the whole plugin, so it never dims.
                let kc = row(1);
                skin.panel(&painter, kc, "KICK COPY", "");
                skin.power(ui, setter, Rect::from_min_size(pos2(kc.left() + 128.0, kc.top() + 8.0), vec2(30.0, 16.0)), &p.kick_on);
                let r = Rect::from_min_size(pos2(kc.left() + 168.0, kc.top() + 7.0), vec2(118.0, 18.0));
                skin.chip_param(ui, setter, r, &p.kick_thru, ("KICK THRU: ON", "KICK THRU: OFF"));
                if p.kick_thru.value() {
                    skin.small(&painter, pos2(r.right() + 10.0, r.center().y), Align2::LEFT_CENTER, "unroute the kick's own track", skin.text_faint());
                }
                let kick_dim = !p.kick_on.value();
                let kc_knobs: [(&str, &FloatParam, bool); 7] = [
                    ("LOOKAHEAD", &p.lookahead, false),
                    ("THRESH", &p.kick_thresh, false),
                    ("LEVEL", &p.kick_level, false),
                    ("LOW", &p.kick_low, true),
                    ("HIGH", &p.kick_high, true),
                    ("DRIVE", &p.kick_drive, false),
                    ("LENGTH", &p.kick_length, false),
                ];
                knob_row(ui, setter, skin, kc, knob_y(kc), 22.0, &kc_knobs, |i| kick_dim && i > 0);

                // TRANSIENT shaper on the kick output.
                let tr = row(2);
                skin.panel(&painter, tr, "TRANSIENT", "");
                skin.power(ui, setter, Rect::from_min_size(pos2(tr.left() + 128.0, tr.top() + 8.0), vec2(30.0, 16.0)), &p.ts_on);
                skin.small(&painter, pos2(tr.left() + 168.0, tr.top() + 16.0), Align2::LEFT_CENTER, "3-BAND, ON THE KICK OUTPUT", skin.text_faint());
                let ts_dim = !p.ts_on.value();
                let ts_knobs: [(&str, &FloatParam, bool); 8] = [
                    ("LOW X", &p.ts_low_x, false),
                    ("LOW ATK", &p.ts_low_atk, true),
                    ("LOW SUS", &p.ts_low_sus, true),
                    ("MID ATK", &p.ts_mid_atk, true),
                    ("MID SUS", &p.ts_mid_sus, true),
                    ("HIGH ATK", &p.ts_high_atk, true),
                    ("HIGH SUS", &p.ts_high_sus, true),
                    ("HIGH X", &p.ts_high_x, false),
                ];
                knob_row(ui, setter, skin, tr, knob_y(tr), 19.0, &ts_knobs, |_| ts_dim);

                // ---- IN / OUT meters
                let mp = Rect::from_min_max(pos2(scope_r.right() + 12.0, scope_r.top()), pos2(full.right() - 14.0, bottom));
                skin.panel(&painter, mp, "I/O", "");
                let my = mp.top() + Skin::TITLE_H + 26.0;
                let mh = mp.bottom() - my - 24.0;
                let m = &meters;
                skin.meter(&painter, mp.left() + 14.0, my, mh, "IN", m.db(0), m.db(1), Side::Right, "in");
                skin.meter(&painter, mp.right() - 28.0, my, mh, "OUT", m.db(2), m.db(3), Side::None, "out");

                // ---- footer
                skin.footer(&painter, foot, 0.0);
                skin.os_selector(ui, setter, pos2(foot.left() + 16.0, foot.center().y - 8.5), &p.os, 0.6);
                skin.safe_chip(ui, setter, Rect::from_center_size(pos2(foot.right() - 250.0, foot.center().y), vec2(112.0, 18.0)), &p.safe, meters.db(4));
                let look_ms = p.lookahead.value();
                let os_lat = oversample::latency(oversample::factor_for(p.os.value()));
                skin.small(
                    &painter,
                    pos2(foot.left() + 430.0, foot.center().y),
                    Align2::LEFT_CENTER,
                    format!("LOOKAHEAD {look_ms:.1} ms  +  OS {os_lat} smp"),
                    skin.text_faint(),
                );
            });
        }

#[cfg(test)]
mod shot_tests {
    use super::*;
    use crate::shot::*;
    use std::collections::HashMap;
    /// SHOT_DIR=/src/target/shots cargo test --lib screenshots  ->  kicknull0.rgb
    #[test]
    fn screenshots() {
        let Ok(dir) = std::env::var("SHOT_DIR") else { return };
        let plug = crate::KickNull::default();
        let ctx = egui::Context::default();
        install_fonts(&ctx);
        let mock = Mock;
        let setter = ParamSetter::new(&mock);
        let mut st = St { history: VecDeque::from(vec![ScopeFrame::default(); HISTORY]), presets: PresetBar::new("KickNullShot"), skin: Skin::new(Look::Machined, ACC, "KICKNULL", env!("CARGO_PKG_VERSION"), "") };
        let mut texs = HashMap::new();
        let mut out = None;
        for i in 0..4 {
            out = Some(ctx.run(raw(i, vec![]), |c| frame(c, &setter, &mut st, &plug.params, &plug.scope, &plug.bpm, &plug.meters)));
            if i < 3 { apply_delta(&mut texs, out.as_ref().unwrap()); }
        }
        render(&ctx, out.unwrap(), &mut texs, &format!("{dir}/kicknull0.rgb"));
    }
}
