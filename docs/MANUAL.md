# KICKNULL manual

## How it works

```
out = 808 − cancel(t) × band(808)
cancel(t) = AMOUNT × curve(envelope(kick × SENSITIVITY), TENSION)
```

`band()` is the whole 808 (FREQ fully right, "Full") or only the part below FREQ.

## Setup (FL Studio)

1. Put KICKNULL on the 808's mixer track.
2. Route the kick track to the 808's track as a **sidechain** (right-click the send knob and choose "Sidechain to this track").
3. Start from a preset, and use THE DIP to see what one kick does to the 808.

## SIDECHAIN row

| Control | What it does |
|---|---|
| **MODE** | PHASE, VOLUME, COMPRESS, RINGMOD or SHAPE. The knobs on the row change with the mode. |
| **SENSITIVITY** | Gain on the kick before it drives the duck. |
| **ATTACK / DECAY** | How fast the duck rises when the kick hits and falls away afterwards. |
| **TENSION** | Shape of the kick-to-cancellation curve. 50 % is linear; higher cancels harder at lower kick levels. |
| **AMOUNT** | Maximum cancellation. 100 % is a full null. |
| **FREQ** | Only cancel below this frequency. Fully right means the whole 808. |
| **CURVE / FOLLOW** | CURVE fires a fixed dip shape on each kick (LENGTH as a note value, SHAPE for how fast it comes back); FOLLOW tracks the kick's envelope. |
| **CLICK** | Key only on the kick's click (300 Hz to 3 kHz). |
| **GATE** | THRESH, RANGE (0 dB = off) and RELEASE: an extra dip triggered by the kick's click. |
| **VOLUME** | How far the 808 is turned down at full duck. |
| **COMPRESS** | THRESH, RATIO, ATTACK, RELEASE, KNEE. |
| **SHAPE** | SMOOTH, PUNCH, LINEAR, EXPO, SINE, PUMP, GATE or DOUBLE. BEAT SYNC loops it on the host beat instead of firing per kick. |

## KICK COPY row

Adds the sidechain kick to the output through a lookahead brickwall limiter: THRESH, RANGE, LEVEL, LOW, HIGH, DRIVE and LENGTH. **KICK THRU** passes the dry kick through as well, so the shaper works on the whole kick. Unroute the kick's own track when it's on.

## TRANSIENT row

A three-band transient shaper on the kick output, with low/high crossovers and ATTACK / SUSTAIN per band.

## Displays

- **LIVE:** the 808 over the last ~2.5 s in dB. Red is what the kick takes away, grey is what's left, and the thin line is the kick.
- **THE DIP:** what one kick hit does to the 808 with the current settings.

## Footer

**LOOKAHEAD** delays the 808 so the cut can start just before the kick lands (FL compensates). **OVERSAMPLING** covers the RINGMOD product and KICK DRIVE. The total latency is shown.
