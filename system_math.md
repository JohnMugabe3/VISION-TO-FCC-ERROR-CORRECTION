# MINI UMWAMBI — How the system math works

**In one line:** the camera sees a target as a dot in the video; the system
works out how many degrees left/right and up/down that dot is **from the
nose of the UAV**, and sends those two numbers to the flight controller 10
times a second.

Everything below is one idea per part, plain words first, then the formula.
The example numbers are the real ones from the dashboard screenshot.

---

## Part 1 — From pixels to degrees

**The plain idea:** the tracker tells us the target is "30 pixels right of
the middle of the screen." That's not useful to a flight controller. We need
to turn "30 pixels" into "so many degrees to the right." A camera lens does
this with a fixed rule, because we know how wide its view is.

Think of the camera's view as a fan that is 65° wide, spread evenly across
the width of the video. If you know how many pixels wide the fan is, you can
work out how many degrees any single pixel is worth.

**The formula:**

```
focal_px = (frame_width / 2) / tan(65° / 2)      # a fixed "pixels per angle" number for this lens
angle    = atan(pixel_offset / focal_px)
```

**The example:** a 1280-pixel-wide frame gives `focal_px ≈ 1004`. A target
30 px right → `atan(30 / 1004)` ≈ **+1.12° yaw**. The 10 px down → **−0.36°
pitch**. These are the two "OFF BORESIGHT" numbers on the dashboard.

**Why not just "degrees per pixel"?** Because the edges of the lens squash
things slightly. The `tan`/`atan` step corrects for that, so the angle is
right whether the target is in the middle or the corner of the frame.

**The catch:** this angle is measured from **where the camera is pointing**,
not from the nose of the aircraft. The gimbal can be aimed anywhere, so we're
not done — that's Part 2.

---

## Part 2 — Making the gimbal and the flight controller agree

**The plain idea:** the gimbal and the flight controller both report angles,
but they measure them from **different starting points**. If you just
compared their numbers you'd get nonsense. This part translates them into the
same language so they can be combined.

Here's the mismatch that causes all the confusion:

| angle | Flight controller says…       | Gimbal (SIYI, follow mode) says…      |
|-------|-------------------------------|----------------------------------------|
| yaw   | **compass heading** (0 = North) | **left/right of the nose**            |
| pitch | up/down from level (horizon)  | up/down from level (its own IMU)       |
| roll  | tilt from level               | tilt from level (its own IMU)          |

That is why the screen can show flight-controller yaw **+109°** next to
gimbal yaw **−28°** and both are correct: one means "the aircraft is facing
roughly east-southeast," the other means "the camera is aimed 28° to the left
of the nose." They answer different questions — you cannot subtract them
directly.

**How we reconcile them (3 steps):**

1. Take the aircraft's full 3D orientation from the flight controller
   (its yaw *is* the compass heading).
2. Work out the camera's orientation **in that same compass world**. The
   gimbal already holds pitch/roll level to the earth, and the camera's real
   heading is `flight_controller_yaw + gimbal_yaw` (109 − 28).
3. "Subtract" the aircraft's orientation from the camera's. The compass
   heading is in both, so **it cancels out**, and what's left is exactly
   *"where is the camera pointing compared to the nose?"*

**The result** ("CAMERA VS NOSE" on the screen): pitch **+22.7°**, yaw
**−28.6°**, roll **−0.5°**.

**"Why does roll show −0.5° when the gimbal says roll is 0?"** Because the
gimbal keeps the camera level to the **ground**, but the aircraft itself is
tilted +0.7°. Compared to the tilted aircraft, the level camera looks tilted
the other way. It's real, not a glitch — and it's why the numbers don't add
up to a perfectly clean sum (they're 3D rotations, which mix together
slightly, not plain numbers you can add).

---

## The obvious objection — "They're mounted together and start out aligned. Why don't the numbers match?"

**The plain idea:** bolting the gimbal to the airframe fixes only the gimbal's
**base**. The camera sits on three motorized axes above that base and moves on
its own — that is the entire purpose of a gimbal. The moment it starts
tracking a target, the camera and the airframe point in different directions,
so their attitude numbers **are supposed to disagree**. They would only match
with the gimbal centered.

And even when they physically point the same way, they still *report*
differently, because each device answers a different question:

- the FCC answers *"how am I oriented relative to North and the horizon?"*
- the gimbal answers *"how is my camera aimed relative to the aircraft's
  nose (yaw) and the horizon (pitch/roll)?"*

So comparing FCC yaw +109° with gimbal yaw −28° is comparing a compass
heading with a "degrees left of the nose" — meaningless by construction.
The only row on the dashboard where the numbers are in a *common* frame is
**CAMERA VS NOSE**, which is what Part 2 computes.

**What makes the translation possible at all:** both devices carry an IMU
and both measure against the **same earth** — gravity gives them a shared
horizon, and yaw composition shares the heading. The earth is the common
reference frame; the math in Part 2 passes both attitudes through it and the
heading cancels out.

**"How does the camera correct the FCC's angles?"** It doesn't — and that is
the key design point. The system never touches or adjusts the FCC's attitude.
It translates the target's position into the FCC's **own body frame** *before*
sending, so what arrives ("target is 27.4° left of your nose, 22.3° above it")
is already in the only frame the FCC ever thinks in. There is no frame
conflict on the wire, because the reconciliation happened on the companion
computer.

**Proof the code executes this** (numeric checks run against
`fcc_landing_feedback.py`, reproducible any time):

| check | input | result |
|---|---|---|
| Gimbal centered, vehicle level (the "start" scenario) | gimbal 0/0/0, any heading | camera-vs-nose **0.00 / 0.00 / 0.00** ✓ |
| Vehicle tilted (pitch +5°, roll +2°, heading 200°), camera aimed along the nose | gimbal reports 5/0/2 | camera-vs-nose **0.00 / 0.00 / 0.00** ✓ |
| Same geometry at two different compass headings (109° vs 291°) | identical gimbal values | identical camera-vs-nose — **heading cancels** ✓ |
| The dashboard screenshot values | gimbal 23.3/−28.4/0, FCC 0.34/109.45/0.66 | **+22.69 / −28.58 / −0.46** — matches the screen ✓ |

**The honest limitations** (worth volunteering before someone asks):

1. The math assumes the gimbal base is mounted square to the airframe. A
   mounting twist shows up 1-to-1 as a bias in the correction — checked by
   centering the gimbal and confirming CAMERA VS NOSE reads ~0/0/0 on the
   bench. That is the boresight check, and the dashboard makes it visible.
2. The gimbal's IMU and the FCC's IMU each estimate "level" independently; if
   they disagree by half a degree, that half degree leaks into the
   correction. Small for this class of hardware, but not zero.
3. Compass error does **not** matter — the heading cancels in the math, so a
   bad compass calibration cannot corrupt the correction.

---

## Part 3 — Combining into one final steering command

**The plain idea:** now we have two pieces — "target vs camera" (Part 1) and
"camera vs nose" (Part 2). Chain them together and you get the answer we
actually want: **"target vs nose."**

We take the direction to the target from Part 1, rotate it by the
camera-vs-nose orientation from Part 2, and read off the final two angles:

```
UAV NOSE → CAMERA      pitch +22.7   yaw −28.6     (Part 2)
+ CAMERA → TARGET      pitch  −0.4   yaw  +1.1     (Part 1)
= UAV NOSE → TARGET    pitch +22.3   yaw −27.4     (Part 3 — this is what's sent)
```

**One clever detail:** the pitch is measured against the nose's *flat
horizontal plane*, not straight along the nose. This means the math never
breaks even if the target ends up 90° off to the side — which is exactly what
lets the **same code work for any dive angle, from a shallow 10° to a
near-vertical 90°.**

---

## What actually gets sent

A MAVLink `DEBUG_VECT` message named `VIS_ERR`, 10 times a second:

- **x = yaw error** → positive means "steer right"
- **y = pitch error** → positive means "steer up"

Both in degrees off the nose. This is **information only** — the system does
not fly the aircraft. The flight controller (or a script on it) reads these
numbers and decides what, if anything, to do with them.
