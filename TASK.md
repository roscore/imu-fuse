# Task: IMU attitude-fusion lab (Rust → WebAssembly, static web app)

Original open-source tool. Write all code yourself; do not copy anyone's code, tables or assets.
Use textbook methods only (quaternion kinematics, complementary filter, Mahony PI filter,
Madgwick gradient-descent filter). Sensor presets are generic, clearly labelled "typical"
(no vendor names, no copied datasheet tables).

## Product (SMALL scope — core + UI about 800-1200 lines of new Rust; renderer/math are reused)
A single-page web app (GitHub Pages, sub-path `/imu-fuse/`): "My robot's 6-axis IMU has this much gyro
noise/bias and accelerometer noise, and the body moves like this — which attitude filter and which gains
give the smallest roll/pitch error, and how fast does yaw drift?"

1. **Inputs**: sample rate (100 / 200 / 500 / 1000 Hz, default 200), duration (10-120 s, default 30),
   gyro noise density (deg/s/√Hz), gyro constant bias per axis (deg/s), gyro bias random walk,
   accelerometer noise density (mg/√Hz), accel bias (mg). Presets (editable):
   "consumer MEMS (typical)", "industrial MEMS (typical)", "tactical-grade (typical)".
   Motion scenario: "static", "slow sway" (sinusoidal roll/pitch/yaw), "walking humanoid torso"
   (periodic sway + vertical bounce + foot-strike linear-acceleration spikes), "fast spin".
   Seeded RNG (fixed seed, reproducible; a "new seed" button).
2. **Model (core)**: ground-truth attitude trajectory as unit quaternion + body angular velocity +
   body linear acceleration; synthetic IMU (gyro = ω + bias(t) + white noise; accel = Rᵀ(a − g) + bias
   + noise; Box-Muller Gaussian from a small seeded PRNG). Three filters run on the same samples:
   (a) complementary filter (gyro integration blended with accel tilt, time constant τ),
   (b) Mahony (Kp, Ki with gyro bias estimate), (c) Madgwick (β).
   All filters integrate quaternions exactly (exp map) and renormalise.
   Metrics per filter: RMS and max roll/pitch error (deg), yaw drift at end (deg), settling time
   from an initial 30° tilt error (time until tilt error < 2°), estimated-vs-true gyro bias (Mahony).
3. **Extras (small)**: (a) "auto-tune": a coarse 1-D sweep of each filter's main gain (τ, Kp, β; ~20
   values, log-spaced) minimising RMS tilt error, with a "apply best" button; (b) Allan-deviation-free
   simple readout: angle random walk and in-run drift implied by the inputs (deg/√h, deg/h).
4. **3D view (WebGL2, game-grade — see below)**: a robot-torso-like box (lit, with a front marker)
   in the true attitude, plus translucent ghost boxes for the selected filter's estimate (and the other
   filters optionally), each with RGB body axes; a gravity arrow and the measured accel vector arrow
   (shows linear-acceleration disturbance during foot strikes); a "tilt cone" showing the current tilt
   error. Box sits above a checker ground with shadow.
5. **2D plots** (canvas 2D, own small module): roll / pitch / yaw error vs time for the three filters
   (colour per filter) with the current playback time as a vertical cursor; small bar chart of RMS tilt
   error per filter; gain-sweep curve after auto-tune.
6. Korean + English toggle (browser default). Mobile friendly. CSS prefers-color-scheme only.
Out of scope (do NOT build): magnetometer / 9-axis, Kalman/EKF, real sensor import, Web Sensor API,
URL share, JSON/CSV import/export.

## 3D view requirements (game-like, reuse the existing renderer)
- `crates/if_web/src/gl.rs` and `crates/if_web/src/math.rs` already exist: a tool-independent WebGL2
  renderer (lit meshes, checker/grid ground with fog, sky gradient, stencil planar shadows, translucent
  volumes, lines) and pure math (mat4, damped orbit camera with inertia, ray-plane/ray-sphere picking)
  with unit tests. Reuse them; extend only if needed and keep them tool-independent and tested.
- Scene code in `view3d.rs`. Orbit (left drag) / pan (right drag, two fingers) / zoom (wheel, pinch)
  with inertia, double-click reset, `touch-action:none`. View presets: Perspective / Top / Side /
  Body (camera rides behind the true torso).
- Direct manipulation: in the "static" scenario, dragging the torso box tilts it (sets the static roll
  and pitch inputs) and the simulation re-runs live; hover highlight + cursor change.
- Toggle buttons over the view: ghost estimates (selected / all), body axes, accel arrow, tilt cone.
- Playback: play/pause, speed 0.1× / 0.25× / 1× (default 0.25×), time slider; the plot cursor follows.
  Ghost colour shifts toward red when tilt error > 5°. HUD over the canvas: time, true roll/pitch/yaw,
  selected filter estimate, tilt error, |accel| − g.
- If WebGL2 is unavailable, show a message and keep the numeric readouts and 2D plots working.
- Release wasm ≲ 350 KB. No crates besides wasm-bindgen, web-sys, js-sys.

## Core crate (pure Rust, no web deps, unit tested) — `crates/if_core`
- Quaternion type (mul, conj, exp-map integration, to/from Euler ZYX, rotate vector), PRNG + Gaussian,
  trajectory generators, IMU synthesiser, three filters, metrics, gain sweep.
- Tests (hand-checkable, 6-10): integrating a constant 90°/s about z for 1 s gives yaw 90° within 1e-6;
  Euler→quat→Euler round trip; static level with zero noise/bias → all filters stay at 0 tilt;
  static 20° roll, zero noise → each filter converges from 0 to 20° ± 0.5°;
  complementary filter with bias b and τ: steady tilt error ≈ b·τ (within 20 %);
  Mahony with Ki > 0 estimates a constant gyro bias within 10 % after 60 s static;
  noise generator: sample mean ≈ 0 and std ≈ σ (n=100k, 2 %); same seed → identical output.
- `crates/if_web`: UI, plain web-sys. The existing math.rs tests must keep passing.

## Repo hygiene
- README (Korean first, then English): what it is, model and assumptions (synthetic signals, no
  magnetometer, white noise + bias random walk only, no temperature effects), controls, local build
  (`cargo test`, `trunk build --release`), MIT license, "educational estimates — validate on real hardware".
- LICENSE: MIT, copyright "imu-fuse contributors". `.gitignore` for target/ dist/.
- `index.html` with a CSP meta that blocks external requests:
  `default-src 'self'; script-src 'self' 'wasm-unsafe-eval'; style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; connect-src 'self'; object-src 'none'; base-uri 'self'`.
  In `Trunk.toml` set `filehash = false`, `public_url = "/imu-fuse/"` and
  `pattern_script = "<script type=\"module\" src=\"{base}boot.js\"></script>"` with an external
  `web/boot.js` (copied via `<link data-trunk rel="copy-file">`) that imports `./if_web.js` and calls init.
- `.github/workflows/pages.yml`: on push to main + workflow_dispatch; checkout, rust stable + wasm32
  target, install trunk 0.22.1 binary release, `sh tools/privacy_check.sh`, `cargo test --release`,
  `trunk build --release`, configure-pages@v5, upload-pages-artifact@v3 (dist), deploy job with
  actions/deploy-pages@v4 (permissions pages: write, id-token: write).
- `tools/privacy_check.sh` already exists; keep it and keep it clean.

## Done when
- `cargo test` green (workspace); `trunk build --release` succeeds with no warnings;
  `sh tools/privacy_check.sh` clean.
- Commit author `imu-fuse contributors <59640857+roscore@users.noreply.github.com>`.
- Commit everything with clear commits; open a PR. Do NOT create new GitHub repos or change repo settings.
