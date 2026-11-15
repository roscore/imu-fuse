//! IMU attitude-fusion core: synthetic 6-axis IMU on a ground-truth trajectory, three textbook
//! attitude filters (complementary, Mahony, Madgwick), metrics and a coarse gain sweep.
//! World frame z up; quaternions rotate body → world. Gravity g = [0, 0, −G].
mod quat;
pub use quat::*;

pub const G: f64 = 9.80665;
const D: f64 = std::f64::consts::PI / 180.0;

/// Small seeded PRNG (splitmix64) with Box-Muller Gaussian.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed)
    }
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    /// Uniform in (0, 1].
    fn unit(&mut self) -> f64 {
        ((self.next_u64() >> 11) + 1) as f64 / (1u64 << 53) as f64
    }
    /// Standard normal. ponytail: discards the second Box-Muller value; fine at these sample counts.
    pub fn gauss(&mut self) -> f64 {
        (-2.0 * self.unit().ln()).sqrt() * (std::f64::consts::TAU * self.unit()).cos()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Scenario {
    Static,
    Sway,
    Walking,
    Spin,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind {
    Comp,
    Mahony,
    Madgwick,
}
pub const KINDS: [Kind; 3] = [Kind::Comp, Kind::Mahony, Kind::Madgwick];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gains {
    /// Complementary time constant (s).
    pub tau: f64,
    pub kp: f64,
    pub ki: f64,
    pub beta: f64,
}

/// Sensor numbers in datasheet-style units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sensor {
    /// Gyro noise density, deg/s/√Hz.
    pub gyro_nd: f64,
    /// Constant gyro bias per axis, deg/s.
    pub gyro_bias: [f64; 3],
    /// Gyro bias random walk, (deg/h)/√h.
    pub gyro_rw: f64,
    /// Accelerometer noise density, mg/√Hz.
    pub acc_nd: f64,
    /// Accelerometer bias per axis, mg.
    pub acc_bias: [f64; 3],
}

/// Generic "typical" grades (round illustrative numbers, not from any datasheet).
pub const PRESETS: [Sensor; 3] = [
    Sensor { gyro_nd: 0.01, gyro_bias: [0.5, -0.3, 0.4], gyro_rw: 30.0, acc_nd: 0.15, acc_bias: [20.0, -15.0, 25.0] },
    Sensor { gyro_nd: 0.003, gyro_bias: [0.05, -0.04, 0.03], gyro_rw: 3.0, acc_nd: 0.05, acc_bias: [3.0, -2.0, 4.0] },
    Sensor { gyro_nd: 0.0005, gyro_bias: [0.002, -0.0015, 0.001], gyro_rw: 0.1, acc_nd: 0.02, acc_bias: [0.5, -0.3, 0.4] },
];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params {
    pub fs: f64,
    pub duration: f64,
    pub gyro_nd: f64,
    pub gyro_bias: [f64; 3],
    pub gyro_rw: f64,
    pub acc_nd: f64,
    pub acc_bias: [f64; 3],
    pub scenario: Scenario,
    /// Static scenario roll / pitch (deg).
    pub static_rp: [f64; 2],
    pub seed: u64,
    pub gains: Gains,
}

impl Default for Params {
    fn default() -> Self {
        let mut p = Params {
            fs: 200.0,
            duration: 30.0,
            gyro_nd: 0.0,
            gyro_bias: [0.0; 3],
            gyro_rw: 0.0,
            acc_nd: 0.0,
            acc_bias: [0.0; 3],
            scenario: Scenario::Sway,
            static_rp: [0.0; 2],
            seed: 1,
            gains: Gains { tau: 1.0, kp: 1.0, ki: 0.1, beta: 0.05 },
        };
        p.set_sensor(PRESETS[0]);
        p
    }
}

impl Params {
    pub fn dt(&self) -> f64 {
        1.0 / self.fs
    }
    pub fn n(&self) -> usize {
        (self.duration * self.fs).round().max(2.0) as usize
    }
    pub fn set_sensor(&mut self, s: Sensor) {
        (self.gyro_nd, self.gyro_bias, self.gyro_rw, self.acc_nd, self.acc_bias) =
            (s.gyro_nd, s.gyro_bias, s.gyro_rw, s.acc_nd, s.acc_bias);
    }
    /// Euler [roll, pitch, yaw] (rad) and world linear acceleration (m/s²) at time t.
    fn motion(&self, t: f64) -> (V3, V3) {
        let s = |a: f64, f: f64, ph: f64| a * D * (std::f64::consts::TAU * f * t + ph).sin();
        match self.scenario {
            Scenario::Static => ([self.static_rp[0] * D, self.static_rp[1] * D, 0.0], [0.0; 3]),
            Scenario::Sway => ([s(10.0, 0.2, 0.0), s(8.0, 0.13, 1.0), s(20.0, 0.05, 0.0)], [0.0; 3]),
            Scenario::Walking => {
                // Step frequency f: lateral sway at f/2, vertical bounce at f, a short spike per foot strike.
                let (f, bounce) = (1.8, 0.025);
                let w = std::f64::consts::TAU * f;
                let tk = ((t * f - 0.1).round() + 0.1) / f;
                let spike = (-0.5 * ((t - tk) / 0.012).powi(2)).exp();
                let e = [s(3.0, f / 2.0, 0.0), 4.0 * D + s(2.0, f, 0.0), s(5.0, f / 2.0, 0.5)];
                (e, [0.3 * G * spike, 0.0, -bounce * w * w * (w * t).sin() + G * spike])
            }
            Scenario::Spin => ([s(5.0, 0.5, 0.0), s(3.0, 0.3, 0.0), 360.0 * D * t], [0.0; 3]),
        }
    }
}

pub struct Truth {
    pub q: Vec<Q>,
    /// Body angular velocity over each sample interval (rad/s).
    pub w: Vec<V3>,
    /// World linear acceleration (m/s²).
    pub lin: Vec<V3>,
}

pub fn truth(p: &Params) -> Truth {
    let n = p.n();
    let (mut q, mut lin) = (Vec::with_capacity(n + 1), Vec::with_capacity(n + 1));
    for k in 0..=n {
        let (e, a) = p.motion(k as f64 * p.dt());
        q.push(Q::from_euler(e));
        lin.push(a);
    }
    // Rate consistent with exact exp-map integration: q_{k+1} = q_k ⊗ exp(w_k dt).
    let w = (0..n).map(|k| q[k].conj().mul(q[k + 1]).log().map(|x| x * p.fs)).collect();
    q.truncate(n);
    lin.truncate(n);
    Truth { q, w, lin }
}

pub struct Imu {
    /// gyro[k]: rate over k → k+1 (rad/s). acc[k]: specific force at k (m/s²).
    pub gyro: Vec<V3>,
    pub acc: Vec<V3>,
    /// True gyro bias over time (rad/s).
    pub bias: Vec<V3>,
}

/// gyro = ω + bias(t) + white noise; accel = Rᵀ(a − g) + bias + white noise.
pub fn synth(p: &Params, t: &Truth) -> Imu {
    let mut r = Rng::new(p.seed);
    let sq = p.fs.sqrt();
    let (sg, sa) = (p.gyro_nd * D * sq, p.acc_nd * 1e-3 * G * sq);
    let srw = p.gyro_rw * D / 3600.0 / 60.0 * p.dt().sqrt();
    let mut b = p.gyro_bias.map(|x| x * D);
    let ab = p.acc_bias.map(|x| x * 1e-3 * G);
    let n = t.q.len();
    let mut imu = Imu { gyro: Vec::with_capacity(n), acc: Vec::with_capacity(n), bias: Vec::with_capacity(n) };
    for k in 0..n {
        let f = t.q[k].conj().rotate([t.lin[k][0], t.lin[k][1], t.lin[k][2] + G]);
        imu.acc.push(std::array::from_fn(|i| f[i] + ab[i] + sa * r.gauss()));
        imu.gyro.push(std::array::from_fn(|i| t.w[k][i] + b[i] + sg * r.gauss()));
        imu.bias.push(b);
        b = b.map(|x| x + srw * r.gauss());
    }
    imu
}

/// One filter step: propagate with `gyro` over dt, correct with the new `acc`.
/// Returns the new estimate; `int` is Mahony's integral term (−bias estimate).
fn step(kind: Kind, g: &Gains, q: Q, gyro: V3, acc: V3, dt: f64, int: &mut V3) -> Q {
    let a = norm(acc);
    match kind {
        Kind::Comp => {
            // Rotate (in the world frame) the measured up direction a fraction dt/(τ+dt) toward z.
            let q1 = q.integrate(gyro, dt);
            let up = q1.rotate(a);
            let ax = cross(up, [0.0, 0.0, 1.0]);
            let s = dot(ax, ax).sqrt();
            if s < 1e-12 {
                return q1;
            }
            let ang = dot(up, [0.0, 0.0, 1.0]).clamp(-1.0, 1.0).acos() * dt / (g.tau + dt);
            Q::exp(ax.map(|x| x / s * ang)).mul(q1).normalized()
        }
        Kind::Mahony => {
            let e = cross(a, q.up_in_body());
            *int = std::array::from_fn(|i| int[i] + g.ki * e[i] * dt);
            q.integrate(std::array::from_fn(|i| gyro[i] + g.kp * e[i] + int[i]), dt)
        }
        Kind::Madgwick => {
            let [q0, q1, q2, q3] = q.0;
            let f = [2.0 * (q1 * q3 - q0 * q2) - a[0], 2.0 * (q0 * q1 + q2 * q3) - a[1], 2.0 * (0.5 - q1 * q1 - q2 * q2) - a[2]];
            let j = [[-2.0 * q2, 2.0 * q3, -2.0 * q0, 2.0 * q1], [2.0 * q1, 2.0 * q0, 2.0 * q3, 2.0 * q2], [0.0, -4.0 * q1, -4.0 * q2, 0.0]];
            let grad: [f64; 4] = std::array::from_fn(|c| (0..3).map(|r| j[r][c] * f[r]).sum());
            let gn = grad.iter().map(|x| x * x).sum::<f64>().sqrt();
            // q̇ = ½ q⊗ω − β ĝ  ⇔  body rate ω − 2β vec(q* ⊗ ĝ); the radial part is removed by renormalising.
            let c = if gn > 1e-12 { q.conj().mul(Q(grad.map(|x| x / gn))) } else { Q([0.0; 4]) };
            q.integrate(std::array::from_fn(|i| gyro[i] - 2.0 * g.beta * c.0[i + 1]), dt)
        }
    }
}

/// Run one filter over all samples from `init`. Returns estimates and Mahony's bias estimate (rad/s).
pub fn run(kind: Kind, g: &Gains, imu: &Imu, dt: f64, init: Q) -> (Vec<Q>, V3) {
    let mut int = [0.0; 3];
    let mut est = Vec::with_capacity(imu.acc.len());
    est.push(init);
    for k in 1..imu.acc.len() {
        let q = step(kind, g, est[k - 1], imu.gyro[k - 1], imu.acc[k], dt, &mut int);
        est.push(q);
    }
    (est, int.map(|x| -x))
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Metrics {
    pub rms_roll: f64,
    pub rms_pitch: f64,
    pub rms_tilt: f64,
    /// Max of |roll error| and |pitch error| (deg).
    pub max_rp: f64,
    /// Yaw error at the end (deg).
    pub yaw_drift: f64,
    /// Time until the tilt error stays below 2° after starting 30° off (s); None if never.
    pub settle: Option<f64>,
    /// Mahony gyro bias estimate at the end (deg/s).
    pub bias_est: Option<V3>,
}

pub struct FilterRun {
    pub kind: Kind,
    pub est: Vec<Q>,
    /// Estimate − truth [roll, pitch, yaw] (deg, wrapped).
    pub err: Vec<V3>,
    /// Angle between true and estimated up (deg).
    pub tilt: Vec<f64>,
    pub metrics: Metrics,
}

pub struct Sim {
    pub truth: Truth,
    pub imu: Imu,
    pub filters: Vec<FilterRun>,
}

pub fn tilt_deg(a: Q, b: Q) -> f64 {
    dot(a.up_in_body(), b.up_in_body()).clamp(-1.0, 1.0).acos() / D
}

fn wrap_deg(x: f64) -> f64 {
    (x / D + 180.0).rem_euclid(360.0) - 180.0
}

fn rms(xs: impl Iterator<Item = f64>, n: usize) -> f64 {
    (xs.map(|x| x * x).sum::<f64>() / n as f64).sqrt()
}

pub fn simulate(p: &Params) -> Sim {
    let truth = truth(p);
    let imu = synth(p, &truth);
    let filters = KINDS.iter().map(|&k| evaluate(p, &truth, &imu, k, true)).collect();
    Sim { truth, imu, filters }
}

fn evaluate(p: &Params, t: &Truth, imu: &Imu, kind: Kind, settle: bool) -> FilterRun {
    let (est, bias) = run(kind, &p.gains, imu, p.dt(), t.q[0]);
    let n = est.len();
    let err: Vec<V3> = (0..n)
        .map(|k| {
            let (e, r) = (est[k].to_euler(), t.q[k].to_euler());
            std::array::from_fn(|i| wrap_deg(e[i] - r[i]))
        })
        .collect();
    let tilt: Vec<f64> = (0..n).map(|k| tilt_deg(est[k], t.q[k])).collect();
    let settle = settle.then(|| {
        let init = t.q[0].mul(Q::exp([30.0 * D, 0.0, 0.0]));
        let (e2, _) = run(kind, &p.gains, imu, p.dt(), init);
        let last = (0..n).rev().find(|&k| tilt_deg(e2[k], t.q[k]) >= 2.0).unwrap_or(0);
        (last + 1 < n).then(|| (last + 1) as f64 * p.dt())
    });
    let metrics = Metrics {
        rms_roll: rms(err.iter().map(|e| e[0]), n),
        rms_pitch: rms(err.iter().map(|e| e[1]), n),
        rms_tilt: rms(tilt.iter().copied(), n),
        max_rp: err.iter().map(|e| e[0].abs().max(e[1].abs())).fold(0.0, f64::max),
        yaw_drift: err[n - 1][2],
        settle: settle.flatten(),
        bias_est: (kind == Kind::Mahony).then(|| bias.map(|x| x / D)),
    };
    FilterRun { kind, est, err, tilt, metrics }
}

/// The filter's main gain (τ, Kp or β).
pub fn main_gain(g: &Gains, kind: Kind) -> f64 {
    match kind {
        Kind::Comp => g.tau,
        Kind::Mahony => g.kp,
        Kind::Madgwick => g.beta,
    }
}

pub fn set_main_gain(g: &mut Gains, kind: Kind, v: f64) {
    match kind {
        Kind::Comp => g.tau = v,
        Kind::Mahony => g.kp = v,
        Kind::Madgwick => g.beta = v,
    }
}

/// 20 log-spaced values of the main gain → RMS tilt error (deg), same samples for all.
pub fn sweep(p: &Params, kind: Kind) -> Vec<(f64, f64)> {
    let t = truth(p);
    let imu = synth(p, &t);
    let (lo, hi): (f64, f64) = match kind {
        Kind::Comp => (0.05, 20.0),
        Kind::Mahony => (0.05, 20.0),
        Kind::Madgwick => (0.002, 1.0),
    };
    (0..20)
        .map(|i| {
            let v = lo * (hi / lo).powf(i as f64 / 19.0);
            let mut q = *p;
            set_main_gain(&mut q.gains, kind, v);
            (v, evaluate(&q, &t, &imu, kind, false).metrics.rms_tilt)
        })
        .collect()
}

/// Simple readout implied by the inputs: [angle random walk (deg/√h), constant-bias drift (deg/h),
/// bias random-walk spread after the run (deg/h)].
pub fn readout(p: &Params) -> [f64; 3] {
    let b = p.gyro_bias;
    [p.gyro_nd * 60.0, dot(b, b).sqrt() * 3600.0, p.gyro_rw * (p.duration / 3600.0).sqrt()]
}

#[cfg(test)]
mod tests;
