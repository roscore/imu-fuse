use super::*;

const D: f64 = std::f64::consts::PI / 180.0;

fn quiet(scenario: Scenario) -> Params {
    Params {
        gyro_nd: 0.0,
        gyro_bias: [0.0; 3],
        gyro_rw: 0.0,
        acc_nd: 0.0,
        acc_bias: [0.0; 3],
        scenario,
        ..Params::default()
    }
}

#[test]
fn constant_yaw_rate_integrates_to_90_deg() {
    let mut q = Q::ID;
    let dt = 1e-3;
    for _ in 0..1000 {
        q = q.integrate([0.0, 0.0, 90.0 * D], dt);
    }
    assert!((q.to_euler()[2] - 90.0 * D).abs() < 1e-6 * D, "{:?}", q.to_euler());
}

#[test]
fn euler_round_trip() {
    for e in [[0.3, -0.4, 2.5], [-1.0, 1.2, -3.0], [0.0, 0.0, 0.0], [2.9, -0.1, 0.7]] {
        let r = Q::from_euler(e).to_euler();
        (0..3).for_each(|i| assert!((r[i] - e[i]).abs() < 1e-9, "{e:?} -> {r:?}"));
    }
    // Rotating body z (up) by a pure roll tilts it toward −y.
    let v = Q::from_euler([90.0 * D, 0.0, 0.0]).rotate([0.0, 0.0, 1.0]);
    assert!((v[1] + 1.0).abs() < 1e-12);
}

#[test]
fn static_level_no_noise_stays_level() {
    let s = simulate(&quiet(Scenario::Static));
    for f in &s.filters {
        assert!(f.metrics.max_rp < 1e-6, "{:?} {}", f.kind, f.metrics.max_rp);
        assert!(f.metrics.yaw_drift.abs() < 1e-6);
    }
}

#[test]
fn static_roll_20_converges_from_level() {
    let p = Params { static_rp: [20.0, 0.0], ..quiet(Scenario::Static) };
    let (truth, imu) = (truth(&p), synth(&p, &truth(&p)));
    for kind in KINDS {
        let (est, _) = run(kind, &p.gains, &imu, p.dt(), Q::ID);
        let e = est.last().unwrap().to_euler();
        assert!((e[0] / D - 20.0).abs() < 0.5, "{kind:?} roll {}", e[0] / D);
        assert!(truth.q.len() == est.len());
    }
}

#[test]
fn complementary_bias_error_is_b_tau() {
    let (b, tau) = (0.5, 2.0);
    let mut p = quiet(Scenario::Static);
    p.gyro_bias = [b, 0.0, 0.0];
    p.gains.tau = tau;
    let s = simulate(&p);
    let roll = s.filters[0].err.last().unwrap()[0];
    assert!((roll.abs() - b * tau).abs() < 0.2 * b * tau, "roll err {roll} deg");
}

#[test]
fn mahony_estimates_gyro_bias() {
    let mut p = quiet(Scenario::Static);
    p.duration = 60.0;
    p.gyro_bias = [0.8, -0.5, 0.0];
    let s = simulate(&p);
    let est = s.filters[1].metrics.bias_est.unwrap();
    for i in 0..2 {
        assert!((est[i] - p.gyro_bias[i]).abs() < 0.1 * p.gyro_bias[i].abs(), "{est:?}");
    }
}

#[test]
fn gaussian_mean_and_std() {
    let mut r = Rng::new(7);
    let n = 100_000;
    let sigma = 2.5;
    let xs: Vec<f64> = (0..n).map(|_| sigma * r.gauss()).collect();
    let mean = xs.iter().sum::<f64>() / n as f64;
    let std = (xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n as f64).sqrt();
    assert!(mean.abs() < 0.02 * sigma, "mean {mean}");
    assert!((std - sigma).abs() < 0.02 * sigma, "std {std}");
}

#[test]
fn same_seed_same_output() {
    let p = Params { scenario: Scenario::Walking, duration: 10.0, ..Params::default() };
    let (a, b) = (simulate(&p), simulate(&p));
    assert_eq!(a.imu.acc, b.imu.acc);
    assert_eq!(a.filters[2].metrics.rms_tilt, b.filters[2].metrics.rms_tilt);
    let c = simulate(&Params { seed: 99, ..p });
    assert_ne!(a.imu.gyro, c.imu.gyro);
}

#[test]
fn settling_and_sweep_are_sane() {
    let p = Params { duration: 20.0, ..Params::default() };
    let s = simulate(&p);
    for f in &s.filters {
        let t = f.metrics.settle.expect("settles");
        assert!(t > 0.0 && t < 20.0, "{:?} {t}", f.kind);
    }
    let sw = sweep(&p, Kind::Madgwick);
    assert_eq!(sw.len(), 20);
    assert!(sw.windows(2).all(|w| w[1].0 > w[0].0));
}
