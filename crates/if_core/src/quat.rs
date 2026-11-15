//! Unit quaternion [w, x, y, z] for body → world rotation (world z up), plus small vec3 helpers.
pub type V3 = [f64; 3];

pub fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
pub fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn norm(a: V3) -> V3 {
    let n = dot(a, a).sqrt().max(1e-300);
    a.map(|x| x / n)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Q(pub [f64; 4]);

impl Q {
    pub const ID: Q = Q([1.0, 0.0, 0.0, 0.0]);

    pub fn mul(self, o: Q) -> Q {
        let ([a, b, c, d], [e, f, g, h]) = (self.0, o.0);
        Q([
            a * e - b * f - c * g - d * h,
            a * f + b * e + c * h - d * g,
            a * g - b * h + c * e + d * f,
            a * h + b * g - c * f + d * e,
        ])
    }
    pub fn conj(self) -> Q {
        let [w, x, y, z] = self.0;
        Q([w, -x, -y, -z])
    }
    pub fn normalized(self) -> Q {
        let n = self.0.iter().map(|x| x * x).sum::<f64>().sqrt();
        Q(self.0.map(|x| x / n))
    }
    /// Rotation by the rotation vector `v` (axis · angle, rad).
    pub fn exp(v: V3) -> Q {
        let th = dot(v, v).sqrt();
        let (s, c) = (th / 2.0).sin_cos();
        let k = if th < 1e-12 { 0.5 } else { s / th };
        Q([c, v[0] * k, v[1] * k, v[2] * k])
    }
    /// Rotation vector of the shortest equivalent rotation.
    pub fn log(self) -> V3 {
        let q = if self.0[0] < 0.0 { Q(self.0.map(|x| -x)) } else { self };
        let v = [q.0[1], q.0[2], q.0[3]];
        let s = dot(v, v).sqrt();
        let k = if s < 1e-12 { 2.0 } else { 2.0 * s.atan2(q.0[0]) / s };
        v.map(|x| x * k)
    }
    /// Exact step for constant body rate `w` (rad/s) over `dt`: q ⊗ exp(w·dt), renormalised.
    pub fn integrate(self, w: V3, dt: f64) -> Q {
        self.mul(Q::exp(w.map(|x| x * dt))).normalized()
    }
    /// Body vector → world.
    pub fn rotate(self, v: V3) -> V3 {
        let r = self.mul(Q([0.0, v[0], v[1], v[2]])).mul(self.conj());
        [r.0[1], r.0[2], r.0[3]]
    }
    /// World up (z) expressed in the body frame (= third row of R).
    pub fn up_in_body(self) -> V3 {
        let [w, x, y, z] = self.0;
        [2.0 * (x * z - w * y), 2.0 * (w * x + y * z), w * w - x * x - y * y + z * z]
    }
    /// From ZYX Euler [roll, pitch, yaw] (rad): R = Rz(yaw) Ry(pitch) Rx(roll).
    pub fn from_euler(e: V3) -> Q {
        let h = |a: f64| (a / 2.0).sin_cos();
        let ((sr, cr), (sp, cp), (sy, cy)) = (h(e[0]), h(e[1]), h(e[2]));
        Q([
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ])
    }
    pub fn to_euler(self) -> V3 {
        let [w, x, y, z] = self.0;
        [
            (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y)),
            (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin(),
            (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z)),
        ]
    }
}
