//! 3D scene: true torso box, ghost estimates, body axes, gravity / measured-accel arrows,
//! tilt cone, orbit camera with presets, picking for the static-tilt drag.
use crate::gl::{self, Geo, Mesh, Paint, Renderer};
use crate::math::{self, OrbitCam, Orbit, M4, V};
use if_core::{Q, V3};
use web_sys::HtmlCanvasElement;

const CENTER: V = [0.0, 1.0, 0.0];
const SIZE: V = [0.2, 0.35, 0.5];

/// Core world (x fwd, y left, z up) → GL (y up, right-handed).
pub fn to_gl(v: V3) -> V {
    [v[0] as f32, v[2] as f32, -v[1] as f32]
}

fn axes_of(q: Q) -> [V; 3] {
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]].map(|e| to_gl(q.rotate(e)))
}

fn perp(d: V) -> (V, V) {
    let a = math::norm(math::cross(d, if d[1].abs() < 0.9 { [0.0, 1.0, 0.0] } else { [1.0, 0.0, 0.0] }));
    (a, math::cross(d, a))
}

/// Cone with apex at the origin opening along +y to radius 1 at y = 1.
fn cone(n: usize) -> Geo {
    let mut g = Geo::default();
    for i in 0..=n {
        let (s, c) = (i as f32 / n as f32 * std::f32::consts::TAU).sin_cos();
        g.pos.extend([0.0, 0.0, 0.0, c, 1.0, s]);
        let nr = math::norm([c, -1.0, s]);
        g.nrm.extend(nr);
        g.nrm.extend(nr);
    }
    for i in 0..n as u16 {
        g.idx.extend([2 * i, 2 * i + 1, 2 * i + 3]);
    }
    g
}

fn preset(i: usize) -> Orbit {
    let (yaw, pitch, dist) = [(0.7, 0.3, 2.6), (0.0, 1.55, 3.0), (std::f32::consts::FRAC_PI_2, 0.02, 2.6), (0.0, 0.25, 1.8)][i];
    Orbit { target: CENTER, yaw, pitch, dist }
}

#[derive(PartialEq)]
enum Drag {
    None,
    Orbit,
    Pan,
    Tilt,
    Pinch(f32),
}

pub struct Ghost {
    pub q: Q,
    pub color: V,
    pub tilt: f64,
}

pub struct Scene {
    pub truth: Q,
    pub ghosts: Vec<Ghost>,
    /// Measured specific force rotated to the world by the true attitude, negated, in g.
    pub acc_down: V3,
    pub axes: bool,
    pub acc: bool,
    pub cone: bool,
    pub light: bool,
}

pub struct View {
    r: Renderer,
    cube: Mesh,
    cyl: Mesh,
    cone: Mesh,
    axes: Mesh,
    pub cam: OrbitCam,
    pub preset: usize,
    pub hover: bool,
    inv_vp: M4,
    css: [f32; 2],
    ptrs: Vec<(i32, f32, f32)>,
    drag: Drag,
}

impl View {
    pub fn new(c: &HtmlCanvasElement) -> Option<View> {
        let r = Renderer::new(c)?;
        let mut ax = gl::lines(&[[0.0; 3], [1.0, 0.0, 0.0], [0.0; 3], [0.0, 1.0, 0.0], [0.0; 3], [0.0, 0.0, 1.0]]);
        ax.col = vec![1., 0.2, 0.2, 1., 0.2, 0.2, 0.2, 1., 0.2, 0.2, 1., 0.2, 0.3, 0.5, 1., 0.3, 0.5, 1.];
        Some(View {
            cube: r.mesh(&gl::cube())?,
            cyl: r.mesh(&gl::cylinder(12))?,
            cone: r.mesh(&cone(24))?,
            axes: r.mesh(&ax)?,
            r,
            cam: OrbitCam::new(preset(0)),
            preset: 0,
            hover: false,
            inv_vp: math::IDENTITY,
            css: [1.0; 2],
            ptrs: vec![],
            drag: Drag::None,
        })
    }

    pub fn set_preset(&mut self, i: usize) {
        self.preset = i;
        self.cam.set_goal(preset(i));
    }

    fn arrow(&self, dir: V, len: f32, color: [f32; 4]) {
        if len < 1e-4 {
            return;
        }
        let d = math::norm(dir);
        let (a, b) = perp(d);
        let shaft = math::basis(a, d, b, [0.02, len * 0.8, 0.02], math::add(CENTER, math::scale(d, len * 0.4)));
        self.r.draw(&self.cyl, &shaft, Paint::lit(color));
        let head = math::basis(a, math::scale(d, -1.0), b, [0.05, len * 0.2, 0.05], math::add(CENTER, math::scale(d, len)));
        self.r.draw(&self.cone, &head, Paint::lit(color));
    }

    pub fn draw(&mut self, c: &HtmlCanvasElement, s: &Scene, dt: f32) {
        let dpr = web_sys::window().map(|w| w.device_pixel_ratio()).unwrap_or(1.0);
        let (cw, ch) = (c.client_width().max(1), c.client_height().max(1));
        let (w, h) = ((cw as f64 * dpr) as u32, (ch as f64 * dpr) as u32);
        if c.width() != w || c.height() != h {
            c.set_width(w);
            c.set_height(h);
        }
        self.css = [cw as f32, ch as f32];
        let [bx, by, bz] = axes_of(s.truth);
        if self.preset == 3 {
            // Ride behind the torso: keep the goal yaw on the same turn as the current one.
            let want = (-bx[0]).atan2(-bx[2]);
            let tau = std::f32::consts::TAU;
            self.cam.goal.yaw = want + ((self.cam.cur.yaw - want) / tau).round() * tau;
        }
        self.cam.step(dt);
        let eye = self.cam.cur.eye();
        let view = math::look_at(eye, self.cam.cur.target, [0.0, 1.0, 0.0]);
        let proj = math::perspective(0.8, cw as f32 / ch as f32, 0.05, 100.0);
        self.inv_vp = math::invert(&math::mul(&proj, &view)).unwrap_or(math::IDENTITY);
        self.r.theme = if s.light { gl::LIGHT } else { gl::DARK };
        self.r.begin(&view, &proj, eye, [0, 0, w as i32, h as i32]);

        let body = math::basis(bx, by, bz, SIZE, CENTER);
        let mark_at = math::add(CENTER, math::add(math::scale(bx, 0.105), math::scale(bz, 0.12)));
        let mark = math::basis(bx, by, bz, [0.02, 0.2, 0.1], mark_at);
        self.r.shadow(&self.cube, &body);
        self.r.shadow(&self.cube, &mark);
        let glow = if self.hover { 0.25 } else { 0.0 };
        self.r.draw(&self.cube, &body, Paint { glow, ..Paint::lit([0.72, 0.75, 0.82, 1.0]) });
        self.r.draw(&self.cube, &mark, Paint::lit([0.95, 0.35, 0.15, 1.0]));
        if s.axes {
            self.r.draw(&self.axes, &math::basis(bx, by, bz, [0.45; 3], CENTER), Paint::flat([1.0; 4]));
        }
        self.arrow([0.0, -1.0, 0.0], 0.7, [0.85, 0.85, 0.9, 1.0]);
        if s.acc {
            let d = to_gl(s.acc_down);
            self.arrow(d, 0.7 * math::dot(d, d).sqrt(), [0.95, 0.8, 0.2, 1.0]);
        }
        // Translucent last.
        for g in &s.ghosts {
            let [gx, gy, gz] = axes_of(g.q);
            if s.axes {
                self.r.draw(&self.axes, &math::basis(gx, gy, gz, [0.4; 3], CENTER), Paint::flat([1.0, 1.0, 1.0, 0.6]));
            }
            let red = ((g.tilt as f32 - 5.0) / 5.0).clamp(0.0, 1.0) * 0.8;
            let col = std::array::from_fn::<f32, 3, _>(|i| g.color[i] * (1.0 - red) + [1.0, 0.1, 0.1][i] * red);
            self.r.draw(&self.cube, &math::basis(gx, gy, gz, math::scale(SIZE, 1.02), CENTER), Paint::lit([col[0], col[1], col[2], 0.3]));
        }
        if let (true, Some(g)) = (s.cone, s.ghosts.first()) {
            let r = (g.tilt.to_radians() as f32).tan().min(3.0) * 0.6;
            let (a, b) = perp(bz);
            let m = math::basis(a, bz, b, [r, 0.6, r], CENTER);
            self.r.draw(&self.cone, &m, Paint::lit([1.0, 0.3, 0.3, 0.25]));
        }
    }

    fn hits_box(&self, x: f32, y: f32) -> bool {
        let (o, d) = math::ndc_ray(&self.inv_vp, 2.0 * x / self.css[0] - 1.0, 1.0 - 2.0 * y / self.css[1]);
        math::ray_sphere(o, d, CENTER, 0.3).is_some()
    }

    /// Hover test; true if the box is under the pointer and tilt-dragging is allowed.
    pub fn hover_at(&mut self, x: f32, y: f32, can_tilt: bool) -> bool {
        self.hover = can_tilt && self.drag == Drag::None && self.hits_box(x, y);
        self.hover
    }

    pub fn down(&mut self, id: i32, x: f32, y: f32, button: i16, can_tilt: bool) {
        self.ptrs.retain(|p| p.0 != id);
        self.ptrs.push((id, x, y));
        self.cam.held = true;
        self.drag = match self.ptrs.len() {
            1 if button == 2 => Drag::Pan,
            1 if can_tilt && self.hits_box(x, y) => Drag::Tilt,
            1 => Drag::Orbit,
            _ => Drag::Pinch(self.spread()),
        };
    }

    fn spread(&self) -> f32 {
        let (a, b) = (self.ptrs[0], self.ptrs[1]);
        ((a.1 - b.1).powi(2) + (a.2 - b.2).powi(2)).sqrt().max(1.0)
    }

    /// Returns a tilt delta (deg roll, deg pitch) while dragging the torso.
    pub fn moved(&mut self, id: i32, x: f32, y: f32) -> Option<(f64, f64)> {
        let i = self.ptrs.iter().position(|p| p.0 == id)?;
        let (dx, dy) = (x - self.ptrs[i].1, y - self.ptrs[i].2);
        self.ptrs[i] = (id, x, y);
        match self.drag {
            Drag::Orbit => self.cam.rotate(-dx * 0.008, dy * 0.008),
            Drag::Tilt => return Some((dx as f64 * 0.3, dy as f64 * 0.3)),
            Drag::Pan | Drag::Pinch(_) => {
                let (n, k) = (self.ptrs.len() as f32, self.cam.cur.dist * 0.0015);
                let o = self.cam.cur;
                let f = math::norm(math::sub(o.target, o.eye()));
                let right = math::norm(math::cross(f, [0.0, 1.0, 0.0]));
                let up = math::cross(right, f);
                self.cam.pan(math::add(math::scale(right, -dx * k / n), math::scale(up, dy * k / n)));
                if let Drag::Pinch(s0) = self.drag {
                    let s = self.spread();
                    self.cam.zoom(s0 / s);
                    self.drag = Drag::Pinch(s);
                }
            }
            Drag::None => {}
        }
        None
    }

    pub fn up(&mut self, id: i32) {
        self.ptrs.retain(|p| p.0 != id);
        self.drag = if self.ptrs.len() == 1 { Drag::Orbit } else { Drag::None };
        self.cam.held = !self.ptrs.is_empty();
    }

    pub fn wheel(&mut self, dy: f64) {
        self.cam.zoom((dy * 0.001).exp() as f32);
    }

    pub fn dragging_tilt(&self) -> bool {
        self.drag == Drag::Tilt
    }
}
