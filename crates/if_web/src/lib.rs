//! imu-fuse web UI: plain web-sys DOM wiring, playback loop, i18n.
pub mod gl;
pub mod math;
mod plot;
mod view3d;

use if_core::{Params, Scenario, Sim, KINDS, PRESETS};
use std::cell::RefCell;
use std::rc::Rc;
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use web_sys::{Document, HtmlCanvasElement, HtmlElement};

const TR: &[(&str, &str, &str)] = &[
    ("subtitle", "6축 IMU 자세 융합 실험실", "6-axis IMU attitude-fusion lab"),
    ("sensor", "센서", "Sensor"),
    ("preset", "프리셋", "Preset"),
    ("p0", "소비자용 MEMS (일반적)", "consumer MEMS (typical)"),
    ("p1", "산업용 MEMS (일반적)", "industrial MEMS (typical)"),
    ("p2", "전술급 (일반적)", "tactical-grade (typical)"),
    ("fs", "샘플링 (Hz)", "Sample rate (Hz)"),
    ("dur", "길이 (s)", "Duration (s)"),
    ("gnd", "자이로 잡음 (deg/s/√Hz)", "Gyro noise (deg/s/√Hz)"),
    ("gb", "자이로 바이어스 x/y/z (deg/s)", "Gyro bias x/y/z (deg/s)"),
    ("grw", "자이로 바이어스 랜덤워크 ((deg/h)/√h)", "Gyro bias RW ((deg/h)/√h)"),
    ("and", "가속도 잡음 (mg/√Hz)", "Accel noise (mg/√Hz)"),
    ("ab", "가속도 바이어스 x/y/z (mg)", "Accel bias x/y/z (mg)"),
    ("motion", "움직임", "Motion"),
    ("scen", "시나리오", "Scenario"),
    ("s0", "정지", "static"),
    ("s1", "느린 흔들림", "slow sway"),
    ("s2", "걷는 휴머노이드 몸통", "walking humanoid torso"),
    ("s3", "빠른 회전", "fast spin"),
    ("srp", "정지 롤 / 피치 (deg)", "Static roll / pitch (deg)"),
    ("seed", "시드", "Seed"),
    ("newseed", "새 시드", "new seed"),
    ("filters", "필터", "Filters"),
    ("sel", "선택 필터", "Selected filter"),
    ("f0", "상보 필터", "complementary"),
    ("f1", "Mahony", "Mahony"),
    ("f2", "Madgwick", "Madgwick"),
    ("tune", "자동 튜닝", "auto-tune"),
    ("apply", "최적값 적용", "apply best"),
    ("implied", "입력값으로 본 지표", "Implied by inputs"),
    ("disclaimer", "교육용 추정치입니다 — 실제 하드웨어로 검증하세요.", "Educational estimates — validate on real hardware."),
    ("v0", "원근", "Perspective"),
    ("v1", "위", "Top"),
    ("v2", "옆", "Side"),
    ("v3", "몸통", "Body"),
    ("t0", "추정: 선택", "ghost: selected"),
    ("t0all", "추정: 전체", "ghost: all"),
    ("t1", "몸체 축", "body axes"),
    ("t2", "가속도 화살표", "accel arrow"),
    ("t3", "기울기 원뿔", "tilt cone"),
    ("nogl", "WebGL2를 사용할 수 없습니다 — 수치와 그래프는 계속 동작합니다.", "WebGL2 is not available — numeric readouts and plots still work."),
    ("play", "재생", "play"),
    ("pause", "일시정지", "pause"),
    ("hint", "드래그: 회전 · 오른쪽 드래그 / 두 손가락: 이동 · 휠 / 핀치: 확대 · 더블클릭: 초기화. \"정지\"에서는 몸통을 끌어 기울일 수 있습니다.", "Drag: orbit · right-drag / two fingers: pan · wheel / pinch: zoom · double-click: reset. In \"static\", drag the torso to tilt it."),
    ("errplot", "시간별 오차 (deg)", "Error vs time (deg)"),
    ("rmsbar", "RMS 기울기 오차 (deg)", "RMS tilt error (deg)"),
    ("sweepplot", "게인 스윕 (선택 필터)", "Gain sweep (selected filter)"),
    ("nosweep", "자동 튜닝을 실행하세요", "run auto-tune"),
    ("filter", "필터", "Filter"),
    ("rmsr", "RMS 롤", "RMS roll"),
    ("rmsp", "RMS 피치", "RMS pitch"),
    ("maxrp", "최대 롤/피치", "max roll/pitch"),
    ("rmst", "RMS 기울기", "RMS tilt"),
    ("yawd", "요 드리프트(끝)", "yaw drift (end)"),
    ("settle", "수렴 시간 (30°→2°)", "settling (30°→2°)"),
    ("never", "미수렴", "never"),
    ("bias", "Mahony 바이어스 추정 x/y/z (deg/s), 실제", "Mahony bias estimate x/y/z (deg/s), true"),
    ("biasz", "z축은 지자기 없이 관측 불가", "z is unobservable without a magnetometer"),
    ("arw", "각도 랜덤워크", "angle random walk"),
    ("drift", "상수 바이어스 드리프트", "constant-bias drift"),
    ("rwd", "실행 중 바이어스 변화 (1σ)", "in-run bias change (1σ)"),
    ("roll", "롤", "roll"),
    ("pitch", "피치", "pitch"),
    ("yaw", "요", "yaw"),
    ("true", "실제", "true"),
    ("tilt", "기울기 오차", "tilt error"),
];

fn doc() -> Document {
    web_sys::window().unwrap().document().unwrap()
}
fn el(id: &str) -> HtmlElement {
    doc().get_element_by_id(id).unwrap().dyn_into().unwrap()
}
fn canvas(id: &str) -> HtmlCanvasElement {
    el(id).dyn_into().unwrap()
}
fn get(id: &str) -> String {
    js_sys::Reflect::get(&el(id), &"value".into()).ok().and_then(|v| v.as_string()).unwrap_or_default()
}
fn set(id: &str, v: impl std::fmt::Display) {
    let _ = js_sys::Reflect::set(&el(id), &"value".into(), &v.to_string().into());
}
/// Parsed input value, or `old` if empty / invalid; clamped to [lo, hi].
fn num(id: &str, old: f64, lo: f64, hi: f64) -> f64 {
    get(id).trim().parse::<f64>().ok().filter(|v| v.is_finite()).unwrap_or(old).clamp(lo, hi)
}
fn on(target: &web_sys::EventTarget, ev: &str, f: impl FnMut(web_sys::Event) + 'static) {
    let c = Closure::<dyn FnMut(web_sys::Event)>::new(f);
    let _ = target.add_event_listener_with_callback(ev, c.as_ref().unchecked_ref());
    c.forget();
}

struct App {
    p: Params,
    sim: Sim,
    /// Plot series [panel roll/pitch/yaw][filter].
    series: [[Vec<f32>; 3]; 3],
    sel: usize,
    t: f64,
    playing: bool,
    speed: f64,
    ghost_all: bool,
    show: [bool; 3],
    view: Option<view3d::View>,
    sweeps: Option<Vec<Vec<(f64, f64)>>>,
    ko: bool,
    last_ms: f64,
    dirty: bool,
}

thread_local! {
    static APP: RefCell<Option<App>> = const { RefCell::new(None) };
}

fn with<R>(f: impl FnOnce(&mut App) -> R) -> R {
    APP.with(|a| f(a.borrow_mut().as_mut().unwrap()))
}

impl App {
    fn tr(&self, key: &str) -> &'static str {
        TR.iter().find(|t| t.0 == key).map(|t| if self.ko { t.1 } else { t.2 }).unwrap_or("?")
    }

    fn apply_lang(&self) {
        let d = doc();
        if let Some(h) = d.document_element() {
            let _ = h.set_attribute("lang", if self.ko { "ko" } else { "en" });
        }
        if let Ok(list) = d.query_selector_all("[data-i18n]") {
            for i in 0..list.length() {
                if let Some(e) = list.item(i).and_then(|n| n.dyn_into::<web_sys::Element>().ok()) {
                    let key = e.get_attribute("data-i18n").unwrap_or_default();
                    e.set_text_content(Some(self.tr(&key)));
                }
            }
        }
        el("lang").set_text_content(Some(if self.ko { "English" } else { "한국어" }));
        el("t0").set_text_content(Some(self.tr(if self.ghost_all { "t0all" } else { "t0" })));
        el("play").set_text_content(Some(self.tr(if self.playing { "pause" } else { "play" })));
    }

    fn write_sensor(&self) {
        let p = &self.p;
        set("gnd", p.gyro_nd);
        set("grw", p.gyro_rw);
        set("and", p.acc_nd);
        for i in 0..3 {
            set(&format!("gb{i}"), p.gyro_bias[i]);
            set(&format!("ab{i}"), p.acc_bias[i]);
        }
    }

    fn read_inputs(&mut self) {
        let p = &mut self.p;
        p.fs = num("fs", p.fs, 100.0, 1000.0);
        p.duration = num("dur", p.duration, 10.0, 120.0);
        p.gyro_nd = num("gnd", p.gyro_nd, 0.0, 10.0);
        p.gyro_rw = num("grw", p.gyro_rw, 0.0, 1e5);
        p.acc_nd = num("and", p.acc_nd, 0.0, 100.0);
        for i in 0..3 {
            p.gyro_bias[i] = num(&format!("gb{i}"), p.gyro_bias[i], -100.0, 100.0);
            p.acc_bias[i] = num(&format!("ab{i}"), p.acc_bias[i], -1000.0, 1000.0);
        }
        p.scenario = [Scenario::Static, Scenario::Sway, Scenario::Walking, Scenario::Spin][num("scen", 1.0, 0.0, 3.0) as usize];
        p.static_rp = [num("sr", p.static_rp[0], -80.0, 80.0), num("sp", p.static_rp[1], -80.0, 80.0)];
        let g = &mut p.gains;
        g.tau = num("tau", g.tau, 1e-3, 1e3);
        g.kp = num("kp", g.kp, 0.0, 1e3);
        g.ki = num("ki", g.ki, 0.0, 1e3);
        g.beta = num("beta", g.beta, 0.0, 100.0);
        self.sel = num("sel", 2.0, 0.0, 2.0) as usize;
    }

    fn recompute(&mut self, keep_sweep: bool) {
        self.sim = if_core::simulate(&self.p);
        self.series = std::array::from_fn(|a| std::array::from_fn(|f| self.sim.filters[f].err.iter().map(|e| e[a] as f32).collect()));
        if !keep_sweep {
            self.sweeps = None;
            el("apply").set_attribute("disabled", "").ok();
        }
        self.t = self.t.min(self.p.duration);
        let _ = el("time").set_attribute("max", &self.p.duration.to_string());
        self.tables();
        self.dirty = true;
    }

    fn tables(&self) {
        let f2 = |v: f64| format!("{v:.3}");
        let mut h = format!(
            "<tr><th>{}</th><th>{}</th><th>{}</th><th>{}</th><th>{}</th><th>{}</th><th>{}</th></tr>",
            self.tr("filter"), self.tr("rmsr"), self.tr("rmsp"), self.tr("maxrp"), self.tr("rmst"), self.tr("yawd"), self.tr("settle")
        );
        for (i, f) in self.sim.filters.iter().enumerate() {
            let m = &f.metrics;
            let settle = m.settle.map_or(self.tr("never").to_string(), |s| format!("{s:.2} s"));
            h += &format!(
                "<tr><td class=\"{}\">{}</td><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td>{:.2}</td><td>{settle}</td></tr>",
                ["k-comp", "k-mahony", "k-madgwick"][i], self.tr(&format!("f{i}")), f2(m.rms_roll), f2(m.rms_pitch), f2(m.max_rp), f2(m.rms_tilt), m.yaw_drift
            );
        }
        if let Some(b) = self.sim.filters[1].metrics.bias_est {
            let t = self.p.gyro_bias;
            h += &format!(
                "<tr><td colspan=\"7\">{}: {:.4} / {:.4} / {:.4} — {} {:.4} / {:.4} / {:.4} ({})</td></tr>",
                self.tr("bias"), b[0], b[1], b[2], self.tr("true"), t[0], t[1], t[2], self.tr("biasz")
            );
        }
        el("metrics").set_inner_html(&h);
        let r = if_core::readout(&self.p);
        el("readout").set_inner_html(&format!(
            "{}: {:.3} deg/√h<br>{}: {:.1} deg/h<br>{}: {:.2} deg/h",
            self.tr("arw"), r[0], self.tr("drift"), r[1], self.tr("rwd"), r[2]
        ));
        let names = [0, 1, 2].map(|i| self.tr(["f0", "f1", "f2"][i]));
        plot::bars(&canvas("bars"), names, std::array::from_fn(|i| self.sim.filters[i].metrics.rms_tilt));
        match &self.sweeps {
            Some(s) => plot::sweep(&canvas("sweep"), ["τ", "Kp", "β"][self.sel], &s[self.sel], plot::COLORS[self.sel]),
            None => plot::sweep(&canvas("sweep"), self.tr("nosweep"), &[], ""),
        }
    }

    fn frame(&mut self, now: f64) {
        let dt = ((now - self.last_ms) / 1000.0).clamp(0.0, 0.1);
        self.last_ms = now;
        let n = self.sim.truth.q.len();
        if self.playing {
            self.t += dt * self.speed;
            if self.t >= self.p.duration {
                self.t = 0.0;
            }
            set("time", format!("{:.3}", self.t));
            self.dirty = true;
        }
        let k = ((self.t * self.p.fs) as usize).min(n - 1);
        let tq = self.sim.truth.q[k];
        let sel = &self.sim.filters[self.sel];
        let order = [self.sel, (self.sel + 1) % 3, (self.sel + 2) % 3];
        let light = web_sys::window()
            .and_then(|w| w.match_media("(prefers-color-scheme: light)").ok().flatten())
            .is_some_and(|m| m.matches());
        let f = self.sim.imu.acc[k];
        let fw = tq.rotate(f);
        if let Some(v) = &mut self.view {
            let ghosts = order
                .iter()
                .take(if self.ghost_all { 3 } else { 1 })
                .map(|&i| {
                    let c = u32::from_str_radix(&plot::COLORS[i][1..], 16).unwrap_or(0);
                    let color = [16, 8, 0].map(|s| ((c >> s) & 255) as f32 / 255.0);
                    view3d::Ghost { q: self.sim.filters[i].est[k], color, tilt: self.sim.filters[i].tilt[k] }
                })
                .collect();
            let scene = view3d::Scene {
                truth: tq,
                ghosts,
                acc_down: fw.map(|x| -x / if_core::G),
                axes: self.show[0],
                acc: self.show[1],
                cone: self.show[2],
                light,
            };
            v.draw(&canvas("gl"), &scene, dt as f32);
        }
        let (e, s) = (tq.to_euler().map(f64::to_degrees), sel.est[k].to_euler().map(f64::to_degrees));
        el("hud").set_text_content(Some(&format!(
            "t = {:.2} s\n{} r/p/y  {:7.2} {:7.2} {:7.2}\n{} r/p/y  {:7.2} {:7.2} {:7.2}\n{}  {:.3}°\n|a| − g  {:+.3} m/s²",
            self.t,
            self.tr("true"),
            e[0], e[1], e[2],
            self.tr(&format!("f{}", self.sel)),
            s[0], s[1], s[2],
            self.tr("tilt"),
            sel.tilt[k],
            if_core::dot(f, f).sqrt() - if_core::G
        )));
        if self.dirty {
            self.dirty = false;
            let names = ["roll", "pitch", "yaw"].map(|k| self.tr(k));
            let s = &self.series;
            let panels = std::array::from_fn(|a| std::array::from_fn(|f| s[a][f].as_slice()));
            plot::series(&canvas("plot"), names, panels, self.p.duration, self.t);
        }
    }
}

#[wasm_bindgen(start)]
pub fn start() {
    let ko = web_sys::window().and_then(|w| w.navigator().language()).is_some_and(|l| l.starts_with("ko"));
    let p = Params::default();
    let view = view3d::View::new(&canvas("gl"));
    if view.is_none() {
        el("nogl").style().set_property("display", "block").ok();
        el("tools").style().set_property("display", "none").ok();
    }
    let app = App {
        sim: if_core::simulate(&Params { duration: 1.0, ..p }),
        p,
        series: Default::default(),
        sel: 2,
        t: 0.0,
        playing: true,
        speed: 0.25,
        ghost_all: false,
        show: [true; 3],
        view,
        sweeps: None,
        ko,
        last_ms: 0.0,
        dirty: true,
    };
    app.write_sensor();
    APP.with(|a| *a.borrow_mut() = Some(app));
    with(|a| {
        a.apply_lang();
        a.read_inputs();
        a.recompute(false);
    });
    wire();
    animate();
}

fn wire() {
    let ids = |s: &str| -> Vec<String> { s.split(' ').map(String::from).collect() };
    for id in ids("fs dur gnd gb0 gb1 gb2 grw and ab0 ab1 ab2 scen sr sp") {
        on(&el(&id), "change", |_| with(|a| { a.read_inputs(); a.recompute(false); }));
    }
    for id in ids("tau kp ki beta sel") {
        on(&el(&id), "change", |_| with(|a| { a.read_inputs(); a.recompute(true); }));
    }
    on(&el("preset"), "change", |_| {
        with(|a| {
            a.p.set_sensor(PRESETS[num("preset", 0.0, 0.0, 2.0) as usize]);
            a.write_sensor();
            a.recompute(false);
        })
    });
    on(&el("newseed"), "click", |_| {
        with(|a| {
            a.p.seed = (js_sys::Math::random() * 1e9) as u64 + 1;
            el("seed").set_text_content(Some(&a.p.seed.to_string()));
            a.recompute(false);
        })
    });
    on(&el("lang"), "click", |_| with(|a| { a.ko = !a.ko; a.apply_lang(); a.tables(); a.dirty = true; }));
    on(&el("tune"), "click", |_| {
        with(|a| {
            a.sweeps = Some(KINDS.iter().map(|&k| if_core::sweep(&a.p, k)).collect());
            el("apply").remove_attribute("disabled").ok();
            a.tables();
        })
    });
    on(&el("apply"), "click", |_| {
        with(|a| {
            let Some(s) = &a.sweeps else { return };
            for (i, &k) in KINDS.iter().enumerate() {
                let best = s[i].iter().min_by(|x, y| x.1.total_cmp(&y.1)).unwrap().0;
                if_core::set_main_gain(&mut a.p.gains, k, best);
            }
            let g = a.p.gains;
            (set("tau", format!("{:.4}", g.tau)), set("kp", format!("{:.4}", g.kp)), set("beta", format!("{:.4}", g.beta)));
            a.read_inputs();
            a.recompute(true);
        })
    });
    on(&el("play"), "click", |_| with(|a| { a.playing = !a.playing; a.apply_lang(); }));
    on(&el("speed"), "change", |_| with(|a| a.speed = num("speed", 0.25, 0.1, 1.0)));
    on(&el("time"), "input", |_| with(|a| { a.t = num("time", 0.0, 0.0, a.p.duration); a.dirty = true; }));
    for i in 0..4 {
        on(&el(&format!("v{i}")), "click", move |_| {
            with(|a| if let Some(v) = &mut a.view { v.set_preset(i) });
            (0..4).for_each(|j| { el(&format!("v{j}")).class_list().toggle_with_force("on", i == j).ok(); });
        });
    }
    on(&el("t0"), "click", |_| with(|a| { a.ghost_all = !a.ghost_all; a.apply_lang(); }));
    for i in 0..3 {
        on(&el(&format!("t{}", i + 1)), "click", move |_| {
            let on = with(|a| { a.show[i] = !a.show[i]; a.show[i] });
            el(&format!("t{}", i + 1)).class_list().toggle_with_force("on", on).ok();
        });
    }
    if let Some(w) = web_sys::window() {
        on(&w, "resize", |_| with(|a| { a.tables(); a.dirty = true; }));
    }
    wire_canvas();
}

fn wire_canvas() {
    let c = canvas("gl");
    let pe = |e: &web_sys::Event| e.clone().dyn_into::<web_sys::PointerEvent>().ok();
    on(&c, "pointerdown", move |e| {
        let Some(e) = pe(&e) else { return };
        e.prevent_default();
        let _ = canvas("gl").set_pointer_capture(e.pointer_id());
        with(|a| {
            let can = a.p.scenario == Scenario::Static;
            if let Some(v) = &mut a.view {
                v.down(e.pointer_id(), e.offset_x() as f32, e.offset_y() as f32, e.button(), can);
            }
        });
    });
    on(&c, "pointermove", move |e| {
        let Some(e) = pe(&e) else { return };
        let (x, y) = (e.offset_x() as f32, e.offset_y() as f32);
        let cursor = with(|a| {
            let can = a.p.scenario == Scenario::Static;
            if let Some((dr, dp)) = a.view.as_mut()?.moved(e.pointer_id(), x, y) {
                let rp = a.p.static_rp;
                a.p.static_rp = [(rp[0] + dr).clamp(-80.0, 80.0), (rp[1] + dp).clamp(-80.0, 80.0)];
                set("sr", format!("{:.1}", a.p.static_rp[0]));
                set("sp", format!("{:.1}", a.p.static_rp[1]));
                a.recompute(false);
            }
            let v = a.view.as_mut()?;
            Some(if v.dragging_tilt() { "grabbing" } else if v.hover_at(x, y, can) { "grab" } else { "default" })
        });
        if let Some(cur) = cursor {
            canvas("gl").style().set_property("cursor", cur).ok();
        }
    });
    for ev in ["pointerup", "pointercancel"] {
        on(&c, ev, move |e| {
            let Some(e) = pe(&e) else { return };
            with(|a| if let Some(v) = &mut a.view { v.up(e.pointer_id()) });
        });
    }
    on(&c, "wheel", |e| {
        e.prevent_default();
        let Ok(w) = e.dyn_into::<web_sys::WheelEvent>() else { return };
        with(|a| if let Some(v) = &mut a.view { v.wheel(w.delta_y()) });
    });
    on(&c, "dblclick", |_| with(|a| if let Some(v) = &mut a.view { v.set_preset(v.preset) }));
    on(&c, "contextmenu", |e| e.prevent_default());
}

fn animate() {
    let cb: Rc<RefCell<Option<Closure<dyn FnMut(f64)>>>> = Rc::new(RefCell::new(None));
    let again = cb.clone();
    *cb.borrow_mut() = Some(Closure::new(move |now: f64| {
        with(|a| a.frame(now));
        if let (Some(w), Some(f)) = (web_sys::window(), again.borrow().as_ref()) {
            let _ = w.request_animation_frame(f.as_ref().unchecked_ref());
        }
    }));
    if let (Some(w), Some(f)) = (web_sys::window(), cb.borrow().as_ref()) {
        let _ = w.request_animation_frame(f.as_ref().unchecked_ref());
    };
}
