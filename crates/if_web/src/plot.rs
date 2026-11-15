//! Small canvas-2D plots: stacked time series with a cursor, bar chart, log-x sweep curve.
use wasm_bindgen::JsCast;
use web_sys::{CanvasRenderingContext2d as Ctx, HtmlCanvasElement};

pub const COLORS: [&str; 3] = ["#e8a33d", "#3d9be8", "#5ac46a"];
const AXIS: &str = "#888";

/// Size the backing store to the CSS box × devicePixelRatio; returns (ctx, w, h, dpr).
fn prep(c: &HtmlCanvasElement) -> Option<(Ctx, f64, f64, f64)> {
    let dpr = web_sys::window()?.device_pixel_ratio();
    let (w, h) = (c.client_width() as f64 * dpr, c.client_height() as f64 * dpr);
    if c.width() != w as u32 || c.height() != h as u32 {
        c.set_width(w as u32);
        c.set_height(h as u32);
    }
    let ctx: Ctx = c.get_context("2d").ok()??.dyn_into().ok()?;
    ctx.clear_rect(0.0, 0.0, w, h);
    ctx.set_font(&format!("{}px sans-serif", (11.0 * dpr).round()));
    ctx.set_line_width(dpr);
    Some((ctx, w, h, dpr))
}

fn line(ctx: &Ctx, pts: impl Iterator<Item = (f64, f64)>) {
    ctx.begin_path();
    for (i, (x, y)) in pts.enumerate() {
        if i == 0 { ctx.move_to(x, y) } else { ctx.line_to(x, y) }
    }
    ctx.stroke();
}

/// `panels[p][f]` is the series of filter f in panel p over `dur` seconds; cursor at `t`.
pub fn series(c: &HtmlCanvasElement, names: [&str; 3], panels: [[&[f32]; 3]; 3], dur: f64, t: f64) {
    let Some((ctx, w, h, dpr)) = prep(c) else { return };
    let (l, ph) = (40.0 * dpr, h / 3.0);
    for (p, set) in panels.iter().enumerate() {
        let y0 = p as f64 * ph;
        let m = set.iter().flat_map(|s| s.iter()).fold(0.5f32, |a, v| a.max(v.abs())) as f64 * 1.1;
        let ys = |v: f32| y0 + ph / 2.0 - v as f64 / m * (ph / 2.0 - 4.0 * dpr);
        ctx.set_stroke_style_str(AXIS);
        ctx.set_fill_style_str(AXIS);
        line(&ctx, [(l, ys(0.0)), (w, ys(0.0))].into_iter());
        ctx.stroke_rect(l, y0 + 1.0, w - l - 1.0, ph - 2.0);
        let _ = ctx.fill_text(names[p], l + 4.0 * dpr, y0 + 13.0 * dpr);
        let _ = ctx.fill_text(&format!("±{m:.2}"), 2.0, y0 + 13.0 * dpr);
        for (f, s) in set.iter().enumerate() {
            ctx.set_stroke_style_str(COLORS[f]);
            let k = (s.len() / 900).max(1);
            let xs = |i: usize| l + (w - l) * i as f64 / s.len().max(1) as f64;
            line(&ctx, (0..s.len()).step_by(k).map(|i| (xs(i), ys(s[i]))));
        }
    }
    ctx.set_stroke_style_str("#e04040");
    let x = l + (w - l) * (t / dur).clamp(0.0, 1.0);
    line(&ctx, [(x, 0.0), (x, h)].into_iter());
}

pub fn bars(c: &HtmlCanvasElement, names: [&str; 3], v: [f64; 3]) {
    let Some((ctx, w, h, dpr)) = prep(c) else { return };
    let m = v.iter().fold(1e-9f64, |a, b| a.max(*b));
    let bw = w / 3.0;
    for i in 0..3 {
        let bh = (h - 30.0 * dpr) * v[i] / m;
        ctx.set_fill_style_str(COLORS[i]);
        ctx.fill_rect(i as f64 * bw + bw * 0.2, h - 16.0 * dpr - bh, bw * 0.6, bh);
        ctx.set_fill_style_str(AXIS);
        let _ = ctx.fill_text(&format!("{:.3}", v[i]), i as f64 * bw + bw * 0.2, h - 19.0 * dpr - bh);
        let _ = ctx.fill_text(names[i], i as f64 * bw + bw * 0.2, h - 3.0 * dpr);
    }
}

/// Log-x curve of (gain, rms); the best point is marked.
pub fn sweep(c: &HtmlCanvasElement, label: &str, pts: &[(f64, f64)], color: &str) {
    let Some((ctx, w, h, dpr)) = prep(c) else { return };
    ctx.set_fill_style_str(AXIS);
    if pts.len() < 2 {
        let _ = ctx.fill_text(label, 6.0 * dpr, h / 2.0);
        return;
    }
    let (x0, x1) = (pts[0].0.ln(), pts[pts.len() - 1].0.ln());
    let ymax = pts.iter().fold(1e-9f64, |a, p| a.max(p.1));
    let (l, b) = (6.0 * dpr, 16.0 * dpr);
    let xy = |p: &(f64, f64)| (l + (w - 2.0 * l) * (p.0.ln() - x0) / (x1 - x0), h - b - (h - b - 14.0 * dpr) * p.1 / ymax);
    ctx.set_stroke_style_str(color);
    line(&ctx, pts.iter().map(xy));
    let best = pts.iter().min_by(|a, b| a.1.total_cmp(&b.1)).unwrap();
    let (bx, by) = xy(best);
    ctx.fill_rect(bx - 3.0 * dpr, by - 3.0 * dpr, 6.0 * dpr, 6.0 * dpr);
    let _ = ctx.fill_text(&format!("{label} {:.3} → {:.3}°", best.0, best.1), l, 11.0 * dpr);
    let _ = ctx.fill_text(&format!("{:.3}", pts[0].0), l, h - 3.0 * dpr);
    let hi = format!("{:.2}", pts[pts.len() - 1].0);
    let tw = ctx.measure_text(&hi).map(|m| m.width()).unwrap_or(0.0);
    let _ = ctx.fill_text(&hi, w - l - tw, h - 3.0 * dpr);
}
