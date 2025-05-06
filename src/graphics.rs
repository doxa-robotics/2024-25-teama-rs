use embedded_graphics::{pixelcolor::Rgb888, prelude::RgbColor};
use tinybmp::Bmp;
use vexide::{
    devices::{display::Rect, math::Point2},
    prelude::Display,
};

const WIDTH: usize = 480;
const HEIGHT: usize = 240;

pub fn render_banner(mut display: Display) {
    display.erase((0, 0, 0));
    let bmp = Bmp::<Rgb888>::from_slice(include_bytes!("../assets/brain-image.bmp")).unwrap();
    display.draw_buffer(
        Rect {
            start: Point2 { x: 0, y: 0 },
            end: Point2 {
                x: WIDTH as i16,
                y: HEIGHT as i16,
            },
        },
        bmp.pixels().map(|p| (p.1.r(), p.1.g(), p.1.b())),
        WIDTH as i32,
    );
}
