use bytemuck::{Pod, Zeroable};

/// Linear RGBA color, 0..=1 per channel.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Pod, Zeroable)]
pub struct Color {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

impl Color {
    pub const WHITE: Color = Color::rgb(1.0, 1.0, 1.0);
    pub const BLACK: Color = Color::rgb(0.0, 0.0, 0.0);
    pub const RED: Color = Color::rgb(1.0, 0.0, 0.0);
    pub const GREEN: Color = Color::rgb(0.0, 1.0, 0.0);
    pub const BLUE: Color = Color::rgb(0.0, 0.0, 1.0);
    pub const GREY: Color = Color::rgb(0.5, 0.5, 0.5);

    pub const fn rgb(r: f32, g: f32, b: f32) -> Color {
        Color { r, g, b, a: 1.0 }
    }

    pub const fn rgba(r: f32, g: f32, b: f32, a: f32) -> Color {
        Color { r, g, b, a }
    }

    pub fn to_array(self) -> [f32; 4] {
        [self.r, self.g, self.b, self.a]
    }
}

impl Default for Color {
    fn default() -> Self {
        Color::WHITE
    }
}

/// How per-cell `u8` values map to color in a Grid primitive.
#[derive(Clone, Debug, PartialEq)]
pub enum Colormap {
    /// White=free (0), black=occupied (100), grey=unknown (255 / 0xFF for -1).
    OccupancyDefault,
    Grayscale,
    Inferno,
    /// 256-entry LUT.
    Custom(Vec<Color>),
}

impl Colormap {
    /// Sample a single byte through this colormap on the CPU.
    /// (Used for tests and fallbacks; the GPU path samples in WGSL.)
    pub fn sample(&self, v: u8) -> Color {
        match self {
            Colormap::OccupancyDefault => match v {
                0 => Color::WHITE,
                100 => Color::BLACK,
                255 => Color::rgba(0.5, 0.5, 0.5, 0.6),
                _ => {
                    let t = (v as f32) / 100.0;
                    let g = 1.0 - t;
                    Color::rgb(g, g, g)
                }
            },
            Colormap::Grayscale => {
                let g = (v as f32) / 255.0;
                Color::rgb(g, g, g)
            }
            Colormap::Inferno => {
                // Cheap two-stop ramp; replaced with a real LUT later.
                let t = (v as f32) / 255.0;
                Color::rgb(t, t * 0.3, (1.0 - t) * 0.6)
            }
            Colormap::Custom(lut) if !lut.is_empty() => lut[(v as usize).min(lut.len() - 1)],
            Colormap::Custom(_) => Color::WHITE,
        }
    }
}
