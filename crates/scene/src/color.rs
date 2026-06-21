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

    /// Format as `#RRGGBB`. Alpha is dropped (UI only exposes RGB).
    pub fn to_hex(self) -> String {
        let clamp = |v: f32| (v.clamp(0.0, 1.0) * 255.0 + 0.5) as u8;
        format!("#{:02X}{:02X}{:02X}", clamp(self.r), clamp(self.g), clamp(self.b))
    }

    /// Parse `#RRGGBB`, `RRGGBB`, `#RGB`, or `RGB`. Returns `None` on any
    /// malformed input. Alpha is preserved at 1.0.
    pub fn from_hex(s: &str) -> Option<Self> {
        let s = s.trim().trim_start_matches('#');
        let (r, g, b) = match s.len() {
            6 => (
                u8::from_str_radix(&s[0..2], 16).ok()?,
                u8::from_str_radix(&s[2..4], 16).ok()?,
                u8::from_str_radix(&s[4..6], 16).ok()?,
            ),
            3 => {
                let hi = |c: char| u8::from_str_radix(&c.to_string(), 16).ok();
                let mut chars = s.chars();
                let r = hi(chars.next()?)?;
                let g = hi(chars.next()?)?;
                let b = hi(chars.next()?)?;
                (r * 17, g * 17, b * 17)
            }
            _ => return None,
        };
        Some(Color::rgb(r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0))
    }
}

impl Default for Color {
    fn default() -> Self {
        Color::WHITE
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hex_round_trip_six_digit() {
        let c = Color::from_hex("#FFAA00").unwrap();
        assert_eq!(c.to_hex(), "#FFAA00");
    }

    #[test]
    fn hex_accepts_three_digit_and_missing_hash() {
        let a = Color::from_hex("#F0A").unwrap();
        let b = Color::from_hex("ff00aa").unwrap();
        assert!((a.r - 1.0).abs() < 1e-6);
        assert!((b.b - 170.0 / 255.0).abs() < 1e-6);
    }

    #[test]
    fn costmap_free_and_unknown_are_transparent() {
        assert_eq!(Colormap::Costmap.sample(0).a, 0.0);
        assert_eq!(Colormap::Costmap.sample(255).a, 0.0);
        // Cost cells are visible.
        assert!(Colormap::Costmap.sample(50).a > 0.0);
        assert!(Colormap::Costmap.sample(100).a > 0.0);
    }

    #[test]
    fn hex_rejects_garbage() {
        assert!(Color::from_hex("not-a-color").is_none());
        assert!(Color::from_hex("#FFAA").is_none());
        assert!(Color::from_hex("#GGHHII").is_none());
    }
}

/// How per-cell `u8` values map to color in a Grid primitive.
#[derive(Clone, Debug, PartialEq)]
pub enum Colormap {
    /// White=free (0), black=occupied (100), grey=unknown (255 / 0xFF for -1).
    OccupancyDefault,
    /// Nav2/RViz cost gradient for `nav_msgs/OccupancyGrid` costmaps. Free
    /// space (0) and unknown (255) are transparent so the map underneath shows
    /// through; cost 1..98 ramps blue→red; 99 (inscribed) is cyan; 100
    /// (lethal) is magenta.
    Costmap,
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
            Colormap::Costmap => match v {
                // Free space and unknown are transparent so any map below shows through.
                0 | 255 => Color::rgba(0.0, 0.0, 0.0, 0.0),
                99 => Color::rgba(0.0, 1.0, 1.0, 0.85), // inscribed → cyan
                100 => Color::rgba(1.0, 0.0, 1.0, 0.85), // lethal → magenta
                // 1..=98: blue (low cost) → red (high cost).
                _ => {
                    let t = (v as f32) / 100.0;
                    Color::rgba(t, 0.0, 1.0 - t, 0.85)
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
