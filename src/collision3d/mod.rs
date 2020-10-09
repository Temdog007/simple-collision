pub mod shapes;
pub use shapes::*;

use nalgebra::*;

pub fn from_f32(f: f32) -> Vector3<f32> {
    Vector3::new(f, f, f)
}
