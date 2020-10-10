pub mod aabb;
pub use aabb::*;

pub mod sphere;
pub use sphere::*;

pub mod plane;
pub use plane::*;

pub mod triangle;
pub use triangle::*;

use nalgebra::*;

use super::*;

pub trait Shape3D {
    fn bounding_aabb(&self) -> AABB;

    fn bounding_sphere(&self) -> Sphere;

    fn center(&self) -> Vector3<f32>;

    fn translate(&self, point: &Vector3<f32>) -> Self;

    fn set_center(&self, point: &Vector3<f32>) -> Self;

    fn translate_mut(&mut self, point: &Vector3<f32>);

    fn set_center_mut(&mut self, point: &Vector3<f32>);
}
