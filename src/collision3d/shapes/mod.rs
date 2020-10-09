pub mod aabb;
pub use aabb::*;

pub mod sphere;
pub use sphere::*;

use nalgebra::*;

use super::*;

pub trait Shape3D {
    fn bounding_aabb(&self) -> AABB;

    fn bounding_sphere(&self) -> Sphere;

    fn center(&self) -> Vector3<f32>;

    fn translate(&mut self, point: &Vector3<f32>);

    fn set_center(&mut self, point: &Vector3<f32>);
}
