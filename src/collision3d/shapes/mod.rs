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

pub trait Shape3D<N : PhysicsScalar> {
    fn bounding_aabb(&self) -> AABB<N>;

    fn bounding_sphere(&self) -> Sphere<N>;

    fn center(&self) -> Vector3<N>;

    fn translate(&self, point: &Vector3<N>) -> Self;

    fn set_center(&self, point: &Vector3<N>) -> Self;

    fn translate_mut(&mut self, point: &Vector3<N>);

    fn set_center_mut(&mut self, point: &Vector3<N>);
}
