pub mod aabb;
pub use aabb::*;

pub mod sphere;
pub use sphere::*;

pub mod plane;
pub use plane::*;

pub mod triangle;
pub use triangle::*;

pub mod capsule;
pub use capsule::*;

pub mod ray;
pub use ray::*;

use super::*;

pub trait Shape3D<N : PhysicsScalar> {
    fn bounding_aabb(&self) -> AxisAlignedBoundingBox<N>;

    fn bounding_sphere(&self) -> Sphere<N>;

    fn center(&self) -> Vector3<N>;

    #[must_use = "Did you mean to use translate_mut()?"]
    fn translate(&self, point: &Vector3<N>) -> Self;

    #[must_use = "Did you mean to use set_center_mut()?"]
    fn set_center(&self, point: &Vector3<N>) -> Self;

    fn translate_mut(&mut self, point: &Vector3<N>);

    fn set_center_mut(&mut self, point: &Vector3<N>);
}
