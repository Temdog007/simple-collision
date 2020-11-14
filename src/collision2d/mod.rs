use super::*;

pub mod shapes;
pub use shapes::*;

pub trait Shape2D<N : PhysicsScalar> {
    fn bounding_aabb(&self) -> AxisAlignedBoundingBox<N>;

    fn bounding_sphere(&self) -> Circle<N>;

    fn center(&self) -> Vector2<N>;

    #[must_use = "Did you mean to use translate_mut()?"]
    fn translate(&self, point: &Vector2<N>) -> Self;

    #[must_use = "Did you mean to use set_center_mut()?"]
    fn set_center(&self, point: &Vector2<N>) -> Self;

    fn translate_mut(&mut self, point: &Vector2<N>);

    fn set_center_mut(&mut self, point: &Vector2<N>);
}
