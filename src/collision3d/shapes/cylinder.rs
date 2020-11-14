use super::*;
use nalgebra::*;

use crate::collision2d::shapes::{Circle, AABB2D};

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct Cylinder<N: PhysicsScalar> {
    pub center: Vector3<N>,
    pub half_height: N,
    pub radius: N,
    pub axis: Axis,
}

impl<N: FloatingPhysicsScalar> Cylinder<N> {
    pub fn from_aabb(aabb: &AABB3D<N>, axis: Axis) -> Self {
        let (half_height, radius) = match axis {
            Axis::X => (
                aabb.half_width(),
                RealField::max(aabb.half_height(), aabb.half_depth()),
            ),
            Axis::Y => (
                aabb.half_height(),
                RealField::max(aabb.half_width(), aabb.half_depth()),
            ),
            Axis::Z => (
                aabb.half_depth(),
                RealField::max(aabb.half_width(), aabb.half_height()),
            ),
        };
        Self {
            center: aabb.center(),
            half_height,
            radius,
            axis,
        }
    }
    pub fn height(&self) -> N {
        self.half_height * N::from_f64(2.0).unwrap()
    }
    pub fn diameter(&self) -> N {
        self.radius * N::from_f64(2.0).unwrap()
    }
    pub fn half_height(&self, axis: Axis) -> N {
        if axis == self.axis {
            self.half_height
        } else {
            self.radius
        }
    }
    pub fn get_position(&self, axis: Axis) -> N {
        match axis {
            Axis::X => self.center.x,
            Axis::Y => self.center.y,
            Axis::Z => self.center.z,
        }
    }
    fn get_shape(&self, horizontal: Axis, vertical: Axis) -> CylinderShape<N> {
        let center = Vector2::new(self.get_position(horizontal), self.get_position(vertical));
        if self.axis == horizontal || self.axis == vertical {
            let x = self.half_height(horizontal);
            let y = self.half_height(vertical);
            let v = Vector2::new(x, y);
            CylinderShape::AABB(AABB2D {
                start: center - v,
                end: center + v,
            })
        } else {
            CylinderShape::Circle(Circle {
                center,
                radius: self.radius,
            })
        }
    }
    fn get_collision(
        collisions: &[Option<CollisionResolution<Vector2<N>, N>>],
    ) -> Option<CollisionResolution<Vector3<N>, N>> {
        if collisions.iter().any(|c| c.is_none()) {
            None
        } else {
            let (index, res) = collisions
                .iter()
                .filter_map(|c| c.as_ref())
                .enumerate()
                .min_by(
                    |(_, a), (_, b)| match a.penetration.partial_cmp(&b.penetration) {
                        Some(o) => o,
                        None => Ordering::Equal,
                    },
                )
                .unwrap();
            let rval = match index {
                0 => CollisionResolution {
                    normal: Vector3::new(res.normal.x, res.normal.y, N::zero()),
                    penetration: res.penetration,
                },
                1 => CollisionResolution {
                    normal: Vector3::new(N::zero(), res.normal.y, res.normal.x),
                    penetration: res.penetration,
                },
                2 => CollisionResolution {
                    normal: Vector3::new(res.normal.x, N::zero(), res.normal.y),
                    penetration: res.penetration,
                },
                v => panic!("Unexpected index {}", v),
            };
            Some(rval)
        }
    }
    pub fn get_sphere_collision(
        &self,
        sphere: &Sphere<N>,
    ) -> Option<CollisionResolution<Vector3<N>, N>> {
        let axes = [(Axis::X, Axis::Y), (Axis::Z, Axis::Y), (Axis::X, Axis::Z)];

        let mut collisions = [None; 3];

        for (index, (horizontal, vertical)) in axes.iter().enumerate() {
            unsafe {
                *collisions.get_unchecked_mut(index) = self
                    .get_shape(*horizontal, *vertical)
                    .get_collision(&CylinderShape::Circle(
                        sphere.to_circle(*horizontal, *vertical),
                    ));
            }
        }

        Self::get_collision(&collisions)
    }
    pub fn get_aabb_collision(
        &self,
        aabb: &AABB3D<N>,
    ) -> Option<CollisionResolution<Vector3<N>, N>> {
        let axes = [(Axis::X, Axis::Y), (Axis::Z, Axis::Y), (Axis::X, Axis::Z)];

        let mut collisions = [None; 3];

        for (index, (horizontal, vertical)) in axes.iter().enumerate() {
            unsafe {
                *collisions.get_unchecked_mut(index) = self
                    .get_shape(*horizontal, *vertical)
                    .get_collision(&CylinderShape::AABB(aabb.to_2d(*horizontal, *vertical)));
            }
        }

        Self::get_collision(&collisions)
    }
}

impl<N: FloatingPhysicsScalar> Shape3D<N> for Cylinder<N> {
    fn bounding_aabb(&self) -> AxisAlignedBoundingBox<N> {
        let v = Vector3::new(
            self.half_height(Axis::X),
            self.half_height(Axis::Y),
            self.half_height(Axis::Z),
        );
        let start = self.center - v;
        let end = self.center + v;
        AxisAlignedBoundingBox { start, end }
    }
    fn bounding_sphere(&self) -> Sphere<N> {
        Sphere {
            center: self.center,
            radius: [self.half_height, self.radius]
                .iter()
                .max_by(|a, b| match a.partial_cmp(b) {
                    Some(o) => o,
                    None => Ordering::Equal,
                })
                .cloned()
                .unwrap(),
        }
    }
    fn center(&self) -> Vector3<N> {
        self.center
    }
    fn translate(&self, point: &Vector3<N>) -> Self {
        Self {
            center: self.center + point,
            half_height: self.half_height,
            radius: self.radius,
            axis: self.axis,
        }
    }
    fn translate_mut(&mut self, point: &Vector3<N>) {
        self.center += point;
    }
    fn set_center(&self, point: &Vector3<N>) -> Self {
        let mut c = *self;
        c.set_center_mut(point);
        c
    }
    fn set_center_mut(&mut self, point: &Vector3<N>) {
        self.center = *point;
    }
}

enum CylinderShape<N: PhysicsScalar> {
    AABB(AABB2D<N>),
    Circle(Circle<N>),
}

impl<N: FloatingPhysicsScalar> CylinderShape<N> {
    fn get_collision(&self, other: &Self) -> Option<CollisionResolution<Vector2<N>, N>> {
        match self {
            CylinderShape::AABB(a) => match other {
                CylinderShape::AABB(b) => a.get_aabb_collision(b),
                CylinderShape::Circle(b) => a.get_circle_collision(b),
            },
            CylinderShape::Circle(a) => match other {
                CylinderShape::AABB(b) => b.get_circle_collision(a).map(|r| r.flip()),
                CylinderShape::Circle(b) => a.get_circle_collision(b),
            },
        }
    }
}
