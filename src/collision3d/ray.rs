use super::*;
use nalgebra::*;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct Ray<N: PhysicsScalar> {
    point: Vector3<N>,
    direction: Vector3<N>,
}

impl<N: PhysicsScalar> Ray<N> {
    pub fn new(point: &Vector3<N>, direction: &Vector3<N>) -> Self {
        Ray {
            point: *point,
            direction: *direction,
        }
    }
    pub fn get_point(&self, dist: N) -> Vector3<N> {
        self.point + self.direction * dist
    }
    pub fn to_plane(&self, normal: &Vector3<N>) -> Plane<N> {
        let d = -normal.dot(&self.point);
        Plane { normal: *normal, d }
    }
}
