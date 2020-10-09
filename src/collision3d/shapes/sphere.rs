use super::*;
use nalgebra::*;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct Sphere {
    pub center: Vector3<f32>,
    pub radius: f32,
}

impl Shape3D for Sphere {
    fn bounding_aabb(&self) -> AABB {
        AABB {
            start: self.center() - from_f32(self.radius),
            end: self.center() + from_f32(self.radius),
        }
    }
    fn bounding_sphere(&self) -> Sphere {
        *self
    }
    fn center(&self) -> Vector3<f32> {
        self.center
    }
    fn translate(&mut self, point: &Vector3<f32>) {
        self.center += point
    }
    fn set_center(&mut self, point: &Vector3<f32>) {
        self.center = *point;
    }
}
