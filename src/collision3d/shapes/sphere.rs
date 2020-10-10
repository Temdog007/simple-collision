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

impl Sphere {
    pub fn new(center: &Vector3<f32>, radius: f32) -> Sphere {
        Sphere {
            center: *center,
            radius,
        }
    }
}

impl Shape3D for Sphere {
    fn bounding_aabb(&self) -> AABB {
        AABB {
            start: self.center() - Vector3::from_element(self.radius),
            end: self.center() + Vector3::from_element(self.radius),
        }
    }
    fn bounding_sphere(&self) -> Sphere {
        *self
    }
    fn center(&self) -> Vector3<f32> {
        self.center
    }
    fn translate(&self, point: &Vector3<f32>) -> Sphere {
        Sphere {
            center: self.center + point,
            radius: self.radius,
        }
    }
    fn set_center(&self, point: &Vector3<f32>) -> Sphere {
        Sphere {
            center: *point,
            radius: self.radius,
        }
    }
    fn translate_mut(&mut self, point: &Vector3<f32>) {
        self.center += point
    }
    fn set_center_mut(&mut self, point: &Vector3<f32>) {
        self.center = *point;
    }
}
