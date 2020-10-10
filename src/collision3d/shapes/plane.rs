use nalgebra::*;

use super::*;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct Plane {
    pub normal: Vector3<f32>,
    pub d: f32,
}

impl Plane {
    pub fn from_point(normal: &Vector3<f32>, point: &Vector3<f32>) -> Plane {
        Plane {
            normal: *normal,
            d: -normal.dot(point),
        }
    }
    pub fn distance(&self, point: &Vector3<f32>) -> f32 {
        self.normal.dot(point) + self.d
    }
    pub fn random_point(&self) -> Vector3<f32> {
        if !is_zero(get_x(&self.normal)) {
            return Vector3::new(-self.d / get_x(&self.normal), 0f32, 0f32);
        }
        if !is_zero(get_y(&self.normal)) {
            return Vector3::new(0f32, -self.d / get_y(&self.normal), 0f32);
        }
        if !is_zero(get_z(&self.normal)) {
            return Vector3::new(0f32, 0f32, -self.d / get_z(&self.normal));
        }
        panic!("Failed to find random point on plane");
    }
    pub fn closest_point(&self, point: &Vector3<f32>) -> Vector3<f32> {
        point - self.normal * self.distance(point)
    }
}

impl Shape3D for Plane {
    fn bounding_aabb(&self) -> AABB {
        AABB {
            start: Vector3::from_element(std::f32::MIN),
            end: Vector3::from_element(std::f32::MAX),
        }
    }
    fn bounding_sphere(&self) -> Sphere {
        Sphere {
            center: Vector3::from_element(0f32),
            radius: std::f32::MAX,
        }
    }
    fn center(&self) -> Vector3<f32> {
        Vector3::from_element(0f32)
    }
    fn translate_mut(&mut self, _: &Vector3<f32>) {}
    fn set_center_mut(&mut self, _: &Vector3<f32>) {}
    fn translate(&self, _: &Vector3<f32>) -> Plane {
        *self
    }
    fn set_center(&self, _: &Vector3<f32>) -> Plane {
        *self
    }
}
