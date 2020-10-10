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
    pub fn get_sphere_collision(&self, sphere: &Sphere) -> Option<CollisionResolution> {
        let n = self.center - sphere.center;
        let r = (self.radius + sphere.radius).powi(2);

        let len = n.magnitude_squared();
        if len > r {
            return None;
        }

        let d = len.sqrt();
        let (normal, penetration) = if is_zero(d) {
            (Vector3::new(1f32, 0f32, 0f32), self.radius)
        } else {
            let pen = r.sqrt() - d;
            (n.normalize(), pen)
        };
        Some(CollisionResolution {
            normal,
            penetration,
        })
    }
    pub fn get_plane_collision(&self, plane: &Plane) -> Option<CollisionResolution> {
        let dist = (self.center - plane.random_point()).dot(&plane.normal);
        if dist < 0f32 || dist > self.radius {
            None
        } else {
            Some(CollisionResolution {
                normal: plane.normal,
                penetration: self.radius - dist,
            })
        }
    }
    pub fn get_triangle_collision(&self, triangle: &Triangle) -> Option<CollisionResolution> {
        let normal = triangle.normal();
        let dist = (self.center - triangle.point1).dot(&normal);
        if dist < 0f32 || dist > self.radius {
            return None;
        }

        let point = self.center - normal * dist;
        if triangle.contains(&point) {
            Some(CollisionResolution {
                normal,
                penetration: self.radius - dist,
            })
        } else {
            None
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
