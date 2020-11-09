use super::*;

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct Capsule<N: PhysicsScalar> {
    pub start: Vector3<N>,
    pub end: Vector3<N>,
    pub radius: N,
}

impl<N: PhysicsScalar> Capsule<N> {
    pub fn distance(&self) -> N {
        (self.start - self.end).magnitude() + self.radius * N::from_f64(2f64).unwrap()
    }
    pub fn direction(&self) -> Vector3<N> {
        (self.start - self.end).normalize()
    }
    pub fn closest_point(&self, point: &Vector3<N>) -> Vector3<N> {
        closest_to_segment(&self.start, &self.end, point)
    }
    pub fn closest_points(&self, capsule: &Capsule<N>) -> (Vector3<N>, Vector3<N>) {
        let other_closest = capsule.closest_point(&self.start);
        let this_closest = self.closest_point(&other_closest);
        (this_closest, other_closest)
    }
    pub fn get_capsule_collision(&self, capsule: &Capsule<N>) -> Option<CollisionResolution<N>> {
        if self
            .bounding_aabb()
            .get_aabb_collision(&capsule.bounding_aabb())
            .is_none()
        {
            return None;
        }

        let (point1, point2) = self.closest_points(capsule);

        Sphere {
            center: point1,
            radius: self.radius,
        }
        .get_sphere_collision(&Sphere {
            center: point2,
            radius: self.radius,
        })
    }
    pub fn get_triangle_collision(
        &self,
        triangle: &Triangle<N>,
        double_sided: bool,
    ) -> Option<CollisionResolution<N>> {
        if self
            .bounding_aabb()
            .get_aabb_collision(&triangle.bounding_aabb())
            .is_none()
        {
            return None;
        }

        let mut points = [self.start, self.end, unsafe { std::mem::zeroed() }];

        let plane = Plane::from(triangle);
        let ray = Ray::from(self);

        if let Some(t) = ray.intersects_plane(&plane) {
            if N::zero() < t && t < self.distance() {
                unsafe {
                    *points.get_unchecked_mut(2) = ray.get_point(t);
                }
            }
        }

        points.sort_by(|a, b| {
            let (_, a) = triangle.closest_point(a);
            let (_, b) = triangle.closest_point(b);
            a.partial_cmp(&b).unwrap_or(Ordering::Equal)
        });

        points.iter().find_map(|point| {
            Sphere {
                center: *point,
                radius: self.radius,
            }
            .get_triangle_collision(triangle, double_sided)
        })
    }
    pub fn get_plane_collision(&self, plane: &Plane<N>) -> Option<CollisionResolution<N>> {
        let ray = Ray::from(self);
        match ray.intersects_plane(plane) {
            Some(t) if N::zero() < t && t < self.distance() => {
                let center = ray.get_point(t);
                Sphere {
                    center,
                    radius: self.radius,
                }
                .get_plane_collision(plane)
            }
            _ => None,
        }
    }
    pub fn to_spheres(&self) -> (Sphere<N>, Sphere<N>) {
        let radius = self.radius;
        (
            Sphere {
                radius,
                center: self.start,
            },
            Sphere {
                radius,
                center: self.end,
            },
        )
    }
}

impl<N: PhysicsScalar> Shape3D<N> for Capsule<N> {
    fn bounding_aabb(&self) -> AxisAlignedBoundingBox<N> {
        let (start, end) = self.to_spheres();
        start.bounding_aabb() + end.bounding_aabb()
    }
    fn bounding_sphere(&self) -> Sphere<N> {
        self.bounding_aabb().bounding_sphere()
    }
    fn center(&self) -> Vector3<N> {
        self.start * N::from_f64(0.5f64).unwrap() + self.end * N::from_f64(0.5f64).unwrap()
    }
    fn translate(&self, point: &Vector3<N>) -> Self {
        Capsule {
            radius: self.radius,
            start: self.start + *point,
            end: self.end + *point,
        }
    }
    fn translate_mut(&mut self, point: &Vector3<N>) {
        self.start += *point;
        self.end += *point;
    }
    fn set_center(&self, point: &Vector3<N>) -> Self {
        let mut c = *self;
        c.set_center_mut(point);
        c
    }
    fn set_center_mut(&mut self, new_center: &Vector3<N>) {
        let center = self.center();
        let start_dist = self.start - center;
        let end_dist = self.end - center;
        self.start = new_center + start_dist;
        self.end = new_center + end_dist;
    }
}
