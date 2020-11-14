use super::*;

pub struct Ray<N: PhysicsScalar> {
    point: Vector3<N>,
    direction: Vector3<N>,
}

impl<N: FloatingPhysicsScalar> Ray<N> {
    pub fn get_point(&self, distance: N) -> Vector3<N> {
        self.point + self.direction * distance
    }
    pub fn to_plane(&self, normal: &Vector3<N>) -> Plane<N> {
        let d = -self.point.dot(normal);
        Plane { normal: *normal, d }
    }
    pub fn intersects_plane(&self, plane: &Plane<N>) -> Option<N> {
        let denom = plane.normal.dot(&self.direction);
        if N::is_zero(&denom) {
            None
        } else {
            let p = plane.random_point() - self.point;
            Some(p.dot(&plane.normal) / denom)
        }
    }
}

impl<N: FloatingPhysicsScalar> From<&Capsule<N>> for Ray<N> {
    fn from(capsule: &Capsule<N>) -> Self {
        let direction = capsule.direction();
        Ray {
            direction,
            point: capsule.start,
        }
    }
}
