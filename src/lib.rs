use derive_new::new;
use num::Float;
use std::marker::PhantomData;

use point_like::*;
pub use point_like::{self, FromInto as FromIntoPointLike, PointLike};

#[derive(Debug, PartialEq, Clone)]
pub enum IntersectionResult<F: Float, P: PointLike<F>> {
    Intersection(Intersection<F, P>),
    Parallel,
    None,
}

impl<F: Float, P: PointLike<F>> IntersectionResult<F, P> {
    /// Returns `true` if the intersection result is [`Intersection`].
    ///
    /// [`Intersection`]: IntersectionResult::Intersection
    #[must_use]
    pub fn is_intersection(&self) -> bool {
        matches!(self, Self::Intersection(..))
    }

    /// Convert from `IntersectionResult<F, P>` to `Result<Intersection<F, P>, Self>`.
    ///
    /// Converts `IntersectionResult::Parallel` or `IntersectionResult::None` to `Err(self)`.
    pub fn try_into_intersection(self) -> Result<Intersection<F, P>, Self> {
        if let Self::Intersection(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    /// Convert from `IntersectionResult<F, P>` to `Option<&Intersection<F, P>>`.
    ///
    /// Discards `IntersectionResult::Parallel` and `IntersectionResult::None` variants.
    pub fn as_intersection(&self) -> Option<&Intersection<F, P>> {
        if let Self::Intersection(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

impl<F: Float, P: PointLike<F>> PrecisionRound for IntersectionResult<F, P> {
    /// Round each component in the result of an intersection computation to `precision` decimal places.
    ///
    /// This does not operate on `Parallel` or `None` variants.
    #[inline]
    fn round(self, precision: usize) -> Self {
        match self {
            Self::Intersection(intersection) => Self::Intersection(intersection.round(precision)),
            Self::Parallel => Self::Parallel,
            Self::None => Self::None,
        }
    }
}

/// An intersection between two [`Line`].
#[derive(Debug, PartialEq, new, Clone)]
pub enum Intersection<F: Float, P: PointLike<F>> {
    Point(P),
    CollinearSegment(Line<F, P>),
    Coincident(Line<F, P>),
}

impl<F: Float, P: PointLike<F>> PrecisionRound for Intersection<F, P> {
    /// Round an intersection's components to `precision` decimal places.
    #[inline]
    fn round(self, precision: usize) -> Self {
        match self {
            Self::Point(p) => Self::Point(p.round(precision)),
            Self::CollinearSegment(line) => Self::CollinearSegment(line.round(precision)),
            Self::Coincident(line) => Self::Coincident(line.round(precision)),
        }
    }
}

impl<F: Float, P: PointLike<F>> From<P> for Intersection<F, P> {
    fn from(point: P) -> Self {
        Self::Point(point)
    }
}

/// A line segment from `start` to `end`.
///
/// See: [`PointLike`]
#[derive(Eq, Debug, new, Clone, Copy)]
pub struct Line<F: Float + PartialEq, P: PointLike<F>> {
    pub start: P,
    pub end: P,
    #[new(default)]
    marker: PhantomData<F>,
}

impl<F: Float, P: PointLike<F>> PartialEq for Line<F, P> {
    fn eq(&self, other: &Self) -> bool {
        let epsilon = F::epsilon();
        self.start.approx_eq(other.start, epsilon) && self.end.approx_eq(other.end, epsilon)
    }
}

impl<F: Float, P: PointLike<F>> PrecisionRound for Line<F, P> {
    /// Round a line segment's start and end points to `precision` decimal places.
    #[inline]
    fn round(self, precision: usize) -> Self {
        Self::new(self.start.round(precision), self.end.round(precision))
    }
}

impl<F: Float, P: PointLike<F>> ApproxEq<&Self, F> for Line<F, P> {
    /// Approximate equality between two line segments.
    ///
    /// See: [`PointLike::approx_eq`]
    #[inline]
    fn approx_eq(&self, other: &Self, epsilon: F) -> bool {
        self.start.approx_eq(other.start, epsilon) && self.end.approx_eq(other.end, epsilon)
    }
}

impl<F: Float, P: PointLike<F>> Line<F, P> {
    /// Compute the length of the line segment.
    #[inline]
    pub fn length(&self) -> F {
        self.start.distance(self.end)
    }

    /// Compute the non-normalized direction of self.
    #[inline]
    pub fn direction(&self) -> P {
        self.end.sub(self.start)
    }

    /// Compute the intersection between two line segments, if any.
    pub fn intersection(&self, other: &Self, epsilon: F) -> IntersectionResult<F, P> {
        if self.approx_eq(&other, epsilon) {
            return IntersectionResult::Intersection(Intersection::Coincident(*self));
        }
        let zero = F::zero();
        let one = F::one();

        let p = self.start;
        let r = self.direction();
        let q = other.start;
        let s = other.direction();
        let rs = r.cross(s);
        let qpr = q.sub(p).cross(r);

        let rs_is_zero = rs.approx_eq(zero, epsilon);
        let qpr_is_zero = qpr.approx_eq(zero, epsilon);
        let r_len = r.length();
        if rs_is_zero && qpr_is_zero && r_len.gt(&zero) {
            let t1 = q.add(s.sub(p)).dot(r) / r.dot(r);
            let t0 = t1.sub(s.dot(r) / r_len);

            if t0.ge(&zero) && t0.le(&one) || t1.ge(&zero) && t1.le(&one) {
                let first_point = self.start.max(other.start);
                let second_point = self.end.min(other.end);
                let line = Line::new(first_point, second_point);

                return IntersectionResult::Intersection(Intersection::CollinearSegment(line));
            }
        }

        if rs_is_zero && !qpr_is_zero {
            return IntersectionResult::Parallel;
        }

        let t = q.sub(p).cross(s) / rs;
        let u = qpr / rs;
        if t >= zero && t <= one && u >= zero && u <= one {
            let intersection = self.start.add(r.mul_f(t));
            return IntersectionResult::Intersection(Intersection::Point(intersection));
        }

        IntersectionResult::None
    }
}

impl<F: Float, P: PointLike<F>> From<(P, P)> for Line<F, P> {
    fn from((start, end): (P, P)) -> Self {
        Self::new(start, end)
    }
}

impl<F: Float, P: PointLike<F>> From<[P; 2]> for Line<F, P> {
    fn from([start, end]: [P; 2]) -> Self {
        Self::new(start, end)
    }
}

impl<F: Float, P: PointLike<F>> From<(&P, &P)> for Line<F, P>
where
    P: Clone,
{
    fn from((start, end): (&P, &P)) -> Self {
        Self::new(start.to_owned(), end.to_owned())
    }
}

impl<F: Float, P: PointLike<F>> From<[&P; 2]> for Line<F, P>
where
    P: Clone,
{
    fn from([start, end]: [&P; 2]) -> Self {
        Self::new(start.to_owned(), end.to_owned())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;

    #[rstest]
    // 🙈 NONE
    #[case(
        Line::new([0.0_f32, 0.0_f32], [0.0_f32, 1.0_f32]),
        Line::new([0.5_f32, 0.0_f32], [1.0_f32, 0.0_f32]),
        IntersectionResult::None,
    )]
    // ➕ PERPENDICULAR 1:1
    #[case(
        Line::new([0.0_f32, 0.5_f32], [1.0_f32, 0.5_f32]), // —
        Line::new([0.0_f32, 0.0_f32], [1.0_f32, 1.0_f32]), // |
        IntersectionResult::Intersection(Intersection::Point([0.5_f32, 0.5_f32]))
    )]
    // ✖️ CROSSING (1:1)
    #[case(
        Line::new([0.0_f32, 0.0_f32], [1.0_f32, 1.0_f32]),
        Line::new([0.0_f32, 1.0_f32], [1.0_f32, 0.0_f32]),
        IntersectionResult::Intersection(Intersection::Point([0.5_f32, 0.5_f32]))
    )]
    // ✖️ CROSSING (ASYMMETRIC)
    #[case(
        Line::new([0.0_f32, 0.0_f32], [5.0_f32, 5.0_f32]),
        Line::new([0.0_f32, 10.0_f32], [3.0_f32, 0.0_f32]),
        IntersectionResult::Intersection(Intersection::Point([2.308_f32, 2.308_f32]))
    )]
    // ┃ COINCIDENT
    #[case(
        Line::new([0.0_f32, 0.0_f32], [0.0_f32, 1.0_f32]),
        Line::new([0.0_f32, 0.0_f32], [0.0_f32, 1.0_f32]),
        IntersectionResult::Intersection(Intersection::Coincident(Line::new([0.0_f32, 0.0_f32], [0.0_f32, 1.0_f32])))
    )]
    // ╿ COLLINEAR SEGMENT (VERTICAL)
    #[case(
        Line::new([0.0_f32, 0.0_f32], [0.0_f32, 1.0_f32]),
        Line::new([0.0_f32, 0.5_f32], [0.0_f32, 1.5_f32]),
        IntersectionResult::Intersection(Intersection::CollinearSegment(Line::new([0.0_f32, 0.5_f32], [0.0_f32, 1.0_f32])))
    )]
    // ╿ COLLINEAR SEGMENT (DIAGONAL)
    #[case(
        Line::new([0.0_f32, 0.0_f32], [1.0_f32, 1.0_f32]),
        Line::new([0.5_f32, 0.5_f32], [1.5_f32, 1.5_f32]),
        IntersectionResult::Intersection(Intersection::CollinearSegment(Line::new([0.5_f32, 0.5_f32], [1.0_f32, 1.0_f32])))
    )]
    // ╻┃ PARALLEL (VERTICAL, ASYMMETRIC LEN)
    #[case(
        Line::new([0.0_f32, 0.0_f32], [0.0_f32, 1.0_f32]),
        Line::new([1.0_f32, 0.0_f32], [1.0_f32, 2.0_f32]),
        IntersectionResult::Parallel
    )]
    // ╱╱ PARALLEL (DIAGONAL, SYMMETRIC LEN)
    #[case(
        Line::new([0.0_f32, 0.0_f32], [1.0_f32, 1.0_f32]),
        Line::new([0.0_f32, 2.0_f32], [1.0_f32, 3.0_f32]),
        IntersectionResult::Parallel
    )]

    fn test_intersections(
        #[case] line1: Line<f32, [f32; 2]>,
        #[case] line2: Line<f32, [f32; 2]>,
        #[case] expected: IntersectionResult<f32, [f32; 2]>,
    ) {
        // Round for use with assert_eq!.
        let computation = line1.intersection(&line2, 0.001).round(3);
        assert_eq!(computation, expected);
    }

    #[test]
    fn test() {
        let line1 = Line::new([0.0, 0.0], [1.0, 1.0]);
        let line2 = Line::new([0.0, 1.0], [1.0, 0.0]);
        let computation = line1
            .intersection(&line2, f32::EPSILON)
            .try_into_intersection()
            .ok();

        assert_eq!(computation, Some(Intersection::Point([0.5, 0.5])));
    }
}
