# intersection_detection

This crate provides functionality for computing intersections between line segments in 2D space.
It defines types for representing intersections and includes methods for checking intersection results,
converting between intersection types, and rounding intersection components to a specified precision.

## Usage

To use this crate, add it as a dependency to your `Cargo.toml` file:

```rust
[dependencies]
intersection_detection = "0.1.2"
```

## Example

```rust
use intersection_detection::{IntersectionResult, Line, PointLike};

fn main() {
    let line1 = Line::new([0.0, 0.0], [1.0, 1.0]);
    let line2 = Line::new([0.0, 1.0], [1.0, 0.0]);

    let computation = line1
            .intersection(&line2, f32::EPSILON)
            .try_into_intersection()
            .ok();

    assert_eq!(computation, Some(Intersection::Point([0.5, 0.5])));
}
```

## Points

Implement `FromIntoPointLike` to use custom types as points.

Out-of-the-box implementations are provided for:

- `[F; 2]`
- `[F; 3]`
- `(F, F)`
- `(F, F, F)`

> **Note:** This crate re-exports the `point_like` crate, which is a trait for types that can be used as points.
> The idea here was to let users determine float precision and avoid forcing usage of a specific algebra crate.
> However, most of the methods in `PointLike` are "inspired" by the `glam` crate, so I would recommend using that if you're fine with `f32`.
