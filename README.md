# intersection_detection

This crate provides functionality for computing intersections between line segments in 2D space.
It defines types for representing intersections and includes methods for checking intersection results, 
converting between intersection types, and rounding intersection components to a specified precision.

## Usage

To use this crate, add it as a dependency to your `Cargo.toml` file:

```rust
[dependencies]
intersection_detection = "0.1"
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
- `(F, F, F)`.
