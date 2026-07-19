# PocketCar

A top-down, toy-sized car model for building intuition about how a real car moves at low
speed: why the rear wheels cut the corner, how far the nose and tail sweep out through a
turn, and why you parallel park in reverse.

**Live demo: https://sarfios20.github.io/pocketcar/**

No dependencies: open `index.html` in a browser and go.

## Use

- **Drag the car** with the mouse to push it around like a toy (or `↑` / `↓`).
  It can only move along the arc its wheels allow, so pulling sideways does nothing,
  the same constraint a real car has.
- **Steering**: scroll wheel, `←` / `→` (or `A` / `D`), or drag the steering wheel at the bottom.
- `R` resets the scenario · `C` clears the tracks · `G` toggles the turning geometry.

## What to watch

- **Green** track: rear wheels. **Orange** track: front wheels. Green always runs inside;
  that gap is what wipes out whatever sits on the inside of the curve.
- The amber outlines are the **body swept path**: anything inside that corridor gets hit,
  even if no wheel touches it.
- With `G` on, the app draws the **instantaneous turning center** and the two radii that
  matter: the smallest (inner rear wheel) and the largest (outer body corner).

## The model

Kinematic bicycle model referenced to the rear axle (valid at low speed, no skidding),
which is exactly the regime of parking and tight corners. Collisions use oriented bounding
boxes with the separating-axis test; motion integrates in 2 px sub-steps and stops on
contact. Scale: 30 px = 1 m; a 4.4 × 1.8 m car with a 2.7 m wheelbase and 35° max road-wheel
angle.

## Debugging

URL parameters: `?sc=N` loads scenario N (0 to 3) and
`?demo=fwd:400,steer:-35,fwd:300` drives it in a scripted way (pixels and degrees).
