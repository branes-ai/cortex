---
title: Visual Front End
description: Image pyramid, FAST corner detection, and pyramidal KLT tracking — turning frames into stable feature observations.
---

The visual front end (`cv/`, namespace `branes::cv`) turns a grayscale image stream
into **stable, id-tagged feature observations** for the estimator. It is built from
four primitives.

## Image container & I/O (#48)

`cv::Image<T>` is a non-owning view (pointer + dimensions + stride); `OwnedImage<T>`
owns the pixels. I/O reads PGM (`read_pgm`) and PNG (`read_png`, via stb_image). The
view type is what crosses into the estimator, so frames are borrowed, not copied.

## Gaussian pyramid (#49)

`cv::Pyramid<T>` builds 3–5 levels, each a separable-Gaussian blur of the previous
resampled down by a scale factor (default 2.0). Level 0 is the full-resolution copy.
The pyramid is what makes KLT robust to larger inter-frame motion — tracking starts
coarse and refines.

## FAST corner detector (#37)

`detect_fast(image, threshold, nms_radius)` implements FAST-9 (a pixel is a corner if
a contiguous arc of ≥9 of the 16 ring pixels is uniformly brighter/darker than the
center by a threshold), followed by non-maximum suppression on the corner response.
Each `KeyPoint` carries `(x, y, response)`.

> A note recorded during development: checkerboard **X-junctions are not FAST-9
> corners** — FAST fires on convex corners (e.g. the corners of an isolated bright
> square), not saddle points. Test fixtures use square grids, not checkerboards.

## Pyramidal KLT tracker (#38)

`track_klt_pyramidal(prev, next, points, params)` follows points from the previous
pyramid into the next with inverse-compositional Lucas–Kanade, coarse-to-fine across
levels. Each `KltResult` reports the new `(x, y)` and a `TrackStatus`
(`Tracked` / `Lost` / `OutOfBounds`). Tuning (`KltParams`) covers window half-size,
iteration count, convergence epsilon, and a minimum eigenvalue threshold that rejects
ill-conditioned (textureless) windows.

## How they compose in the estimator

The [`VioEstimator`](/vio/vio-estimator/) owns the front-end state. On each frame it:

1. builds a pyramid of the new image;
2. **KLT-tracks** the existing features into it, dropping `Lost`/`OutOfBounds` ones;
3. **re-detects** the strongest FAST corners to top the count back up, suppressing
   any within `min_feature_distance` of an existing track;
4. assigns stable feature ids and emits one `FrontendObservation` (id, camera id,
   pixel `u,v`) per surviving track to the backend.

This detect-then-track loop is what produces the *multi-view* tracks the MSCKF
consumes — a feature seen across several frames becomes a constraint linking the
corresponding cloned poses.
