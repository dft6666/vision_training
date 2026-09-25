# Task 2: Synthetic Rotation Video Parameter Fitting

## Model

$$\omega(t)=b+A\sin(\Omega t+\phi)$$

$$\theta(t)=\theta_0+bt+\frac{A}{\Omega}[\cos\phi-\cos(\Omega t+\phi)]$$

## Estimated Parameters

- A = 0.150827 rad/s
- b = 1.34716 rad/s
- Omega = 2.03523 rad/s
- phi = -0.776962 rad
- theta0 = 0.586967 rad
- Velocity variation period = 3.08721 s

## Method

The cyan target center is extracted using HSV segmentation, contour detection and image moments. The target angle is calculated using atan2 and then unwrapped. The integrated angle model is fitted using Ceres Solver with automatic differentiation.

The parameterization b = A + d with A > 0 and d > 0 is used to enforce b > A. Omega is also constrained to be positive.

## Error

- Angle RMSE = 0.226276 rad
- Valid samples = 1440
- Frame range = 0 - 1439
- FPS = 60

## Outputs

- `result/task2_fit/tracking_overlay.mp4`
- `result/task2_fit/fit_comparison.png`
- `result/task2_fit/angular_velocity.png`
- `result/task2_fit/residuals.png`
