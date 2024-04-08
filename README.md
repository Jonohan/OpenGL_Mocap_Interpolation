# Motion Interpolation in Animation
## Overview
This project focuses on implementing various interpolation techniques to animate motion sequences from motion capture data. Utilizing the AMC (Acclaim Motion Capture) format, the program interpolates joint rotations represented by Euler angles and quaternions to generate smooth and realistic animations.

![Demo Animation](./bq_be_inter.gif)

## Features
* Euler Angle and Quaternion Representations: Implements dual support for both Euler angles and quaternion representations of joint rotations.
* Linear and Bezier Interpolation in Euler Space: Applies linear and Bezier interpolation techniques to compute intermediate frames in Euler space.
* Quaternion Interpolation: Incorporates Spherical Linear Interpolation (SLERP) and Bezier SLERP for quaternions, offering a smoother and more realistic motion transition.
* Non-Uniform Keyframes Support: Enhances the flexibility of animation by supporting keyframes that are non-uniform in time.
* Root Position Interpolation: For quaternion interpolations, the root position is interpolated using corresponding Euler interpolation techniques (Linear for Linear Quaternion, Bezier for Bezier Quaternion).
