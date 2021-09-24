Polynomials
===========
The planning occurs via separation of the task into two independent planning tasks - the longitudinal planning and the lateral planning, and its subsequent integration. In each sampling interval, various paths are sampled in order to figure out the most optimal path. These paths are modelled by making use of polynomials which give the value of the longitudinal/lateral distances and their derivatives at the time instants in between the start and the end point. Therefore, between every time interval, the longitudinal and lateral distance can be thought of as a function of time. The most jerk optimal trajectory has been proven to be given by a quintic polynomial and hence we make use of quintic polynomials in this case.

s = f(t) =  a₀ + a₁t + a₂t² + a₃t³ +a₄t⁴ + a₅t⁵

d = f(t) = b₀ + b₁t + b₂t² + b₃t³ +b₄t⁴ + b₅t⁵, where aₗ and bₗ are constants.

The constants can be determined using the end conditions. At any point before determining these functions we are provided with the position, velocity and acceleration at the end points, via sampling. These provide us with a total of 6 equations which can be used directly to solve for these constants.

The following equations can be directly obtained:

s(t)= a₀ + a₁t + a₂t² + a₃t³ +a₄t⁴ + a₅t⁵

s(0) = a₀ = xs

s(T) =a₀ + a₁T + a₂T² + a₃T³ + a₄T⁴ + a₅T⁵  = xe

The first derivative of the function gives the velocity at different end points.

s’(t) = a₁ + 2a₂t + 3a₃t² + 4a₄t³ + 5a₅t⁴

s’(0) = a₁ = vxs

s’(T) = a₁ + 2a₂T + 3a₃T² + 4a₄T³ + 5a₅T⁴ = vxe

Similarly, the second derivative of the equation gives the acceleration at different end points. 

s’’(t) = 2a₂ + 6a₃t + 12a₄t² + 20a₅t³

s’’(0) = 2a₂ = axs

s’’(T) = 2a₂ + 6a₃T + 12a₄T² + 20a₅T³ = axe

Therefore, we have six equations and six variables. These equations can be easily solved using methods of Linear Algebra.

Similarly, we obtain a similar set of equations for lateral distance.

d(0) = a₀ = xs

d(T) =a₀ + a₁T + a₂T² + a₃T³ + a₄T⁴ + a₅T⁵  = xe

d’(0) = a₁ = vxs

d’(T) = a₁ + 2a₂T + 3a₃T² + 4a₄T³ + 5a₅T⁴ = vxe

d’’(0) = 2a₂ = axs

d’’(T) = 2a₂ + 6a₃T + 12a₄T² + 20a₅T³ = axe

Under the quintic polynomial class, we further have a few methods to help calculate the distance(longitudinal/lateral), velocity and acceleration at different time instants.
These can easily be calculated using the functions that have been evaluated previously.

s(t)/d(t)= a₀ + a₁t + a₂t² + a₃t³ +a₄t⁴ + a₅t⁵

s’(t)/d'(t) = a₁ + 2a₂t + 3a₃t² + 4a₄t³ + 5a₅t⁴

s’’(t)/d''(t) = 2a₂ + 6a₃t + 12a₄t² + 20a₅t³

Further, there exists a similar class called quartic polynomials, which works similarly to calculate functions and has similar methods to calculate various parameters at different time instants.
