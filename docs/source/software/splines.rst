Splines
=======
A spline is an aggregation of several piecewise functions that is used to fit a smooth curve(approximately) passing through several vector/data points.

Splines provide smoothness and control which makes them a useful and important interpolation technique in practical scenarios

One such class of Splines "Cubic Splines" are used in our code

Key Properties of Cubic Splines
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1. They are continuous at the merging points

2. They have continuous first and second derivatives at the points where the piecewise functions join/merge


Calculating A Cubic Spline 
^^^^^^^^^^^^^^^^^^^^^^^^^^
A cubic spline is made up of piecewise cubic polynomials each satisfying the interval and the endpoints of the interval they are defined in.

Let's say there are n + 1 points (x0, y0), (x1, y1), (x, y2) ....... (xn, yn)

We want to fit a cubic spline through these n+1 points or we can say that we need to find n cubic equations that make up the spline over these n-intervals 

Any cubic equation passing through a point x_i, y_i can be written in the form :

y_i = a + b(x - x_i) + c(x-x_i)² + d(x-x_i)³

Here we have 4 unknowns for one cubic equation. So for n cubic equations, we will have 4n unknowns

As cubic splines satisfy certain properties, they serve as constraints for these unknowns 

1. The spline passes through all the n+1 points hence it satisfies y0, y1, .... yn. These are n+1 constraints

2. The spline is continuous at all the points(except end-points) so we have additional n-1 constraints

3. The first and second derivatives of the spline are also continuous at these n-1 points hence we have 2(n-1)more constraints

In total, we have 4n-2 constraints and 4n unknowns till now. This is an under-determined system so to make it solvable two additional constraints are enforced externally (generally that the second derivative = 0 at the end-points)

Spline Representation
^^^^^^^^^^^^^^^^^^^^^^

f_i(x) = y_i(== a_i) + b_i(x - x_i) + c_i(x - x_i)² + d_i(x - x_i)³

For points (x_i, y_i) and parameters a_i, b_i, c_i, d_i and f_i(x) are the individual piecewise cubic functions

We use Matrix Algebra to solve for the system reduced to a matrix equation AX = B 

Use In Code 
^^^^^^^^^^^^
Used to calculate global paths (csp) from waypoints and local paths while sampling 

References
^^^^^^^^^^^
http://www.cs.cornell.edu/courses/cs4620/2013fa/lectures/16spline-curves.pdf

https://en.wikipedia.org/wiki/Spline_(mathematics)

