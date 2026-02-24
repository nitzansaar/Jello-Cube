# Jello-Cube

https://github.com/user-attachments/assets/8ba1e4ab-2af7-48e3-b41e-2d8178b7ea78



This assignment is a simulation of a jello cube in a box. It reads a 'world file' which contains the instructions of the physics that will be acted upon the jello cube. The jello cube is composed of 512 points that are all connected by edges. There are different types of edges/springs.

A. Structural edges
B. Shear edges
C. Bend edges

Each of the edges connects the points in separate ways. Because there are these different types of edges, it wouldn't be enough to simply calculate the force acting on each point by its direct edge. Rather, I needed to calculate the force acting upon each point by its neighbors from the different edge types. My implementation goes through each point in the jello cube and individually computes the force using Hook's law on each point from the different edge types. Then accumulate the total force for that point and calculate the acceleration for that point by Newton's second law (a = F/m) dividing the force by the mass of that point. This is done for every point in the cube and finally an array of accelerations is returned. This array is ultimately used in the Euler and RK4 methods to update the jello structure.

**Hook's Law**

For each spring connection, I compute the force using Hook's law: F = k * (length - rest_length) * direction. If the spring is stretched beyond its rest length, it pulls the point toward its neighbor. If it's compressed, it pushes the point away. I also add a damping force to prevent the cube from oscillating forever. The damping force resists motion along the spring direction, acting like friction.

**Structural Springs**

Structural springs connect each point to its 6 direct neighbors (up, down, left, right, front, back). These are the axis-aligned connections and they resist stretching along the main axes. The rest length is simply the spacing between adjacent grid points (2.0 / 7.0, since the cube spans a length of 2 and is divided into 7 intervals).

**Shear Springs**

Shear springs connect each point to its 20 diagonal neighbors. 12 of these are face diagonals (diagonal across two axes) and 8 are body diagonals (diagonal across all three axes). These prevent the cube from collapsing diagonally, like how cross-bracing keeps a bookshelf from leaning. The rest length is calculated using the Pythagorean theorem since the diagonal distance is longer than the axis-aligned spacing.

**Bend Springs**

Bend springs connect each point to the point two steps away along each axis (skipping one neighbor). There are 6 of these per point. They resist the cube from bending or folding too sharply. The rest length is twice the regular spacing.

**External Force Field**

The world file can also specify an external force field, which is a 3D grid of force vectors that act on the jello cube (like wind or gravity pulling in different directions at different locations). Since the jello points won't always line up exactly with the force field grid, I use trilinear interpolation to estimate the force at each point's exact position. This works by finding the 8 surrounding grid points and taking a weighted average of their force values based on how close the jello point is to each one.

**Collision Forces**

The jello cube is contained within a bounding box from -2 to 2 in all directions. If any point moves past a wall, a penalty spring pushes it back inside. The further the point penetrates the wall, the stronger the push. Damping is also applied to prevent the point from bouncing off the wall forever.

**Two Jello Cubes**

The simulation features two jello cubes bouncing and colliding inside the bounding box. A second cube (`jello2`) is initialized by mirroring the first cube's positions across the origin (negating x and y) and reversing its x and y velocities so the two cubes start on opposite sides and move toward each other. Both cubes share the same physics parameters (spring constants, damping, mass, integrator, timestep).

The cubes are scaled to half size so they have room to move around inside the box. All spring rest lengths and the inter-cube collision threshold are scaled accordingly.

Inter-cube collision uses the same penalty-spring model as wall collision. For every pair of points between the two cubes, if the distance is less than one (scaled) grid spacing, a repulsive penalty force pushes the points apart and a damping term resists inward relative velocity. This prevents the cubes from passing through each other.


Integration uses a "frozen other" approach: at each timestep, snapshots of both cubes are taken, and each cube is integrated against the other's snapshot. This avoids order-dependent artifacts and is a standard game physics simplification that is accurate with small dt.

Cube 1 is rendered with a blue material and cube 2 with a warm orange/red material so they are visually distinct.
