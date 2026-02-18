/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

  createWorld utility to create your own world files

  Note: this utility uses its own copy of writeWorld routine, which is identical to the one
  found in input.cpp . If you need to change that routine, or even the definition of the
  world structure (you don't have to do this unless you decide to do some fancy
  extra credit), you have to update both copies.

*/

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

struct point 
{
   double x;
   double y;
   double z;
};

struct world
{
  char integrator[10]; // "RK4" or "Euler"
  double dt; // timestep, e.g.. 0.001
  int n; // display only every nth timestep
  double kElastic; // Hook's elasticity coefficient for all springs except collision springs
  double dElastic; // Damping coefficient for all springs except collision springs
  double kCollision; // Hook's elasticity coefficient for collision springs
  double dCollision; // Damping coefficient collision springs
  double mass; // mass of each of the 512 control points, mass assumed to be equal for every control point
  int incPlanePresent; // Is the inclined plane present? 1 = YES, 0 = NO
  double a,b,c,d; // inclined plane has equation a * x + b * y + c * z + d = 0; if no inclined plane, these four fields are not used
  int resolution; // resolution for the 3d grid specifying the external force field; value of 0 means that there is no force field
  struct point * forceField; // pointer to the array of values of the force field
  struct point p[8][8][8]; // position of the 512 control points
  struct point v[8][8][8]; // velocities of the 512 control points
};


/* writes the world parameters to a world file on disk*/
/* fileName = string containing the name of the output world file, ex: jello1.w */
/* function creates the output world file and then fills it corresponding to the contents
   of structure 'jello' */
/* function aborts the program if can't access the file */

/* writes the world parameters to a world file on disk*/
/* fileName = string containing the name of the output world file, ex: jello1.w */
/* function creates the output world file and then fills it corresponding to the contents
   of structure 'jello' */
/* function aborts the program if can't access the file */
void writeWorld(const char * fileName, struct world * jello)
{
  int i,j,k;
  FILE * file;
  
  file = fopen(fileName, "w");
  if (file == NULL) {
    printf ("can't open file\n");
    exit(1);
  }

  /* write integrator algorithm */ 
  fprintf(file,"%s\n",jello->integrator);

  /* write timestep */
  fprintf(file,"%lf %d\n",jello->dt,jello->n);

  /* write physical parameters */
  fprintf(file, "%lf %lf %lf %lf\n", 
    jello->kElastic, jello->dElastic, jello->kCollision, jello->dCollision);

  /* write mass */
  fprintf(file, "%lf\n", jello->mass);

  /* write info about the plane */
  fprintf(file, "%d\n", jello->incPlanePresent);
  if (jello->incPlanePresent == 1)
    fprintf(file, "%lf %lf %lf %lf\n", jello->a, jello->b, jello->c, jello->d);

  /* write info about the force field */
  fprintf(file, "%d\n", jello->resolution);
  if (jello->resolution != 0)
    for (i=0; i<= jello->resolution-1; i++)
      for (j=0; j<= jello->resolution-1; j++)
        for (k=0; k<= jello->resolution-1; k++)
          fprintf(file, "%lf %lf %lf\n", 
             jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].x, 
             jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].y, 
             jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].z);
  

  /* write initial point positions */
  for (i = 0; i <= 7 ; i++)
  {
    for (j = 0; j <= 7; j++)
    {
      for (k = 0; k <= 7; k++)
        fprintf(file, "%lf %lf %lf\n", 
          jello->p[i][j][k].x, jello->p[i][j][k].y, jello->p[i][j][k].z);
    }
  }
      
  /* write initial point velocities */
  for (i = 0; i <= 7 ; i++)
  {
    for (j = 0; j <= 7; j++)
    {
      for (k = 0; k <= 7; k++)
        fprintf(file, "%lf %lf %lf\n", 
          jello->v[i][j][k].x, jello->v[i][j][k].y, jello->v[i][j][k].z);
    }
  }

  fclose(file);
  
  return;
}

// helper: set default cube positions (centered, -1 to 1) and zero velocities
void initCube(struct world * jello)
{
  for (int i=0; i<=7; i++)
    for (int j=0; j<=7; j++)
      for (int k=0; k<=7; k++)
      {
        jello->p[i][j][k].x = -1.0 + 2.0 * i / 7;
        jello->p[i][j][k].y = -1.0 + 2.0 * j / 7;
        jello->p[i][j][k].z = -1.0 + 2.0 * k / 7;
        jello->v[i][j][k].x = 0.0;
        jello->v[i][j][k].y = 0.0;
        jello->v[i][j][k].z = 0.0;
      }
}

// helper: set common simulation parameters
void initParams(struct world * jello)
{
  strcpy(jello->integrator, "RK4");
  jello->dt = 0.001;
  jello->n = 1;
  jello->kElastic = 500.0;
  jello->dElastic = 0.5;
  jello->kCollision = 500.0;
  jello->dCollision = 0.5;
  jello->mass = 1.0 / 512;
  jello->incPlanePresent = 0;
  jello->a = 0; jello->b = 0; jello->c = 0; jello->d = 0;
}

// helper: allocate force field
struct point * allocField(int res)
{
  return (struct point *)malloc(res * res * res * sizeof(struct point));
}

void createExplosion()
{
  struct world jello;
  initParams(&jello);
  initCube(&jello);

  // give the cube an initial kick so it's already moving
  for (int i=0; i<=7; i++)
    for (int j=0; j<=7; j++)
      for (int k=0; k<=7; k++)
      {
        jello.v[i][j][k].x = 3.0;
        jello.v[i][j][k].y = 2.0;
        jello.v[i][j][k].z = -4.0;
      }

  jello.resolution = 30;
  jello.forceField = allocField(jello.resolution);

  // oscillating radial force: pushes outward near center, pulls inward far from center
  // creates a "breathing" effect - cube bounces back and forth through space
  double eqRadius = 1.5;  // equilibrium distance from origin
  double strength = 0.08;

  for (int i = 0; i < jello.resolution; i++)
    for (int j = 0; j < jello.resolution; j++)
      for (int k = 0; k < jello.resolution; k++)
      {
        double x = -2 + 4.0 * i / (jello.resolution - 1);
        double y = -2 + 4.0 * j / (jello.resolution - 1);
        double z = -2 + 4.0 * k / (jello.resolution - 1);

        double r = sqrt(x*x + y*y + z*z);
        int idx = i * jello.resolution * jello.resolution + j * jello.resolution + k;

        if (r > 0.01)
        {
          // positive when r < eqRadius (push out), negative when r > eqRadius (pull in)
          double f = strength * (eqRadius - r);
          jello.forceField[idx].x = f * (x / r);
          jello.forceField[idx].y = f * (y / r);
          jello.forceField[idx].z = f * (z / r);
        }
        else
        {
          jello.forceField[idx].x = 0;
          jello.forceField[idx].y = 0;
          jello.forceField[idx].z = 0;
        }

        // light gravity
        jello.forceField[idx].y += -0.015;
      }

  writeWorld("world/explosion.w", &jello);
  free(jello.forceField);
  printf("Created world/explosion.w\n");
}

void createGravityWell()
{
  struct world jello;
  initParams(&jello);

  // offset cube to one side and give tangential velocity for orbit
  for (int i=0; i<=7; i++)
    for (int j=0; j<=7; j++)
      for (int k=0; k<=7; k++)
      {
        // cube centered at (0.8, 0, 0)
        jello.p[i][j][k].x = -0.2 + 2.0 * i / 7;
        jello.p[i][j][k].y = -1.0 + 2.0 * j / 7;
        jello.p[i][j][k].z = -1.0 + 2.0 * k / 7;
        // tangential velocity in z for orbital motion around Y-axis
        jello.v[i][j][k].x = 0.0;
        jello.v[i][j][k].y = 2.0;
        jello.v[i][j][k].z = 5.0;
      }

  jello.resolution = 30;
  jello.forceField = allocField(jello.resolution);

  // gravity well at center - cube orbits around it
  double wellStrength = 0.06;

  for (int i = 0; i < jello.resolution; i++)
    for (int j = 0; j < jello.resolution; j++)
      for (int k = 0; k < jello.resolution; k++)
      {
        double x = -2 + 4.0 * i / (jello.resolution - 1);
        double y = -2 + 4.0 * j / (jello.resolution - 1);
        double z = -2 + 4.0 * k / (jello.resolution - 1);

        double dist = sqrt(x*x + y*y + z*z);
        int idx = i * jello.resolution * jello.resolution + j * jello.resolution + k;

        if (dist > 0.2)
        {
          // attraction toward center, falls off with distance
          double f = wellStrength / (dist * dist);
          if (f > 0.3) f = 0.3;
          jello.forceField[idx].x = f * (-x / dist);
          jello.forceField[idx].y = f * (-y / dist);
          jello.forceField[idx].z = f * (-z / dist);
        }
        else
        {
          jello.forceField[idx].x = 0;
          jello.forceField[idx].y = 0;
          jello.forceField[idx].z = 0;
        }
      }

  writeWorld("world/gravityWell.w", &jello);
  free(jello.forceField);
  printf("Created world/gravityWell.w\n");
}

void createWindShear()
{
  struct world jello;
  initParams(&jello);
  initCube(&jello);

  jello.resolution = 30;
  jello.forceField = allocField(jello.resolution);

  // circular conveyor-belt flow in the XY plane:
  // pushes right on bottom, up on right, left on top, down on left
  // cube rides the current around the box
  double flowStrength = 0.06;

  for (int i = 0; i < jello.resolution; i++)
    for (int j = 0; j < jello.resolution; j++)
      for (int k = 0; k < jello.resolution; k++)
      {
        double x = -2 + 4.0 * i / (jello.resolution - 1);
        double y = -2 + 4.0 * j / (jello.resolution - 1);

        int idx = i * jello.resolution * jello.resolution + j * jello.resolution + k;

        // circular flow: F = (-y, x, 0) pushes things in a loop around the Y=0, X=0 center
        jello.forceField[idx].x = -y * flowStrength;
        jello.forceField[idx].y =  x * flowStrength;
        jello.forceField[idx].z = 0;
      }

  writeWorld("world/windShear.w", &jello);
  free(jello.forceField);
  printf("Created world/windShear.w\n");
}

void createTornado()
{
  struct world jello;
  initParams(&jello);

  // softer springs so the cube visibly deforms as it's twisted and stretched
  jello.kElastic = 300.0;
  jello.dElastic = 0.3;

  // start cube off to one side, near the bottom — it has to get pulled in
  for (int i=0; i<=7; i++)
    for (int j=0; j<=7; j++)
      for (int k=0; k<=7; k++)
      {
        jello.p[i][j][k].x =  0.8 + (-1.0 + 2.0 * i / 7) * 0.5;
        jello.p[i][j][k].y = -1.5 + (2.0 * j / 7) * 0.5;
        jello.p[i][j][k].z =        (-1.0 + 2.0 * k / 7) * 0.5;
        // tangential kick: already moving in the direction the vortex spins
        jello.v[i][j][k].x =  0.0;
        jello.v[i][j][k].y =  2.0;
        jello.v[i][j][k].z = -3.5;
      }

  jello.resolution = 30;
  jello.forceField = allocField(jello.resolution);

  // Rankine vortex parameters
  double eyeRadius  = 0.45;   // core radius: solid-body rotation inside, 1/r outside
  double maxSwirl   = 0.22;   // peak tangential force at the eye wall
  double eyeInward  = 0.09;   // inward suction, strongest at the eye wall
  double updraftPk  = 0.14;   // peak updraft force at the vortex axis
  double gravity    = -0.03;  // mild gravity throughout

  for (int i = 0; i < jello.resolution; i++)
    for (int j = 0; j < jello.resolution; j++)
      for (int k = 0; k < jello.resolution; k++)
      {
        double x = -2 + 4.0 * i / (jello.resolution - 1);
        double y = -2 + 4.0 * j / (jello.resolution - 1);
        double z = -2 + 4.0 * k / (jello.resolution - 1);

        double r = sqrt(x*x + z*z); // radial distance from the Y-axis
        int idx = i * jello.resolution * jello.resolution + j * jello.resolution + k;

        // --- Tangential swirl (Rankine vortex profile) ---
        // Inside the eye: linear with r (solid body, like a spinning top)
        // Outside the eye: falls off as 1/r (free vortex, like water spiraling a drain)
        double swirlSpeed;
        if (r <= eyeRadius)
          swirlSpeed = maxSwirl * (r / eyeRadius);
        else
          swirlSpeed = maxSwirl * (eyeRadius / r);

        // Height modulation: tornado is more intense higher up
        // 0 at bottom wall, 1 at top wall -> swirl ramps from 30% to 100%
        double yNorm = (y + 2.0) / 4.0;
        swirlSpeed *= (0.3 + 0.7 * yNorm);

        double fx = 0, fy = 0, fz = 0;
        if (r > 0.01)
        {
          // counterclockwise in XZ: tangential direction is (-z/r, 0, x/r)
          fx = (-z / r) * swirlSpeed;
          fz = ( x / r) * swirlSpeed;
        }

        // --- Inward suction: bell-shaped, strongest at the eye wall ---
        // This is what drags the cube off-axis into the spiral
        if (r > 0.01)
        {
          double dr = (r - eyeRadius) / (eyeRadius * 0.8);
          double inward = eyeInward * exp(-0.5 * dr * dr);
          fx += (-x / r) * inward;
          fz += (-z / r) * inward;
        }

        // --- Updraft: Gaussian at center, weaker at top ---
        // Cube rises up through the eye, deforms, then spills out over the walls
        double updraft = updraftPk * exp(-(r * r) / (eyeRadius * eyeRadius));
        updraft *= (1.0 - 0.6 * yNorm); // diminishes at top
        fy = updraft + gravity;

        jello.forceField[idx].x = fx;
        jello.forceField[idx].y = fy;
        jello.forceField[idx].z = fz;
      }

  writeWorld("world/tornado.w", &jello);
  free(jello.forceField);
  printf("Created world/tornado.w\n");
}

// Lorenz attractor force field - chaotic butterfly-shaped trajectories
void createChaos()
{
  struct world jello;
  initParams(&jello);
  initCube(&jello);

  // offset cube slightly so it's not perfectly centered
  for (int i=0; i<=7; i++)
    for (int j=0; j<=7; j++)
      for (int k=0; k<=7; k++)
      {
        jello.p[i][j][k].x += 0.3;
        jello.p[i][j][k].y -= 0.5;
        jello.v[i][j][k].x = 2.0;
        jello.v[i][j][k].y = 1.0;
        jello.v[i][j][k].z = -1.5;
      }

  jello.resolution = 30;
  jello.forceField = allocField(jello.resolution);

  // Lorenz-like vector field scaled to [-2,2] box
  double sigma = 10.0, rho = 28.0, beta = 8.0/3.0;
  double scale = 0.0003; // tiny scale since Lorenz values are huge

  for (int i = 0; i < jello.resolution; i++)
    for (int j = 0; j < jello.resolution; j++)
      for (int k = 0; k < jello.resolution; k++)
      {
        double x = -2 + 4.0 * i / (jello.resolution - 1);
        double y = -2 + 4.0 * j / (jello.resolution - 1);
        double z = -2 + 4.0 * k / (jello.resolution - 1);

        // map to Lorenz space (roughly [-20,20] range)
        double lx = x * 10, ly = y * 10, lz = (z + 2) * 7;

        int idx = i * jello.resolution * jello.resolution + j * jello.resolution + k;
        jello.forceField[idx].x = scale * sigma * (ly - lx);
        jello.forceField[idx].y = scale * (lx * (rho - lz) - ly);
        jello.forceField[idx].z = scale * (lx * ly - beta * lz);
      }

  writeWorld("world/chaos.w", &jello);
  free(jello.forceField);
  printf("Created world/chaos.w\n");
}

// Multiple repulsion points - cube bounces between them like a pinball
void createPinball()
{
  struct world jello;
  initParams(&jello);
  initCube(&jello);

  // initial kick
  for (int i=0; i<=7; i++)
    for (int j=0; j<=7; j++)
      for (int k=0; k<=7; k++)
      {
        jello.v[i][j][k].x = 4.0;
        jello.v[i][j][k].y = -2.0;
        jello.v[i][j][k].z = 3.0;
      }

  jello.resolution = 30;
  jello.forceField = allocField(jello.resolution);

  // bumper positions (like pinball bumpers scattered in the box)
  double bumpers[][3] = {
    { 1.5,  1.5,  0.0},
    {-1.5,  1.5,  0.0},
    { 0.0, -1.5,  1.5},
    { 0.0, -1.5, -1.5},
    { 1.5,  0.0,  1.5},
    {-1.5,  0.0, -1.5},
  };
  int nBumpers = 6;
  double repelStrength = 0.04;
  double repelRadius = 1.2;

  for (int i = 0; i < jello.resolution; i++)
    for (int j = 0; j < jello.resolution; j++)
      for (int k = 0; k < jello.resolution; k++)
      {
        double x = -2 + 4.0 * i / (jello.resolution - 1);
        double y = -2 + 4.0 * j / (jello.resolution - 1);
        double z = -2 + 4.0 * k / (jello.resolution - 1);

        int idx = i * jello.resolution * jello.resolution + j * jello.resolution + k;
        jello.forceField[idx].x = 0;
        jello.forceField[idx].y = 0;
        jello.forceField[idx].z = 0;

        for (int b = 0; b < nBumpers; b++)
        {
          double dx = x - bumpers[b][0];
          double dy = y - bumpers[b][1];
          double dz = z - bumpers[b][2];
          double dist = sqrt(dx*dx + dy*dy + dz*dz);

          if (dist < repelRadius && dist > 0.01)
          {
            // strong repulsion that falls off with distance
            double f = repelStrength * (repelRadius - dist) / (dist * dist);
            jello.forceField[idx].x += f * (dx / dist);
            jello.forceField[idx].y += f * (dy / dist);
            jello.forceField[idx].z += f * (dz / dist);
          }
        }

        // light gravity to keep things moving
        jello.forceField[idx].y += -0.01;
      }

  writeWorld("world/pinball.w", &jello);
  free(jello.forceField);
  printf("Created world/pinball.w\n");
}

// Counter-rotating layers - top spins one way, bottom the other, cube tumbles
void createWashingMachine()
{
  struct world jello;
  initParams(&jello);
  initCube(&jello);

  // initial upward velocity so it crosses between layers
  for (int i=0; i<=7; i++)
    for (int j=0; j<=7; j++)
      for (int k=0; k<=7; k++)
      {
        jello.v[i][j][k].y = 3.0;
        jello.v[i][j][k].x = 1.0;
      }

  jello.resolution = 30;
  jello.forceField = allocField(jello.resolution);

  double spinStrength = 0.08;

  for (int i = 0; i < jello.resolution; i++)
    for (int j = 0; j < jello.resolution; j++)
      for (int k = 0; k < jello.resolution; k++)
      {
        double x = -2 + 4.0 * i / (jello.resolution - 1);
        double y = -2 + 4.0 * j / (jello.resolution - 1);
        double z = -2 + 4.0 * k / (jello.resolution - 1);

        int idx = i * jello.resolution * jello.resolution + j * jello.resolution + k;

        // rotation direction flips based on height
        // y > 0: spin clockwise in XZ,  y < 0: spin counter-clockwise
        double dir = (y > 0) ? 1.0 : -1.0;

        // smooth transition near y=0
        double blend = y / 0.5;
        if (blend > 1.0) blend = 1.0;
        if (blend < -1.0) blend = -1.0;

        jello.forceField[idx].x = -z * spinStrength * blend;
        jello.forceField[idx].z =  x * spinStrength * blend;

        // vertical force pushes cube toward y=0 boundary so it keeps crossing
        jello.forceField[idx].y = -0.03 * y;
      }

  writeWorld("world/washingMachine.w", &jello);
  free(jello.forceField);
  printf("Created world/washingMachine.w\n");
}

// Spiraling corkscrew - swirl + upward thrust, falls back, repeats
void createHelix()
{
  struct world jello;
  initParams(&jello);

  // start cube low
  for (int i=0; i<=7; i++)
    for (int j=0; j<=7; j++)
      for (int k=0; k<=7; k++)
      {
        jello.p[i][j][k].x = -1.0 + 2.0 * i / 7;
        jello.p[i][j][k].y = -1.8 + 2.0 * j / 7;  // near bottom
        jello.p[i][j][k].z = -1.0 + 2.0 * k / 7;
        jello.v[i][j][k].x = 0;
        jello.v[i][j][k].y = 4.0;  // initial upward kick
        jello.v[i][j][k].z = 0;
      }

  jello.resolution = 30;
  jello.forceField = allocField(jello.resolution);

  double swirlStrength = 0.10;
  double liftStrength = 0.06;
  double gravity = -0.025;

  for (int i = 0; i < jello.resolution; i++)
    for (int j = 0; j < jello.resolution; j++)
      for (int k = 0; k < jello.resolution; k++)
      {
        double x = -2 + 4.0 * i / (jello.resolution - 1);
        double y = -2 + 4.0 * j / (jello.resolution - 1);
        double z = -2 + 4.0 * k / (jello.resolution - 1);

        double r = sqrt(x*x + z*z);
        int idx = i * jello.resolution * jello.resolution + j * jello.resolution + k;

        // strong swirl in XZ
        jello.forceField[idx].x = -z * swirlStrength;
        jello.forceField[idx].z =  x * swirlStrength;

        // upward lift that's stronger near the bottom, weaker at top
        // creates the corkscrew rise, then cube falls from top
        double yNorm = (y + 2.0) / 4.0; // 0 at bottom, 1 at top
        jello.forceField[idx].y = liftStrength * (1.0 - 2.0 * yNorm) + gravity;

        // inward pull to keep it spiraling tight
        if (r > 0.01)
        {
          jello.forceField[idx].x += -0.02 * (x / r);
          jello.forceField[idx].z += -0.02 * (z / r);
        }
      }

  writeWorld("world/helix.w", &jello);
  free(jello.forceField);
  printf("Created world/helix.w\n");
}

// Tidal forces - stretches cube along one axis, compresses along others
// Axis of stretch rotates through space so cube tumbles and deforms
void createTidal()
{
  struct world jello;
  initParams(&jello);
  jello.kElastic = 300.0;  // softer springs so deformation is more visible
  initCube(&jello);

  // give it a slow tumble
  for (int i=0; i<=7; i++)
    for (int j=0; j<=7; j++)
      for (int k=0; k<=7; k++)
      {
        jello.v[i][j][k].x = 2.0;
        jello.v[i][j][k].y = 3.0;
        jello.v[i][j][k].z = -2.5;
      }

  jello.resolution = 30;
  jello.forceField = allocField(jello.resolution);

  for (int i = 0; i < jello.resolution; i++)
    for (int j = 0; j < jello.resolution; j++)
      for (int k = 0; k < jello.resolution; k++)
      {
        double x = -2 + 4.0 * i / (jello.resolution - 1);
        double y = -2 + 4.0 * j / (jello.resolution - 1);
        double z = -2 + 4.0 * k / (jello.resolution - 1);

        int idx = i * jello.resolution * jello.resolution + j * jello.resolution + k;

        // stretch along diagonal (1,1,0), compress along (-1,1,0) and z
        // the stretch axis rotates based on z-position, creating twist
        double angle = z * 0.8; // rotation angle depends on z
        double cosA = cos(angle), sinA = sin(angle);

        // rotated tidal tensor in XY plane
        double strength = 0.04;
        double tx = x * cosA + y * sinA;   // component along stretch axis
        double ty = -x * sinA + y * cosA;  // component along compress axis

        // stretch along tx, compress along ty
        double fx_rot = strength * tx;
        double fy_rot = -strength * ty * 0.5;

        // rotate back
        jello.forceField[idx].x = fx_rot * cosA - fy_rot * sinA;
        jello.forceField[idx].y = fx_rot * sinA + fy_rot * cosA;
        jello.forceField[idx].z = -strength * z * 0.3; // mild compression in z

        // gentle pull toward center so it doesn't drift to a wall
        double dist = sqrt(x*x + y*y + z*z);
        if (dist > 1.0)
        {
          double pull = -0.01 * (dist - 1.0);
          jello.forceField[idx].x += pull * x / dist;
          jello.forceField[idx].y += pull * y / dist;
          jello.forceField[idx].z += pull * z / dist;
        }
      }

  writeWorld("world/tidal.w", &jello);
  free(jello.forceField);
  printf("Created world/tidal.w\n");
}

int main()
{
  createExplosion();
  createGravityWell();
  createWindShear();
  createTornado();
  createChaos();
  createPinball();
  createWashingMachine();
  createHelix();
  createTidal();
  return 0;
}

