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
int main()
{
  createTornado();
  return 0;
}

