/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"

void computeForceHooksLaw(struct point *pA, struct point *vA, struct point *pB, struct point *vB,
                          double kE, double dE, double rest_length, struct point *forceOut)
{
  struct point L, vDiff;
  pDIFFERENCE(*pB, *pA, L); // L is the vector pointing from A -> B
  // calculate magnitude using 3d euclidean distance
  double len = sqrt(L.x*L.x + L.y*L.y + L.z*L.z); 
  if (len == 0) return;

  struct point dir;
  pMULTIPLY(L, 1.0/len, dir); // dividing vector by magnitude gives its direction

  struct point fHook;
  pMULTIPLY(dir, kE * (len - rest_length), fHook); // f = k * x

  // Damping: -d * dot(vA - vB, dir) * dir
  pDIFFERENCE(*vA, *vB, vDiff);
  double dot = vDiff.x*dir.x + vDiff.y*dir.y + vDiff.z*dir.z;
  struct point fDamp;
  pMULTIPLY(dir, -dE * dot, fDamp);

  // Accumulate
  forceOut->x += fHook.x + fDamp.x;
  forceOut->y += fHook.y + fDamp.y;
  forceOut->z += fHook.z + fDamp.z;
}

/* Computes acceleration to every control point of the jello cube (512 points), 
   which is in state given by 'jello'.

   Takes into account the forces due to
    - structural, shear and bend springs,
    - external forces (force field, if any), and
    - bouncing off the walls. 
    
    Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
  /*compute force on each of the points
      - force comes from 
        - structural, shear and bend springs,
        - external forces (force field, if any), and
        - bouncing off the walls.*/
  int n = 8;
  for (int i = 0; i < n; i++){
    for (int j = 0; j < n; j++){
      for (int k = 0; k < n; k++){
        struct point force;
        pMAKE(0, 0 ,0, force); // set force in each direction to 0

        int structural_neighbors[6][3] = { // structural sprints connect every node to its 6 neighbors
          {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
        };
        
        for (int s = 0; s < 6; s++){
          // compute each neighbors index
          int ni = i + structural_neighbors[s][0];
          int nj = j + structural_neighbors[s][1];
          int nk = k + structural_neighbors[s][2];
          // make sure the neighbor exists
          if (ni >= 0 && ni < 8 && nj >= 0 && nj < 8 && nk >= 0 && nk < 8){
            double spacing = 2.0 / 7.0; // distance between adjacent grid points along one axis
            computeForceHooksLaw(&jello->p[i][j][k], &jello->v[i][j][k], &jello->p[ni][nj][nk],
              &jello->v[ni][nj][nk], jello->kElastic, jello->dElastic, spacing, &force);
          }
        }
        // calculate shear force (diagonal neighbors) - there are 20
        int shear_neighbors[20][3] = {
          // 2D diagonals in ij plane
          {1,1,0},{-1,1,0},{1,-1,0},{-1,-1,0},
          // 2D diagonals in ik plane
          {1,0,1},{-1,0,1},{1,0,-1},{-1,0,-1},
          // 2D diagonals in jk plane
          {0,1,1},{0,-1,1},{0,1,-1},{0,-1,-1},
          // 3D diagonals
          {1,1,1},{-1,1,1},{1,-1,1},{-1,-1,1},
          {1,1,-1},{-1,1,-1},{1,-1,-1},{-1,-1,-1}
        };
        for (int s = 0; s < 20; s++){
          // compute neighbor indices
          int ni = i + shear_neighbors[s][0];
          int nj = j + shear_neighbors[s][1];
          int nk = k + shear_neighbors[s][2];
          if (ni >= 0 && ni < 8 && nj >= 0 && nj < 8 && nk >= 0 && nk < 8){
            double spacing = 2.0 / 7.0;
            // pythagorean theorem
            double restLen = spacing * sqrt((double)(
              shear_neighbors[s][0]*shear_neighbors[s][0] +
              shear_neighbors[s][1]*shear_neighbors[s][1] +
              shear_neighbors[s][2]*shear_neighbors[s][2]));
            computeForceHooksLaw(&jello->p[i][j][k], &jello->v[i][j][k], &jello->p[ni][nj][nk],
              &jello->v[ni][nj][nk], jello->kElastic, jello->dElastic, restLen, &force);
          }
        }

        // calculate bend force (each node connected to its second neighbor)
        int bend_neighbors[6][3] = {
          {2,0,0},{-2,0,0},{0,2,0},{0,-2,0},{0,0,2},{0,0,-2}
        };
        for (int s = 0; s < 6; s++){
          int ni = i + bend_neighbors[s][0];
          int nj = j + bend_neighbors[s][1];
          int nk = k + bend_neighbors[s][2];
          if (ni >= 0 && ni < 8 && nj >= 0 && nj < 8 && nk >= 0 && nk < 8){
            double spacing = 2.0 / 7.0;
            computeForceHooksLaw(&jello->p[i][j][k], &jello->v[i][j][k], &jello->p[ni][nj][nk],
              &jello->v[ni][nj][nk], jello->kElastic, jello->dElastic, 2.0 * spacing, &force);
          }
        }

        // calculate external forces (trilinear interpolation of force field)
        if (jello->resolution > 0) {
          struct point pos = jello->p[i][j][k];
          int res = jello->resolution;

          // map position [-2,2] to grid index [0, res-1]
          double gx = (pos.x + 2.0) / 4.0 * (res - 1);
          double gy = (pos.y + 2.0) / 4.0 * (res - 1);
          double gz = (pos.z + 2.0) / 4.0 * (res - 1);

          // clamp to valid range
          if (gx < 0) gx = 0; if (gx > res - 1) gx = res - 1;
          if (gy < 0) gy = 0; if (gy > res - 1) gy = res - 1;
          if (gz < 0) gz = 0; if (gz > res - 1) gz = res - 1;

          // find the 8 surrounding grid points
          int x0 = (int)gx, y0 = (int)gy, z0 = (int)gz;
          int x1 = x0 + 1, y1 = y0 + 1, z1 = z0 + 1;
          if (x1 > res - 1) x1 = res - 1;
          if (y1 > res - 1) y1 = res - 1;
          if (z1 > res - 1) z1 = res - 1;

          // fractional position within the cell (0 = at low corner, 1 = at high corner)
          double fx = gx - x0, fy = gy - y0, fz = gz - z0;

          // look up force at each of the 8 corners
          #define FF(xi,yi,zi) jello->forceField[(xi)*res*res + (yi)*res + (zi)]
          struct point c000 = FF(x0,y0,z0), c100 = FF(x1,y0,z0);
          struct point c010 = FF(x0,y1,z0), c110 = FF(x1,y1,z0);
          struct point c001 = FF(x0,y0,z1), c101 = FF(x1,y0,z1);
          struct point c011 = FF(x0,y1,z1), c111 = FF(x1,y1,z1);
          #undef FF

          // blend the 8 corners based on how close the point is to each one
          struct point f;
          f.x = (1-fx)*(1-fy)*(1-fz)*c000.x + fx*(1-fy)*(1-fz)*c100.x
              + (1-fx)*fy*(1-fz)*c010.x     + fx*fy*(1-fz)*c110.x
              + (1-fx)*(1-fy)*fz*c001.x      + fx*(1-fy)*fz*c101.x
              + (1-fx)*fy*fz*c011.x           + fx*fy*fz*c111.x;
          f.y = (1-fx)*(1-fy)*(1-fz)*c000.y + fx*(1-fy)*(1-fz)*c100.y
              + (1-fx)*fy*(1-fz)*c010.y     + fx*fy*(1-fz)*c110.y
              + (1-fx)*(1-fy)*fz*c001.y      + fx*(1-fy)*fz*c101.y
              + (1-fx)*fy*fz*c011.y           + fx*fy*fz*c111.y;
          f.z = (1-fx)*(1-fy)*(1-fz)*c000.z + fx*(1-fy)*(1-fz)*c100.z
              + (1-fx)*fy*(1-fz)*c010.z     + fx*fy*(1-fz)*c110.z
              + (1-fx)*(1-fy)*fz*c001.z      + fx*(1-fy)*fz*c101.z
              + (1-fx)*fy*fz*c011.z           + fx*fy*fz*c111.z;

          pSUM(force, f, force);
        }

        // calculate bouncing forces (penalty springs at walls ±2)
        struct point pos = jello->p[i][j][k];
        struct point vel = jello->v[i][j][k];
        double kC = jello->kCollision, dC = jello->dCollision;

        // if the jello goes past a wall it gets pushed back
        if (pos.x > 2.0)  { force.x += -kC * (pos.x - 2.0) - dC * vel.x; }
        if (pos.x < -2.0) { force.x += -kC * (pos.x + 2.0) - dC * vel.x; }
        if (pos.y > 2.0)  { force.y += -kC * (pos.y - 2.0) - dC * vel.y; }
        if (pos.y < -2.0) { force.y += -kC * (pos.y + 2.0) - dC * vel.y; }
        if (pos.z > 2.0)  { force.z += -kC * (pos.z - 2.0) - dC * vel.z; }
        if (pos.z < -2.0) { force.z += -kC * (pos.z + 2.0) - dC * vel.z; }

        // store total force/ mass in points array
        pMULTIPLY(force, 1.0 / jello->mass, a[i][j][k]);
      }
    }
  }
  
}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a);
  
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
        jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
        jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
        jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
        jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

      }
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello

  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],1.0,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],1.0,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a);


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }

  return;  
}
