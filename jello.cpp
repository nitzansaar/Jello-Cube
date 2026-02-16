/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

  Your name:
  Nitzan Saar

*/

#include "jello.h"
#include "showCube.h"
#include "input.h"
#include "physics.h"

// camera parameters
double Theta = pi / 6;
double Phi = pi / 6;
double R = 6;

// mouse control
int g_iMenuId;
int g_vMousePos[2];
int g_iLeftMouseButton,g_iMiddleMouseButton,g_iRightMouseButton;

// number of images saved to disk so far
int sprite=0;

// these variables control what is displayed on screen
int shear=0, bend=0, structural=1, pause=0, viewingMode=0, saveScreenToFile=0;

struct world jello;
struct world jello2;

int windowWidth, windowHeight;

void myinit()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(90.0,1.0,0.01,1000.0);

  // set background color to soft light grey
  glClearColor(0.75, 0.75, 0.78, 0.0);

  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_POLYGON_SMOOTH);
  glEnable(GL_LINE_SMOOTH);

  return; 
}

void reshape(int w, int h) 
{
  // Prevent a divide by zero, when h is zero.
  // You can't make a window of zero height.
  if(h == 0)
    h = 1;

  glViewport(0, 0, w, h);

  // Reset the coordinate system before modifying
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // Set the perspective
  double aspectRatio = 1.0 * w / h;
  gluPerspective(60.0f, aspectRatio, 0.01f, 1000.0f);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity(); 

  windowWidth = w;
  windowHeight = h;

  glutPostRedisplay();
}

void display()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // camera parameters are Phi, Theta, R
  gluLookAt(R * cos(Phi) * cos (Theta), R * sin(Phi) * cos (Theta), R * sin (Theta),
	        0.0,0.0,0.0, 0.0,0.0,1.0);


  /* Lighting */
  /* 3-point lighting setup: key, fill, and rim lights for dramatic,
     professional illumination of the jello cube. */

  // global ambient light
  GLfloat aGa[] = { 0.15, 0.2, 0.3, 1.0 };

  // Key light (GL_LIGHT0): warm white, upper-right-front
  GLfloat lKa0[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd0[] = { 1.0, 0.95, 0.85, 1.0 };
  GLfloat lKs0[] = { 1.0, 0.95, 0.85, 1.0 };
  GLfloat lP0[]  = { 3.0, 4.0, 5.0, 1.0 };

  // Fill light (GL_LIGHT1): cool blue, lower-left
  GLfloat lKa1[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd1[] = { 0.3, 0.4, 0.6, 1.0 };
  GLfloat lKs1[] = { 0.3, 0.4, 0.6, 1.0 };
  GLfloat lP1[]  = { -4.0, -2.0, 0.0, 1.0 };

  // Rim light (GL_LIGHT2): pale white, behind
  GLfloat lKa2[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd2[] = { 0.6, 0.6, 0.7, 1.0 };
  GLfloat lKs2[] = { 0.6, 0.6, 0.7, 1.0 };
  GLfloat lP2[]  = { 0.0, -3.0, -4.0, 1.0 };

  // jelly material color — light blue translucent jello
  GLfloat mKa[] = { 0.2, 0.35, 0.5, 1.0 };
  GLfloat mKd[] = { 0.5, 0.75, 1.0, 0.85 };
  GLfloat mKs[] = { 0.9, 0.95, 1.0, 1.0 };
  GLfloat mKe[] = { 0.0, 0.0, 0.0, 1.0 };

  /* set up lighting */
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, aGa);
  glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

  // set up cube material (both sides for two-sided lighting)
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mKa);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mKd);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mKs);
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, mKe);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 80);

  // Key light setup
  glLightfv(GL_LIGHT0, GL_POSITION, lP0);
  glLightfv(GL_LIGHT0, GL_AMBIENT, lKa0);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lKd0);
  glLightfv(GL_LIGHT0, GL_SPECULAR, lKs0);
  glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 1.0);
  glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.05);
  glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.01);
  glEnable(GL_LIGHT0);

  // Fill light setup
  glLightfv(GL_LIGHT1, GL_POSITION, lP1);
  glLightfv(GL_LIGHT1, GL_AMBIENT, lKa1);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, lKd1);
  glLightfv(GL_LIGHT1, GL_SPECULAR, lKs1);
  glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 1.0);
  glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, 0.05);
  glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, 0.01);
  glEnable(GL_LIGHT1);

  // Rim light setup
  glLightfv(GL_LIGHT2, GL_POSITION, lP2);
  glLightfv(GL_LIGHT2, GL_AMBIENT, lKa2);
  glLightfv(GL_LIGHT2, GL_DIFFUSE, lKd2);
  glLightfv(GL_LIGHT2, GL_SPECULAR, lKs2);
  glEnable(GL_LIGHT2);

  // Disable unused lights
  glDisable(GL_LIGHT3);
  glDisable(GL_LIGHT4);
  glDisable(GL_LIGHT5);
  glDisable(GL_LIGHT6);
  glDisable(GL_LIGHT7);

  // enable lighting
  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);

  // enable blending for translucent jello
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // show cube 1 (blue)
  showCube(&jello);

  // show cube 2 (warm orange/red)
  {
    GLfloat mKa2[] = { 0.5, 0.25, 0.1, 1.0 };
    GLfloat mKd2[] = { 1.0, 0.5, 0.2, 0.85 };
    GLfloat mKs2[] = { 1.0, 0.8, 0.6, 1.0 };
    GLfloat mKe2[] = { 0.0, 0.0, 0.0, 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mKa2);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mKd2);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mKs2);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, mKe2);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 80);
    showCube(&jello2);
  }

  glDisable(GL_BLEND);
  glDisable(GL_LIGHTING);

  // show the bounding box
  showBoundingBox();
 
  glutSwapBuffers();
}

void doIdle()
{
  char s[20]="picxxxx.ppm";
  int i;
  
  // save screen to file
  s[3] = 48 + (sprite / 1000);
  s[4] = 48 + (sprite % 1000) / 100;
  s[5] = 48 + (sprite % 100 ) / 10;
  s[6] = 48 + sprite % 10;

  if (saveScreenToFile==1)
  {
    saveScreenshot(windowWidth, windowHeight, s);
    saveScreenToFile=0; // save only once, change this if you want continuos image generation (i.e. animation)
    sprite++;
  }

  if (sprite >= 300) // allow only 300 snapshots
  {
    exit(0);	
  }

  if (pause == 0)
  {
    // take snapshots for frozen-other integration
    struct world jello_snap = jello;
    struct world jello2_snap = jello2;

    if (strcmp(jello.integrator, "RK4") == 0)
    {
      for (int i = 0; i < jello.n; i++) {
        RK4(&jello, &jello2_snap);
        RK4(&jello2, &jello_snap);
      }
    }
    else
    {
      for (int i = 0; i < jello.n; i++) {
        Euler(&jello, &jello2_snap);
        Euler(&jello2, &jello_snap);
      }
    }
  }

  glutPostRedisplay();
}

int main (int argc, char ** argv)
{
  if (argc<2)
  {  
    printf ("Oops! You didn't say the jello world file!\n");
    printf ("Usage: %s [worldfile]\n", argv[0]);
    exit(0);
  }

  readWorld(argv[1],&jello);

  // Scale cube 1 positions to half size
  for (int i = 0; i < 8; i++)
    for (int j = 0; j < 8; j++)
      for (int k = 0; k < 8; k++)
      {
        jello.p[i][j][k].x *= 0.5;
        jello.p[i][j][k].y *= 0.5;
        jello.p[i][j][k].z *= 0.5;
      }

  // Initialize jello2: copy physics params, mirror positions & velocities
  memset(&jello2, 0, sizeof(jello2));
  strcpy(jello2.integrator, jello.integrator);
  jello2.dt = jello.dt;
  jello2.n = jello.n;
  jello2.kElastic = jello.kElastic;
  jello2.dElastic = jello.dElastic;
  jello2.kCollision = jello.kCollision;
  jello2.dCollision = jello.dCollision;
  jello2.mass = jello.mass;
  jello2.incPlanePresent = 0;
  jello2.resolution = 0;
  jello2.forceField = NULL;

  for (int i = 0; i < 8; i++)
    for (int j = 0; j < 8; j++)
      for (int k = 0; k < 8; k++)
      {
        // mirror across origin: negate x and y
        jello2.p[i][j][k].x = -jello.p[i][j][k].x;
        jello2.p[i][j][k].y = -jello.p[i][j][k].y;
        jello2.p[i][j][k].z =  jello.p[i][j][k].z;
        // negate x,y velocity so cubes move toward each other
        jello2.v[i][j][k].x = -jello.v[i][j][k].x;
        jello2.v[i][j][k].y = -jello.v[i][j][k].y;
        jello2.v[i][j][k].z =  jello.v[i][j][k].z;
      }

  glutInit(&argc,argv);
  
  /* double buffered window, use depth testing, 640x480 */
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  
  windowWidth = 640;
  windowHeight = 480;
  glutInitWindowSize (windowWidth, windowHeight);
  glutInitWindowPosition (0,0);
  glutCreateWindow ("Jello cube");

  /* tells glut to use a particular display function to redraw */
  glutDisplayFunc(display);

  /* replace with any animate code */
  glutIdleFunc(doIdle);

  /* callback for mouse drags */
  glutMotionFunc(mouseMotionDrag);

  /* callback for window size changes */
  glutReshapeFunc(reshape);

  /* callback for mouse movement */
  glutPassiveMotionFunc(mouseMotion);

  /* callback for mouse button changes */
  glutMouseFunc(mouseButton);

  /* register for keyboard events */
  glutKeyboardFunc(keyboardFunc);

  /* do initialization */
  myinit();

  /* forever sink in the black hole */
  glutMainLoop(); 

  return(0);
}

