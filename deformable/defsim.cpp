#include <GL/glfw.h>
#include <cv.h>
#include <highgui.h>
#include "FEM.h"
#include "CollisionData.h"
#include "DeformableLink.h"

#if 0
#include <hrpModel/ConstraintForceSolver.h>
#else
#include "WorldExt.h"
#include "ConstraintForceSolverExt.h"
#endif
#include <hrpModel/ColdetLinkPair.h>
#include <hrpCorba/OpenHRPCommon.hh>
#include <hrpCollision/CollisionData.h>

#include <unistd.h>

#include <iostream>

using namespace afstate;
using namespace std;
using namespace boost;
using namespace hrp;

#define DEFAULT_W 640
#define DEFAULT_H 480
#define DEFAULT_FPS 20
#define TIMESTEP 0.001

// view point control
double panAngle=M_PI/4;
double tiltAngle=M_PI/4;
int prevX, prevY, prevW, prevLeftButtonState;
double radius = 5; 

// movie
CvVideoWriter *videoWriter;
IplImage *image;

bool quit_flag=false, start_flag = false, movie_flag = false;

void keyboard(int key, int action)
{
    //printf("keyboard(%d)\n", key);
    if (action == GLFW_RELEASE) return;

    switch(key){
    case 'Q':
        quit_flag = true;
        break;
    case 'M':
        movie_flag = true;
	break;
    case 'S':
        start_flag = !start_flag;
        break;
    default:
        break;
    }
}

void detectCollision(const std::vector<ColdetLinkPairPtr>& i_pairs,
                     std::vector<CollisionDataSequence>& o_cd)
{
    for (unsigned int j=0; j<i_pairs.size(); j++){
        // TODO : generalize
        DeformableLink *dl = dynamic_cast<DeformableLink *>(i_pairs[j]->link(1));
        if (!dl) continue; 
        MatrixXd& nodes = dl->getNodes();
        
        Vector3 n(0,0,1);
        VectorXi& indexDefNode = dl->getIndexDefNode();
        
        o_cd[0].clear();
        for (int i=0; i<nodes.rows(); i++){
            int idx = indexDefNode(i);
            if (idx < 0 || idx*3 >= dl->getSizeF()) continue;
            Vector3 lp(nodes(i,0), nodes(i,1), nodes(i,2));
            Vector3 gp(dl->p + dl->R*lp);
            if (gp[2] < 0 && gp[0] < 0.5) {
                double depth = -gp[2];
                gp[2] = 0;
                o_cd[0].push_back(CollisionData(gp, n, depth, i, -1));
                //std::cout << i << "(" << idx << ") ";
            }
        }
        //std::cout << std::endl;
    }
}

template<class T>
void stepSimulation(
    WorldExt<T>& refWorld,
    const std::vector<ColdetLinkPairPtr>& i_pairs,
    std::vector<CollisionDataSequence>& io_cd)
{
    refWorld.constraintForceSolver.clearExternalForces();

    // compute forward dynamics
    refWorld.calcNextState(io_cd);

    // collision detection
    detectCollision(i_pairs, io_cd);
}

void drawAxes()
{
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);

    glColor3f(1,0,0);
    glVertex3f(0,0,0);
    glVertex3f(2,0,0);
    
    glColor3f(0,1,0);
    glVertex3f(0,0,0);
    glVertex3f(0,2,0);
    
    glColor3f(0,0,1);
    glVertex3f(0,0,0);
    glVertex3f(0,0,2);

    glEnd();
    glEnable(GL_LIGHTING);
}

void initGL()
{
    glfwInit();
    glfwOpenWindow(DEFAULT_W,DEFAULT_H,0,0,0,0,24,0, GLFW_WINDOW);
    GLfloat light0pos[] = { 0.0, 4.0, 6.0, 1.0 };
    GLfloat light1pos[] = { 6.0, 4.0, 0.0, 1.0 };
    GLfloat white[] = { 0.6, 0.6, 0.6, 1.0 };
    glfwSetKeyCallback(keyboard);
    glfwEnable(GLFW_KEY_REPEAT);
    prevLeftButtonState = glfwGetMouseButton(GLFW_MOUSE_BUTTON_LEFT);
    prevW = glfwGetMouseWheel();
    glfwGetMousePos(&prevX, &prevY);
    
    glClearColor(0.8, 0.8, 1.0, 1.0);
    
    glEnable(GL_DEPTH_TEST);
    
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT1, GL_SPECULAR, white);
    glLightfv(GL_LIGHT0, GL_POSITION, light0pos);
    glLightfv(GL_LIGHT1, GL_POSITION, light1pos);
    
}

BodyPtr createFloor()
{
    Link *root = new Link;
    root->b = 0;
    root->Rs = 1,0,0,0,1,0,0,0,1;
    
    BodyPtr body = new Body();
    body->setRootLink(root);
    body->setDefaultRootPosition(root->b, root->Rs);
    body->installCustomizer();
    body->initializeConfiguration();
    body->setName("floor");

    return body;
}

BodyPtr createCube()
{
    // create a deformable body
    //FEM *cube_ = new GMSH("mid_cube.inp");
    FEM *cube_ = new GMSH("cylinder.inp");
    cube_->assembleK();
    cube_->sortK();
    cube_->sortT();
    cube_->computeInverseForceCondensedK();
    cube_->computeDisplacementCondensedK();
    
    DeformableLink *root = new DeformableLink(*cube_);
    root->isHighGainMode = true;

    BodyPtr body = new Body();
    body->setRootLink(root);
    body->setDefaultRootPosition(root->b, root->Rs);
    body->installCustomizer();
    body->initializeConfiguration();
    body->setName("cube");

    return body;
}

void initMovie()
{
    videoWriter = cvCreateVideoWriter(
	"defsim.avi",
	CV_FOURCC('D','I','V','X'),
	DEFAULT_FPS,
	cvSize(DEFAULT_W, DEFAULT_H));
    image = cvCreateImage(
	cvSize(DEFAULT_W, DEFAULT_H),
	IPL_DEPTH_8U, 3);
}

void addMovieFrame()
{
    unsigned char rgb[DEFAULT_W*DEFAULT_H*3];
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    for (int i=0; i<DEFAULT_H; i++){
	glReadPixels(0,(DEFAULT_H-1-i),DEFAULT_W,
		     1,GL_RGB,GL_UNSIGNED_BYTE,
		     rgb + i*3*DEFAULT_W);
    }
    char *bgr = image->imageData;
    for (int i=0; i<DEFAULT_W*DEFAULT_H; i++){
	bgr[i*3  ] = rgb[i*3+2]; 
	bgr[i*3+1] = rgb[i*3+1]; 
	bgr[i*3+2] = rgb[i*3  ]; 
    }
    cvWriteFrame(videoWriter, image);
}

void finishMovie()
{
    cvReleaseVideoWriter(&videoWriter);
    cvReleaseImage(&image);
}

void drawConstraintForces(BodyPtr body)
{
    Link *root = body->rootLink();
 
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor3f(1,1,0);
    for (unsigned int i=0; i<root->constraintForces.size(); i++){
	Link::ConstraintForce &cf = root->constraintForces[i];
	glVertex3d(cf.point[0], cf.point[1], cf.point[2]);
#define FSCALE 0.01
	glVertex3d(cf.point[0] + cf.force[0]*FSCALE,
		   cf.point[1] + cf.force[1]*FSCALE,
		   cf.point[2] + cf.force[2]*FSCALE);
        //std::cout << i << ":" << cf.force << std::endl;
    }
    glEnd();
    glEnable(GL_LIGHTING);
}

int main(int argc, char* argv[])
{
    //hrp::World<hrp::ConstraintForceSolver> world;
    WorldExt<hrp::ConstraintForceSolverExt> world;
    
    world.setCurrentTime(0.0);
    world.setTimeStep(TIMESTEP);
    world.setRungeKuttaMethod();
    
    BodyPtr cube = createCube();
    BodyPtr floor = createFloor();

    world.addBody(cube);
    world.addBody(floor);

    world.enableSensors(false);
    
    // create collision check pairs
    std::vector<ColdetLinkPairPtr> pairs;
    pairs.push_back(new hrp::ColdetLinkPair(floor->rootLink(),
                                            cube->rootLink()));
    std::vector<CollisionDataSequence> collisions;
    collisions.resize(pairs.size());
    
    for (unsigned int i=0; i<pairs.size(); i++){
        world.constraintForceSolver.addCollisionCheckLinkPair
            (world.bodyIndex(pairs[i]->link(0)->body->name()), 
             pairs[i]->link(0),
             world.bodyIndex(pairs[i]->link(1)->body->name()), 
             pairs[i]->link(1),
             0.5, 0.5, 0.0, 0.0);
    }
    
    int nBodies = world.numBodies();
    for(int i=0; i < nBodies; ++i){
        hrp::BodyPtr bodyPtr = world.body(i);
        bodyPtr->initializeConfiguration();
    }
    
    world.initialize();
    world.constraintForceSolver.enableConstraintForceOutput(true);

#if 1
#define INIT_Z 0.7
    cube->rootLink()->p[2] = INIT_Z;
    //cube->rootLink()->R = rotFromRpy(0.2,0.2,0);
#endif
    
    int movie_step = 1.0/TIMESTEP/DEFAULT_FPS;

    initGL();

    // Simulation loop
    while(!quit_flag){
        if (start_flag || movie_flag) {
	    if (movie_flag && world.currentTime() == 0.0){
		initMovie();
	    }
#if 1 // high-gain control
#define polynomial3(t)   ((3.0 - 2.0*(t))*(t)*(t))
#define dpolynomial3(t)  ((6.0 - 6.0*(t))*(t))
#define ddpolynomial3(t) (6.0 - 12.0*(t))
            Link *link = cube->rootLink(); 
#define DURATION 3.0
            if (world.currentTime() < DURATION){
#define AMP 0.4
                link->p(2) = INIT_Z - AMP*polynomial3(world.currentTime()/DURATION);
                link->v(2) = - AMP*dpolynomial3(world.currentTime()/DURATION);
                link->dv(2) = - AMP*ddpolynomial3(world.currentTime()/DURATION);
            }else if (world.currentTime() < DURATION*2){
		double t = world.currentTime()-DURATION;
                link->p(2) = INIT_Z - AMP + AMP*polynomial3(t/DURATION);
                link->v(2) =  AMP*dpolynomial3(t/DURATION);
                link->dv(2) =  AMP*ddpolynomial3(t/DURATION);
	    }else{
                link->p(2) = INIT_Z;
                link->v(2) = 0;
                link->dv(2) = 0;
            }
#endif
	    stepSimulation( world, pairs, collisions );
            //start_flag = false;
	}
	printf("\rt = %5.3f", world.currentTime());
	fflush(stdout);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(30, 
                       (double)DEFAULT_W / (double)DEFAULT_H, 
                       0.1, 100);
        double eye[3];
	int x,y,w,state;
	glfwGetMousePos(&x, &y);
	w = glfwGetMouseWheel();
	state = glfwGetMouseButton(GLFW_MOUSE_BUTTON_LEFT);
	if (state == GLFW_PRESS && prevLeftButtonState == GLFW_PRESS){
	  panAngle  -= (x - prevX)*0.01;
	  tiltAngle += (y - prevY)*0.01;
	  if (tiltAngle < -M_PI/2) tiltAngle = -M_PI/2;
	  if (tiltAngle >  M_PI/2) tiltAngle =  M_PI/2;
	}
	radius *= 1.0 + (w - prevW)*0.1;
	prevLeftButtonState = state;
	prevX = x;
	prevY = y;
	prevW = w;
	
        eye[0] = radius*cos(tiltAngle)*cos(panAngle)+0.5;
        eye[1] = radius*cos(tiltAngle)*sin(panAngle)+0.5;
        eye[2] = radius*sin(tiltAngle)+0.5;
        gluLookAt(eye[0], eye[1], eye[2],
                  0.5,0.5,0.5,
                  0,0,1);
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        glMatrixMode(GL_MODELVIEW);
        
        drawAxes();
	drawConstraintForces(cube);

        DeformableLink *dl = dynamic_cast<DeformableLink *>(cube->rootLink());
        dl->draw(GL_LINE_LOOP);

	if (movie_flag && ((int)(world.currentTime()/TIMESTEP))%movie_step == 0 )
	    addMovieFrame();

        glfwSwapBuffers();

	//usleep(1000);
    }
    std::cout << std::endl;

    if (movie_flag) finishMovie();

    return 0;
}




