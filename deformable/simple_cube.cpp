#include <GL/glfw.h>
#include <cv.h>
#include <highgui.h>
#include "FEM.h"
#include "DeformableObject.h"

using namespace afstate;

#define DEFAULT_W 640
#define DEFAULT_H 480

bool quit_flag=false, start_flag = false, movie_flag = false, edge_flag = false;

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
    case 'E':
        edge_flag = !edge_flag;
    default:
        break;
    }
}

int main(int argc, char *argv[])
{
    FEM *clip_;
    
    //clip_ = new GMSH("simple_cube.inp");
    clip_ = new GMSH("mid_cube.inp");
    clip_->assembleK();
    clip_->sortK();
    clip_->sortT();
    clip_->computeInverseForceCondensedK();
    clip_->computeDisplacementCondensedK();

    DeformableObject *defClip_;

    defClip_ = new DeformableObject(*clip_);
    VectorXd eF (defClip_->getSizeF());
    //VectorXd uD (defClip_->getSizeD());
    eF.setZero();
    //uD.setZero();
    std::cout << "size of eF = " << eF.size() << std::endl;
    //std::cout << "size of uD = " << uD.size() << std::endl;
    defClip_->setDisplayMode(DeformableObject::REGULAR_RENDERING);
    //defClip_->setDisplayMode(DeformableObject::VON_MISES_STRESS);
    defClip_->setNormalsMode(DeformableObject::NORMAL_PER_TRIANGLE);
    //defClip_->setNormalsMode(DeformableObject::NORMAL_PER_VERTEX);

    double t_ = 0.00;

    glfwInit();
    glfwOpenWindow(DEFAULT_W,DEFAULT_H,0,0,0,0,24,0, GLFW_WINDOW);
    GLfloat light0pos[] = { 0.0, 4.0, 6.0, 1.0 };
    GLfloat light1pos[] = { 6.0, 4.0, 0.0, 1.0 };
    GLfloat white[] = { 0.6, 0.6, 0.6, 1.0 };
    glfwSetKeyCallback(keyboard);

    glClearColor(0, 0, 0, 1.0);
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


    std::cout << std::endl;
    CvVideoWriter *videoWriter;
    IplImage *image;
    double LRangle=M_PI/4;
    double UDangle=M_PI/4;
    int prevState, prevX, prevY;
    prevState = glfwGetMouseButton(GLFW_MOUSE_BUTTON_LEFT);
    glfwGetMousePos(&prevX, &prevY);
    while (!quit_flag){
        std::cout << "\r t = " << t_ << std::flush;
        if (movie_flag && t_ == 0.){
            videoWriter = cvCreateVideoWriter(
                "04_stopper_rotation.avi",
                CV_FOURCC('D','I','V','X'),
                20,
                cvSize(DEFAULT_W, DEFAULT_H));
            image = cvCreateImage(
                cvSize(DEFAULT_W, DEFAULT_H),
                IPL_DEPTH_8U, 3);
        }
        if (edge_flag){
            defClip_->setDisplayMode(DeformableObject::WIRE_FRAME);
        }else{
            defClip_->setDisplayMode(DeformableObject::REGULAR_RENDERING);
        }
#if 1
        eF(11)=10*sin(M_PI*t_);
        //eF(6)=-sin(M_PI*t_);
        //uD(11) = sin(2*M_PI*t_/10);
    
        // the three calls for the deformation
        defClip_->setForce(eF);
        //defClip_->setDisplacement(uD);
        defClip_->solveU();
#endif
        defClip_->updateNodes();

        if (start_flag || movie_flag) t_ += 0.05;

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45, 
                       (double)DEFAULT_W / (double)DEFAULT_H, 
                       0.1, 100);

        int x,y,state;
        glfwGetMousePos(&x, &y);
        state = glfwGetMouseButton(GLFW_MOUSE_BUTTON_LEFT);
        if (state == GLFW_PRESS && prevState == GLFW_PRESS){
            LRangle -= (x - prevX)*0.01;
            UDangle += (y - prevY)*0.01;
            if (UDangle < -M_PI/2) UDangle = -M_PI/2;
            if (UDangle >  M_PI/2) UDangle =  M_PI/2;
        }
        prevState = state;
        prevX = x;
        prevY = y;

        double radius = 3, eye[3];
        double center[3] = {0.5, 0.5, 0.5};
        eye[0] = radius*cos(UDangle)*cos(LRangle)+center[0];
        eye[1] = radius*cos(UDangle)*sin(LRangle)+center[1];
        eye[2] = radius*sin(UDangle)+center[2];
        gluLookAt(eye[0], eye[1], eye[2],
                  center[0], center[1], center[2],
                  0,0,1);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);

        defClip_->draw();

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

        
        glfwSwapBuffers();
        usleep(10000);
    }
    std::cout << std::endl;

    return 0;
}

