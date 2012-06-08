#include <GL/glfw.h>
#include "FEM.h"
#include "DeformableObject.h"

using namespace afstate;

#define DEFAULT_W 640
#define DEFAULT_H 480

int main(int argc, char *argv[])
{
    FEM *clip_;
    
    clip_ = new GMSH("ClipCpp.inp");
    clip_->assembleK();
    clip_->sortK();
    clip_->sortT();
    clip_->computeInverseForceCondensedK();
    clip_->computeDisplacementCondensedK();

    DeformableObject *defClip_;

    defClip_ = new DeformableObject(*clip_);
    VectorXd eF (defClip_->getSizeF());
    VectorXd uD (defClip_->getSizeD());
    eF.setZero();
    uD.setZero();
    std::cout << "size of eF = " << eF.size() << std::endl;
    std::cout << "size of uD = " << uD.size() << std::endl;
    //defClip_->setDisplayMode(DeformableObject::REGULAR_RENDERING);
    defClip_->setDisplayMode(DeformableObject::VON_MISES_STRESS);
    //defClip_->setNormalsMode(DeformableObject::NORMAL_PER_TRIANGLE);
    defClip_->setNormalsMode(DeformableObject::NORMAL_PER_VERTEX);

    double t_ = 0.00;

    glfwInit();
    glfwOpenWindow(DEFAULT_W,DEFAULT_H,0,0,0,0,24,0, GLFW_WINDOW);
    GLfloat light0pos[] = { 0.0, 4.0, 6.0, 1.0 };
    GLfloat light1pos[] = { 6.0, 4.0, 0.0, 1.0 };
    GLfloat white[] = { 0.6, 0.6, 0.6, 1.0 };

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
    while (t_ < 10){
        std::cout << "\r t = " << t_ << std::flush;

        eF(0)=sin(M_PI*t_)/50000;
        eF(6)=-sin(M_PI*t_)/50000;
    
        // the three calls for the deformation
        defClip_->setForce(eF);
        defClip_->setDisplacement(uD);
        defClip_->solveU();
        defClip_->updateNodes();

        t_ += 0.05;

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45, 
                       (double)DEFAULT_W / (double)DEFAULT_H, 
                       0.1, 100);
        gluLookAt(0.1,0.1,0.1,
                  0,0,0,
                  0,0,1);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);

        defClip_->draw();
        
        glfwSwapBuffers();
        usleep(10000);
    }
    std::cout << std::endl;
    sleep(3);

    return 0;
}

