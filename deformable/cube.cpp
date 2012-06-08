#include <GL/glfw.h>
#include "FEM.h"
#include "DeformableObject.h"

using namespace afstate;

#define DEFAULT_W 640
#define DEFAULT_H 480

int main(int argc, char *argv[])
{
    FEM *clip_;
    
    clip_ = new GMSH("cube111.inp");
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
#if 1
        uD(4) = -1;
#if 0
        eF(3) = 1.;
        for( int i=40; i<53; ++i)
            {
                eF(3*i)=sin(4.0*M_PI*t_/3);
            }
#endif
#endif
#if 0
        for( int i=4; i<8; ++i)
            {
                uD(3*i+1)=sin(4.0*M_PI*t_/10)/4;
                uD(3*i)=cos(4.0*M_PI*t_/10)/4;
            }
        for( int i=20; i<32; ++i)
            {
                uD(3*i+1)=sin(4.0*M_PI*t_/10)/4;
                uD(3*i)=cos(4.0*M_PI*t_/10)/4;
            }
        for( int i=46; i<59; ++i)
            {
                uD(3*i+1)=sin(4.0*M_PI*t_/10)/4;
                uD(3*i)=cos(4.0*M_PI*t_/10)/4;
            }
#endif    
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
        gluLookAt(2,-2,2,  0,0,0,  0,0,1);

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

