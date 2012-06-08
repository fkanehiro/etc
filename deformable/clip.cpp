#include <GL/glfw.h>
#include <cv.h>
#include <highgui.h>
#include <sys/time.h>
#include "FEM.h"
#include "DeformableObject.h"

using namespace afstate;

#define DEFAULT_W 640
#define DEFAULT_H 480

double LRangle=M_PI/4;
double UDangle=M_PI/4;
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
    case 'L':
        LRangle -= 0.1;
        break;
    case 'R':
        LRangle += 0.1;
        break;
    case 'U':
        UDangle += 0.1;
        break;
    case 'D':
        UDangle -= 0.1;
        break;
    default:
        break;
    }
}

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
    VectorXd eF (defClip_->getSizeF()); // applied forces
    VectorXd uD (defClip_->getSizeD()); // applied displacements
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
    while (!quit_flag){
        std::cout << "\r t = " << t_ << std::flush;
        if (movie_flag && t_ == 0.){
            videoWriter = cvCreateVideoWriter(
                "clip.avi",
                CV_FOURCC('D','I','V','X'),
                20,
                cvSize(DEFAULT_W, DEFAULT_H));
            image = cvCreateImage(
                cvSize(DEFAULT_W, DEFAULT_H),
                IPL_DEPTH_8U, 3);
        }

        eF(0)=sin(M_PI*t_)/50000;
        eF(6)=-sin(M_PI*t_)/50000;
    
        // the three calls for the deformation
        defClip_->setForce(eF);
        defClip_->setDisplacement(uD);
        struct timeval t1, t2;
        gettimeofday(&t1, NULL);
        defClip_->solveU();
        gettimeofday(&t2, NULL);
        std::cout << "solveU() : " << (t2.tv_sec - t1.tv_sec)*1000 + (t2.tv_usec - t1.tv_usec)/1000.0 << "[ms]" << std::endl;
        defClip_->updateNodes();

        if (start_flag || movie_flag) t_ += 0.05;

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45, 
                       (double)DEFAULT_W / (double)DEFAULT_H, 
                       0.1, 100);
        double radius = 0.2, eye[3];
        eye[0] = radius*cos(UDangle)*cos(LRangle);
        eye[1] = radius*cos(UDangle)*sin(LRangle);
        eye[2] = radius*sin(UDangle);
        gluLookAt(eye[0], eye[1], eye[2],
                  0,0,0,
                  0,0,1);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);

        defClip_->draw();

        if (movie_flag){
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
            if (t_ > 10){
                cvReleaseVideoWriter(&videoWriter);
                cvReleaseImage(&image);
                movie_flag = false;
            }
        }

        glfwSwapBuffers();
        usleep(10000);
    }
    std::cout << std::endl;

    return 0;
}

