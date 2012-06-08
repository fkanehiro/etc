#include <GL/glfw.h>
#include <cv.h>
#include <highgui.h>
#include <sys/time.h>
#include "FEM.h"
#include "DeformableObject.h"
# include <mach/task.h>
# include <mach/mach_init.h>

using namespace afstate;

#define DEFAULT_W 640
#define DEFAULT_H 480

size_t memused()
{
  bool resident = true;
  struct task_basic_info t_info;
  mach_msg_type_number_t t_info_count = TASK_BASIC_INFO_COUNT;
  task_info(current_task(), TASK_BASIC_INFO, (task_info_t)&t_info, &t_info_count);
  size_t size = (resident ? t_info.resident_size : t_info.virtual_size);
  return size/(1024*1024);
}

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
    FEM *stopper_;
    struct timeval t1, t2;
    
    stopper_ = new GMSH("09_part_g_free.inp");
    gettimeofday(&t1, NULL);
    stopper_->assembleK();
    gettimeofday(&t2, NULL);
    printf("assembleK() : %6.3f[ms]\n", (t2.tv_sec - t1.tv_sec)*1000 + (t2.tv_usec - t1.tv_usec)/1000.0);
    gettimeofday(&t1, NULL);
    stopper_->sortK();
    gettimeofday(&t2, NULL);
    printf("sortK() : %6.3f[ms]\n", (t2.tv_sec - t1.tv_sec)*1000 + (t2.tv_usec - t1.tv_usec)/1000.0);
    std::cout << "after sortK():" << memused() << "[MB]" << std::endl;
    stopper_->sortT();
    std::cout << "after sortT():" << memused() << "[MB]" << std::endl; 
    gettimeofday(&t1, NULL);
    stopper_->computeInverseForceCondensedK();
    std::cout << "after computing cK:"  << memused() << "[MB]" << std::endl; 
    gettimeofday(&t2, NULL);
    printf("computeInverseForceCondensedK() : %6.3f[ms]\n", (t2.tv_sec - t1.tv_sec)*1000 + (t2.tv_usec - t1.tv_usec)/1000.0);
    stopper_->computeDisplacementCondensedK();
    std::cout << "before creating a DeformableObject:"  << memused() << "[MB]" << std::endl; 

    DeformableObject *defstopper_;

    defstopper_ = new DeformableObject(*stopper_);
    VectorXd eF (defstopper_->getSizeF()); // applied forces
    VectorXd uD (defstopper_->getSizeD()); // applied displacements
    eF.setZero();
    uD.setZero();
    std::cout << "size of eF = " << eF.size() << std::endl;
    std::cout << "size of uD = " << uD.size() << std::endl;
    defstopper_->setDisplayMode(DeformableObject::VON_MISES_STRESS);
    defstopper_->setNormalsMode(DeformableObject::NORMAL_PER_TRIANGLE);
    //defstopper_->setNormalsMode(DeformableObject::NORMAL_PER_VERTEX);

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
            defstopper_->setDisplayMode(DeformableObject::WIRE_FRAME);
        }else{
            defstopper_->setDisplayMode(DeformableObject::REGULAR_RENDERING);
        }

        eF(1500)=sin(M_PI*t_)*0.1;
    
        // the three calls for the deformation
        defstopper_->setForce(eF);
        defstopper_->setDisplacement(uD);
        struct timeval t1, t2;
        gettimeofday(&t1, NULL);
        defstopper_->solveU();
        gettimeofday(&t2, NULL);
        //printf("\n solveU() : %6.3f[ms]", (t2.tv_sec - t1.tv_sec)*1000 + (t2.tv_usec - t1.tv_usec)/1000.0);
        defstopper_->updateNodes();

        if (start_flag || movie_flag) t_ += 0.05;

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45, 
                       (double)DEFAULT_W / (double)DEFAULT_H, 
                       1, 1000);

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
        
        double radius = 100, eye[3];
        double center[3] = {-100.5, 0, 0};
        eye[1] = radius*cos(UDangle)*cos(LRangle)+center[1];
        eye[2] = radius*cos(UDangle)*sin(LRangle)+center[2];
        eye[0] = radius*sin(UDangle)+center[0];
        gluLookAt(eye[0], eye[1], eye[2],
                  center[0], center[1], center[2],
                  1,0,0);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);

        defstopper_->draw();

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

