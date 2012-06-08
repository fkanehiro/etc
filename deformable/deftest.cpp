#include <GL/glfw.h>
#include <cv.h>
#include <highgui.h>
#include <set>
#include "FEM.h"
#include "DeformableObject.h"
#include "LCPsolver.h"

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
    
    //clip_ = new GMSH("simple_cube.inp");
    clip_ = new GMSH("mid_cube.inp");
    //clip_ = new GMSH("cube111.inp");
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
    //defClip_->setDisplayMode(DeformableObject::REGULAR_RENDERING);
    defClip_->setDisplayMode(DeformableObject::WIRE_FRAME);
    //defClip_->setDisplayMode(DeformableObject::VON_MISES_STRESS);
    defClip_->setNormalsMode(DeformableObject::NORMAL_PER_TRIANGLE);
    //defClip_->setNormalsMode(DeformableObject::NORMAL_PER_VERTEX);

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


    MatrixXd& nodes = defClip_->getNodes();
    VectorXi& indexDefNode = defClip_->getIndexDefNode();
    MatrixXd& fK = defClip_->getfK();
    VectorXd& uL = defClip_->getuL(); 

    defClip_->solveU();
    defClip_->updateNodes();

    double t_ = 0;
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

        if (start_flag || movie_flag) t_ += 0.01;
#if 1
        std::vector<int> collidingNodeIndices;
        double thd = 0.1*sin(t_);
        int axis = 2;
        double n[] = {0,0,0};
        n[axis] = 1;
        
        for (int i=0; i<nodes.rows(); i++){
            int rank = indexDefNode(i);
            //std::cout << "rank:" << rank << std::endl;
            if (rank < 0 || rank*3 >= eF.size()) continue;
            double f[] = {eF(rank*3), eF(rank*3+1), eF(rank*3+2)};
            if (nodes(i,axis) + uL(rank*3+axis) < thd 
                || (f[0]*f[0] + f[1]*f[1] + f[2]*f[2]) > 1e-16) {
                collidingNodeIndices.push_back(i);
                //std::cout << i << "(" << indexDefNode(i) << ") ";
            }
        }
        //std::cout << std::endl;
        
        int nnode = collidingNodeIndices.size();
        std::cout << "nnode:" << nnode << std::endl;
        
        if (nnode > 0){
            MatrixXd H(nnode, nnode*3);
            H.setZero();
            for (int i=0; i<nnode; i++){
                H(i,i*3  ) = n[0];
                H(i,i*3+1) = n[1];
                H(i,i*3+2) = n[2];
            }
            
            // extract C from fK
            MatrixXd C(nnode*3, nnode*3);
            for (int i=0; i<nnode; i++){
                for (int j=0; j<3; j++){
                    int dstrow = i*3+j;
                    int srcrow = indexDefNode(collidingNodeIndices[i])*3+j;
                    for (int k=0; k<nnode; k++){
                        for (int l=0; l<3; l++){
                            int dstcol = k*3+l;
                            int srccol = indexDefNode(collidingNodeIndices[k])*3+l;
                            C(dstrow, dstcol) = fK(srcrow, srccol);
                        }
                    }
                }
            }
            
            MatrixXd W = H*C*H.transpose();
            
            hrp::rmdmatrix M(nnode, nnode);
            for (int i=0; i<W.rows(); i++){
                for (int j=0; j<W.cols(); j++){
                    M(i,j) = W(i,j);
                }
            }
            
            hrp::dvector b(nnode), x(nnode);
            x.clear();
            
            for (int i=0; i<nnode; i++){
                int nodeidx = collidingNodeIndices[i];
                int rank = indexDefNode(nodeidx);
                b(i) = - (thd - nodes(nodeidx,axis));
            }
            
            hrp::solveMCPByProjectedGaussSeidel(M, b, x);

            double totalf=0;
            for (unsigned int i=0; i<x.size(); i++){
                totalf += x(i); 
            }
            //std::cout << "thd = " << thd << ", total_f = " << totalf << std::endl;
            for (int i=0; i<nnode; i++){
                double fn = x(i);
                int rank = indexDefNode(collidingNodeIndices[i]);
                eF(rank*3+0) = n[0]*fn;
                eF(rank*3+1) = n[1]*fn;
                eF(rank*3+2) = n[2]*fn;
            }
            defClip_->setForce(eF);
            defClip_->solveU();
            defClip_->updateNodes();
        }
#endif

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(30, 
                       (double)DEFAULT_W / (double)DEFAULT_H, 
                       0.1, 100);
        double radius = 5, eye[3];
        eye[0] = radius*cos(UDangle)*cos(LRangle)+0.5;
        eye[1] = radius*cos(UDangle)*sin(LRangle)+0.5;
        eye[2] = radius*sin(UDangle)+0.5;
        gluLookAt(eye[0], eye[1], eye[2],
                  0.5,0.5,0.5,
                  0,0,1);
        //gluLookAt(2,2,2,  0,0,0,  0,1,0);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);

        glDisable(GL_LIGHTING);
        glBegin(GL_LINES);
        // axes
        glColor3f(1,0,0);
        glVertex3f(0,0,0);
        glVertex3f(2,0,0);

        glColor3f(0,1,0);
        glVertex3f(0,0,0);
        glVertex3f(0,2,0);

        glColor3f(0,0,1);
        glVertex3f(0,0,0);
        glVertex3f(0,0,2);
        // forces
        glColor3f(1,1,0);
        for (int i=0; i<nnode; i++){
            int nodeidx = collidingNodeIndices[i];
            int rank = indexDefNode(nodeidx);
            glVertex3f(nodes(nodeidx,0) + uL(rank*3+0),
                       nodes(nodeidx,1) + uL(rank*3+1),
                       nodes(nodeidx,2) + uL(rank*3+2));
            glVertex3f(nodes(nodeidx,0) + uL(rank*3+0) + eF(rank*3+0),
                       nodes(nodeidx,1) + uL(rank*3+1) + eF(rank*3+1),
                       nodes(nodeidx,2) + uL(rank*3+2) + eF(rank*3+2));
        }
        
        glEnd();
        // pushing plane
        glBegin(GL_LINE_LOOP);
        glColor3f(1,1,1);
        if (axis == 1){
            glVertex3f(0,thd,0);
            glVertex3f(1,thd,0);
            glVertex3f(1,thd,1);
            glVertex3f(0,thd,1);
        }else if (axis == 2){
            glVertex3f(0,0,thd);
            glVertex3f(1,0,thd);
            glVertex3f(1,1,thd);
            glVertex3f(0,1,thd);
        }
        glEnd();
        glEnable(GL_LIGHTING);
        
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

