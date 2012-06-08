#include <GL/glfw.h>
#include <cv.h>
#include <highgui.h>
#include <sys/time.h>
#include "FEM_sparse.h"
#include "DeformableObject_sparse.h"
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/gzip.hpp>

using namespace afstate;

#define DEFAULT_W 640
#define DEFAULT_H 480

bool quit_flag=false, start_flag = false, movie_flag = false, edge_flag = false;

bool loadMatrix(const char *fname, MatrixXd& m)
{
    std::ifstream ifs(fname);
    if (!ifs.is_open()){
        std::cout << "failed to open(" << fname << ")" << std::endl;
        return false;
    }

    boost::iostreams::filtering_streambuf<boost::iostreams::input> filter;
    filter.push(boost::iostreams::gzip_decompressor());
    filter.push(ifs);
    std::istream stream(&filter);

    unsigned int r, c;
    stream >> c >> r;
    char buf[10];
    stream.getline(buf, 10); // to read newline
    m.resize(c,r);
    for (unsigned int j=0; j<r; j++){
        double col[c];
        stream.read((char *)col, c*sizeof(double));
        for (unsigned int i=0; i<c; i++){
            m(i,j) = col[i];
        }
    }
    return true;
}

void saveMatrix(const char *fname, const MatrixXd& m)
{
    std::ofstream ofs(fname);

    boost::iostreams::filtering_streambuf<boost::iostreams::output> filter;
    filter.push(boost::iostreams::gzip_compressor());
    filter.push(ofs);
    std::ostream stream(&filter);

    stream << m.cols() << " "  << m.rows() << std::endl; 
    stream.write((const char *)m.data(), m.cols()*m.rows()*sizeof(double));
}

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
    
    //stopper_ = new GMSH("01_collar_0.inp");
    //stopper_ = new GMSH("01_collar_02.inp");
    //stopper_ = new GMSH("01_collar_03.inp");
    //stopper_ = new GMSH("01_collar_04.inp");
    //stopper_ = new GMSH("01_collar_270.inp");
    //stopper_ = new GMSH("01_collar_270_2.inp");
    //stopper_ = new GMSH("01_collar_270_3.inp");
    stopper_ = new GMSH("01_collar_270_4.inp");
    struct timeval t1, t2;
    gettimeofday(&t1, NULL);
    std::cout << "before creating matrices(hit return key)" << std::flush; getchar();
    stopper_->assembleK();
    gettimeofday(&t2, NULL);
    std::cout << "assembleK():" << t2.tv_sec - t1.tv_sec << "[s]" << std::endl;
    t1 = t2;
    std::cout << "after mK_ is created(hit return key)" << std::flush; getchar();
    gettimeofday(&t1, NULL);
    stopper_->sortK();
    gettimeofday(&t2, NULL);
    std::cout << "sortK():" << t2.tv_sec - t1.tv_sec << "[s]" << std::endl;
    t1 = t2;
    std::cout << "after mK_ is sorted(hit return key)" << std::flush; getchar();
    gettimeofday(&t1, NULL);
    stopper_->sortT();
    gettimeofday(&t2, NULL);
    std::cout << "sortT():" << t2.tv_sec - t1.tv_sec << "[s]" << std::endl;
    t1 = t2;

    DeformableObject *defstopper_;

    std::cout << "before creating DeformableObject(hit return key)" << std::flush; getchar();
    gettimeofday(&t1, NULL);
    defstopper_ = new DeformableObject(*stopper_);
    gettimeofday(&t2, NULL);
    std::cout << "new DeformableObject():" << t2.tv_sec - t1.tv_sec << "[s]" << std::endl;
    std::cout << "after creating DeformableObject(hit return key)" << std::flush; getchar();
    VectorXd eF (defstopper_->getSizeF()); // applied forces
    VectorXd uD (defstopper_->getSizeD()); // applied displacements
    eF.setZero();
    uD.setZero();
    std::cout << "size of eF = " << eF.size() << std::endl;
    std::cout << "size of uD = " << uD.size() << std::endl;
    //defstopper_->setDisplayMode(DeformableObject::REGULAR_RENDERING);
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

    glClearColor(0.6, 0.6, 0.8, 1.0);
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
      //std::cout << "\r t = " << t_ << std::flush;
        if (movie_flag && t_ == 0.){
            videoWriter = cvCreateVideoWriter(
                "01_collar.avi",
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

#if 1
        // 01_collar_0.inp
        //hrp::Vector3 f(0, 0.0314939, 0.0142797);
	//int nodeid = 618;

	// 01_collar_02.inp
        //hrp::Vector3 f(0, 0.0320369, 0.0130159);
	//int nodeid = 2479;

        // 01_collar_03.inp
        //hrp::Vector3 f(0, 0.0323129, 0.0123147);
	//int nodeid = 5102;
                       
        // 01_collar_04.inp

        // 01_collar_270.inp
        //hrp::Vector3 f(0, -0.0132255, 0.0319509);
	//int nodeid = 9;
        //hrp::Vector3 f(0, -0.0104064, 0.032977);
	//int nodeid = 8;
        //hrp::Vector3 f(0, -0.00969966, 0.0331918);
	//int nodeid = 127;
        //hrp::Vector3 f(0, -0.0143291, 0.0314715);
	//int nodeid = 126;
                       
        // 01_collar_270_2.inp
        //hrp::Vector3 f(0, -0.0119178, 0.0324614);
	//int nodeid = 202;
        //hrp::Vector3 f(0, -0.0130825, 0.0320098);
	//int nodeid = 201;
        //hrp::Vector3 f(0, -0.0107891, 0.0328538);
	//int nodeid = 203;
        //hrp::Vector3 f(0, -0.0120777, 0.0324022);
	//int nodeid = 218;

        // 01_collar_270_3.inp
        //hrp::Vector3 f(0, -0.0118402, 0.0324898); // 4432
        //hrp::Vector3 f(0, -0.01235, 0.0322994); // 310
        //hrp::Vector3 f(0, -0.0113464, 0.0326655); // 309
        //hrp::Vector3 f(0, -0.0116113, 0.0325723); // 280
        //hrp::Vector3 f(0, -0.012274, 0.0323284); // 279
        //hrp::Vector3 f(0, -0.0112917, 0.0326844); // 4346
	//int nodeid = 4432;
	//int nodeid = 310;
	//int nodeid = 309;
	//int nodeid = 280;
	//int nodeid = 279;
	//int nodeid = 4346;

        // 01_collar_270_4.inp
        //hrp::Vector3 f(0, -0.0116997, 0.0325406);
	//int nodeid = 6253;
        hrp::Vector3 f(0, -0.0111137, 0.0327454);
	int nodeid = 414;

        int rank = defstopper_->getNodeIndex(nodeid-1);

        f /= norm2(f);
        eF(rank*3  )=f[0];
        eF(rank*3+1)=f[1];
        eF(rank*3+2)=f[2];
#endif
    
        // the three calls for the deformation
        defstopper_->setForce(eF);
        defstopper_->setDisplacement(uD);
        gettimeofday(&t1, NULL);
        defstopper_->solveU();
        gettimeofday(&t2, NULL);
        defstopper_->updateNodes();
        printf("\n solveU() : %6.3f[ms]", (t2.tv_sec - t1.tv_sec)*1000 + (t2.tv_usec - t1.tv_usec)/1000.0);
        const VectorXd& u = defstopper_->getU();
        hrp::Vector3 uv(u[rank*3], u[rank*3+1], u[rank*3+2]);
        std::cout << "u = " << norm2(uv) << ", f = " << norm2(f) << ", f/u = " << norm2(f)/(norm2(uv)*1e3) << "[N/mm]" << std::endl;

        if (start_flag || movie_flag) t_ += 0.05;

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45, 
                       (double)DEFAULT_W / (double)DEFAULT_H, 
                       0.01, 10);

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

        double radius = 0.05, eye[3];
        double center[3] = {-0.09, 0, 0};
        eye[1] = radius*cos(UDangle)*cos(LRangle)+center[1];
        eye[2] = radius*cos(UDangle)*sin(LRangle)+center[2];
        eye[0] = radius*sin(UDangle)+center[0];
        gluLookAt(eye[0], eye[1], eye[2],
                  center[0], center[1], center[2],
                  1,0,0);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);

        defstopper_->draw();

#if 1
	glDisable(GL_LIGHTING);
	glColor3f(1,0,0);
	glBegin(GL_LINES);
	const MatrixXd& nodes = defstopper_->getNodes();
	double vx = nodes(nodeid-1, 0) + u[rank*3];
	double vy = nodes(nodeid-1, 1) + u[rank*3+1];
	double vz = nodes(nodeid-1, 2) + u[rank*3+2];
	glVertex3f(vx, vy, vz);
	glVertex3f(vx + f[0]*0.01, vy + f[1]*0.01,  vz + f[2]*0.01);

	glEnd();
	glEnable(GL_LIGHTING);
#endif

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

