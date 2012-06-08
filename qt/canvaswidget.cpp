#include <iostream>
#include <QMouseEvent>
#include <math.h>
#include <GL/glut.h>
#include "GLmodel.h"
#include "canvaswidget.h"

CanvasWidget::CanvasWidget(QWidget *parent, const char *name) 
    : QGLWidget(parent)
{
    initViewParameters();
}

CanvasWidget::~CanvasWidget()
{
}

void CanvasWidget::initViewParameters()
{
    pan  = M_PI/4;
    tilt = M_PI/16;
    radius = 5;
    
    xCenter = 0;
    yCenter = 0;
    zCenter = 0.8;
}

void CanvasWidget::initializeGL(){
    std::cout << "initializeGL()" << std::endl;
    int width = geometry().width();
    int height = geometry().height();
    aspect = ((double)width)/height;

    GLscene *scene = GLscene::getInstance();
    scene->init();

    int argc=1;
    char *argv[] = {"dummy"};
    glutInit(&argc, argv);
}

void CanvasWidget::resizeGL( int width, int height )
{
    //std::cout << "resizeGL(" << width << "," << height << ")" << std::endl;
    glViewport(0, 0, width, height);
    GLscene::getInstance()->setScreenSize(width, height);
    aspect = ((double)width)/height;
}

void CanvasWidget::paintGL()
{
    //std::cout << "paintGL()" << std::endl;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(30,aspect, 0.1, 100);

    double xEye = xCenter + radius*cos(tilt)*cos(pan);
    double yEye = yCenter + radius*cos(tilt)*sin(pan);
    double zEye = zCenter + radius*sin(tilt);
    
    gluLookAt(xEye, yEye, zEye,
              xCenter, yCenter, zCenter,
              0,0,1);
    
    glClear(GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT);
    
    GLscene *scene = GLscene::getInstance();
    scene->draw();
}

void CanvasWidget::mousePressEvent(QMouseEvent *event)
{
    //std::cout << "mousePressEvent()" << std::endl;
    lastPos = event->pos();
}

void CanvasWidget::mouseMoveEvent(QMouseEvent *event)
{
    //std::cout << "mouseMoveEvent()" << std::endl;
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();
    Qt::MouseButtons enButton = event->buttons();
    if (enButton & Qt::LeftButton) {
        pan  -= 0.05*dx;
        tilt += 0.05*dy;
        if (tilt >  M_PI/2) tilt =  M_PI/2;
        if (tilt < -M_PI/2) tilt = -M_PI/2;
    } else if (enButton & Qt::RightButton) {
        xCenter += sin(pan)*dx*0.01;
        yCenter -= cos(pan)*dx*0.01;
        zCenter += dy*0.01;
    } else if (enButton & Qt::MidButton){
        radius *= (1+ 0.1*dy);
        if (radius < 0.1) radius = 0.1; 
    }
    lastPos = event->pos();
    glDraw();
}

void CanvasWidget::wheelEvent(QWheelEvent *event)
{
    //zoom( (qreal)event->delta() );
    glDraw();
}

void CanvasWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    Qt::MouseButtons enButton = event->buttons();
    if (enButton & Qt::LeftButton) {
        initViewParameters();
        glDraw();
    }
}
