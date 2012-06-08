#include <GL/glfw.h>
#include <unistd.h>

int main(int argc, char *argv[])
{
    glfwInit();

    glfwOpenWindow(512, 256,0,0,0,0,24,0, GLFW_WINDOW);
    glViewport(0,0,512,256);
    glLoadIdentity();
    glOrtho(0,200,0,100,-1,1);

    glClear(GL_COLOR_BUFFER_BIT);
    glColor3d(1,0,0);
    glBegin(GL_POLYGON);
    glVertex2d( 10, 10);
    glVertex2d( 90, 10);
    glVertex2d( 90,  90);
    glVertex2d(10,  90);
    glEnd();
    glfwSwapBuffers();

    sleep(3);
    return 0;
}
