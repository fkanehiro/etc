#include <GL/glfw.h>
#include <tvmet/Vector.h>
#include <fstream>
#include <iostream>
#include <vector>

typedef tvmet::Vector<float, 3> v3f; 
typedef tvmet::Vector<int, 3> v3i; 


#define DEFAULT_W 1000
#define DEFAULT_H 1000

bool quit_flag=false, edge_flag=false, face_mode=true;
int face_id = 0, vertex_id = 0;
std::vector<std::vector<v3i> > indexes;
std::vector<v3f> vertices;

void keyboard(int key, int action)
{
    //printf("keyboard(%d)\n", key);
    if (action == GLFW_RELEASE) return;

    switch(key){
    case 'Q':
        quit_flag = true;
        break;
    case 'E':
        edge_flag = !edge_flag;
        break;
    case 'M':
        face_mode = !face_mode;
        std::cout << (face_mode ? "face mode" : "vertex mode") << std::endl;
        break;
    case 'N':
        if (face_mode){
            for (unsigned int i=face_id+1; i< indexes.size(); i++){
                if (indexes[i].size() > 0) {
                    face_id = i;
                    std::cout << "face_id = " << face_id << std::endl;
                    break;
                }
            }
        }else{
            if (vertex_id < vertices.size()-1) vertex_id++;
            std::cout << "vertex_id = " << vertex_id << std::endl;
        }
        break;
    case 'P':
        if (face_mode){
            for (int i=face_id-1; i>=0; i--){
                if (indexes[i].size() > 0) {
                    face_id = i;
                    std::cout << "face_id = " << face_id << std::endl;
                    break;
                }
            }
        }else{
            if (vertex_id > 0) vertex_id--; 
            std::cout << "vertex_id = " << vertex_id << std::endl;
        }
        break;
    case 'V':
        for (unsigned int i=0; i<indexes[face_id].size(); i++){
            std::cout << indexes[face_id][i][0] << ","
                      << indexes[face_id][i][1] << ","
                      << indexes[face_id][i][2] << std::endl;
        }
        break;
    default:
        break;
    }
}

int main(int argc, char *argv[])
{
    if (argc < 2){
        std::cerr << "Usage:" << argv[0] << "[msh file]" << std::endl;
        return 1;
    }
    std::ifstream ifs(argv[1]);

    if (!ifs.is_open()){
        std::cerr << "failed to open " << argv[1] << std::endl;
        return 1;
    }

    char buf[1024];
    for (int i=0; i<4; i++){
        ifs.getline(buf, 1024);
    }
    unsigned int nvert, id;
    ifs >> nvert;
    std::cout << "number of vertices = " << nvert << std::endl;
    vertices.resize(nvert);
    for (unsigned int i=0; i<nvert; i++){
        ifs >> id >> vertices[i][0] >> vertices[i][1] >> vertices[i][2];
    }

    v3f vsum(0);
    for (unsigned int i=0; i<nvert; i++){
        vsum += vertices[i];
    }
    vsum /= vertices.size();

    for (int i=0; i<3; i++){
        ifs.getline(buf, 1024);
    }
    unsigned int nelement;
    ifs >> nelement;
    unsigned int dummy, type, fid, ntag;
    std::cout << "number of elements = " << nelement << std::endl;
    v3i tri;
    for (unsigned int i=0; i < nelement; i++){
        ifs >> id >> type >> ntag >> fid;
        for (unsigned int j=0; j<ntag-1; j++) ifs >> dummy;
        ifs >> tri[0] >> tri[1] >> tri[2];
        if (type == 2){
            //std::cout << "fid = " << fid << std::endl;
            if (indexes.size() <= fid) indexes.resize(fid+1);
            for (int j=0; j<3; j++) tri[j] -= 1;
            indexes[fid].push_back(tri);
        }else{
            break;
        }
    } 
    std::cout << "number of faces = " << indexes.size() << std::endl;
    std::vector<std::vector<v3f> > normals(indexes.size());
    for (unsigned int i=0; i<indexes.size(); i++){
        normals[i].resize(indexes[i].size());
        for (unsigned int j=0; j<indexes[i].size(); j++){
            v3f v1(vertices[indexes[i][j][1]] - vertices[indexes[i][j][0]]);
            v3f v2(vertices[indexes[i][j][2]] - vertices[indexes[i][j][1]]);
            v3f v3(cross(v1, v2));
            v3f n(normalize(v3));
            normals[i][j] = n;
        }
    }

    glfwInit();
    glfwOpenWindow(DEFAULT_W,DEFAULT_H,0,0,0,0,24,0, GLFW_WINDOW);
    GLfloat light0pos[] = { 0.0, 120.0, 180.0, 1.0 };
    GLfloat light1pos[] = { 180.0, 120.0, 0.0, 1.0 };
    GLfloat white[] = { 0.6, 0.6, 0.6, 1.0 };
    glfwSetKeyCallback(keyboard);
    glfwEnable( GLFW_KEY_REPEAT );

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
    double LRangle=M_PI/4;
    double UDangle=M_PI/4;
    int prevState, prevX, prevY;
    prevState = glfwGetMouseButton(GLFW_MOUSE_BUTTON_LEFT);
    glfwGetMousePos(&prevX, &prevY);
    while (!quit_flag){
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45, 
                       (double)DEFAULT_W / (double)DEFAULT_H, 
                       0.1, 200);

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

        double radius = 0.2, eye[3];
        eye[0] = radius*cos(UDangle)*cos(LRangle);
        eye[1] = radius*cos(UDangle)*sin(LRangle);
        eye[2] = radius*sin(UDangle);
#if 1
        gluLookAt(eye[0]+vsum[0], eye[1]+vsum[1], eye[2]+vsum[2],
                  vsum[0], vsum[1], vsum[2],
                  0,0,1);
#else
        gluLookAt(eye[2]+vsum[0], -eye[1]+vsum[1], eye[0]+vsum[2],
                  vsum[0], vsum[1], vsum[2],
                  1,0,0);
#endif

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);

        glDisable(GL_LIGHTING);
        glBegin(GL_LINES);
        glColor3f(1,0,0);
        glVertex3f(vsum[0], vsum[1], vsum[2]);
        glVertex3f(vsum[0]+radius, vsum[1], vsum[2]);
        glColor3f(0,1,0);
        glVertex3f(vsum[0], vsum[1], vsum[2]);
        glVertex3f(vsum[0], vsum[1]+radius, vsum[2]);
        glColor3f(0,0,1);
        glVertex3f(vsum[0], vsum[1], vsum[2]);
        glVertex3f(vsum[0], vsum[1], vsum[2]+radius);
        glEnd();
        glEnable(GL_LIGHTING);

        
        float green[] = {0,1,0,1};
        float red[] = {1,0,0,1};
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, 
                     green);

        glBegin(GL_TRIANGLES);
        for (unsigned int i=0; i<indexes.size(); i++){
            if (indexes[i].size() == 0) continue;
            if (face_mode && i == face_id){
                glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, 
                             red);
            }
            for (unsigned int j=0; j<indexes[i].size(); j++){
                glNormal3fv(normals[i][j].data());
                for (unsigned int k=0; k<3; k++){
                    glVertex3fv(vertices[indexes[i][j][k]].data());
                }
            } 
            if (face_mode && i == face_id){
                glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, 
                             green);
            }
        }
        glEnd();
        if (!face_mode){
            glDisable(GL_LIGHTING);
            glBegin(GL_LINES);
            glColor3f(1,0,0);
            v3f v1 = vertices[vertex_id];
            v3f v2 = vertices[vertex_id];
            v1[0] += 0.01;
            v2[0] -= 0.01;
            glVertex3fv(v1.data());
            glVertex3fv(v2.data());
            v1[0] -= 0.01;
            v2[0] += 0.01;
            v1[1] += 0.01;
            v2[1] -= 0.01;
            glVertex3fv(v1.data());
            glVertex3fv(v2.data());
            v1[1] -= 0.01;
            v2[1] += 0.01;
            v1[2] += 0.01;
            v2[2] -= 0.01;
            glVertex3fv(v1.data());
            glVertex3fv(v2.data());
            glEnd();
            glEnable(GL_LIGHTING);
        }

        glfwSwapBuffers();
        usleep(10000);
    }
    std::cout << std::endl;

    return 0;
}

