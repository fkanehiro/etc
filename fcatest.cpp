#include <cstdio>
#ifdef WIN32
#include <glut.h>
#else
#include <GL/glut.h>
#endif
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <tvmet3d.h>
#ifndef WIN32
#include <values.h>
#endif
#include <algorithm>

using namespace OpenHRP;

#define W 600
#define H 600

int prev_x, prev_y, button;

std::vector<vector3> foot_shape;
vector3 p_sup, p_swg, p_swg_mod;
matrix33 R_sup, R_swg;
vector3 p_obb;
matrix33 R_obb;
double len_obb[3];
std::vector<vector3> msum;

class LessArrow {
public:
    bool operator()(const std::pair<double, vector3>& pair1,
                    const std::pair<double, vector3>& pair2) const {
        return pair1.first < pair2.first;
    }
};

vector3 p1, p2;
void minkowskiSum(const std::vector<vector3>& polygon1, 
                  const std::vector<vector3>& polygon2,
                  std::vector<vector3>& sum)
{
    std::vector<std::pair<double, vector3> > arrow;
    for (unsigned int i=0; i<polygon1.size(); i++){
        vector3 dv(polygon1[(i+1)%polygon1.size()] - polygon1[i]);
        double a = atan2(dv[1], dv[0]);
        arrow.push_back(std::make_pair(a, dv));
        if (i==0 || p1[1] < polygon1[i][1]
            || (p1[1] == polygon1[i][1] && p1[0] > polygon1[i][0])){
            p1 = polygon1[i];
        }
    }

    for (unsigned int i=0; i<polygon2.size(); i++){
        vector3 dv(polygon2[i] - polygon2[(i+1)%polygon2.size()]);
        double a = atan2(dv[1], dv[0]);
        arrow.push_back(std::make_pair(a, dv));
        if (i==0 || p2[1] < -polygon2[i][1]
            || (p2[1] == -polygon2[i][1] && p2[0] > -polygon2[i][0])){
            p2 = -polygon2[i];
        }
    }

    std::sort(arrow.begin(), arrow.end(), LessArrow());

    //std::cout << "p1:" << p1  << ", p2:" << p2 << ", arrow[0]:" << arrow[0].second << std::endl;
    vector3 v(p1+p2);
    for (unsigned int i=0; i<arrow.size(); i++){
        sum.push_back(v);
        v += arrow[i].second;
    }
}


bool lineLineIntersect(const vector3& p1, const vector3& p2,
                       const vector3& p3, const vector3& p4,
                       double& mua, double& mub)
{
    vector3 p13, p43, p21;
    double d1343,d4321,d1321,d4343,d2121;
    double numer,denom;

    p13 = p1 - p3;
    p43 = p4 - p3; 

#define EPS 1e-8
    if (fabs(p43[0])  < EPS && fabs(p43[1])  < EPS && fabs(p43[2])  < EPS)
        return false;
    p21 = p2 - p1;
    if (fabs(p21[0])  < EPS && fabs(p21[1])  < EPS && fabs(p21[2])  < EPS)
        return false;

    d1343 = p13[0] * p43[0] + p13[1] * p43[1] + p13[2] * p43[2];
    d4321 = p43[0] * p21[0] + p43[1] * p21[1] + p43[2] * p21[2];
    d1321 = p13[0] * p21[0] + p13[1] * p21[1] + p13[2] * p21[2];
    d4343 = p43[0] * p43[0] + p43[1] * p43[1] + p43[2] * p43[2];
    d2121 = p21[0] * p21[0] + p21[1] * p21[1] + p21[2] * p21[2];

    denom = d2121 * d4343 - d4321 * d4321;
    if (fabs(denom) < EPS) return false;
    numer = d1343 * d4321 - d1321 * d4343;

    mua = numer / denom;
    mub = (d1343 + d4321 * (mua)) / d4343;
    return true;
}

bool lineLineIntersect(const vector3& p1, const vector3& p2,
                       const vector3& p3, const vector3& p4,
                       vector3& pa, vector3& pb)
{
    double mua, mub;

    if (!lineLineIntersect(p1, p2, p3, p4, mua, mub)) return false;

    if (mua < 0 || mua > 1) return false;
    if (mub < 0 || mub > 1) return false;

    pa = p1 + mua*(p2-p1);
    pb = p3 + mub*(p4-p3);
    
    return true;
}


void rotate( double a[3][3], double s, double tau, int i, int j, int k, int l )
{
	double h, g;
	g = a[i][j];
	h = a[k][l];
	a[i][j] = g - s * ( h + g *tau );
	a[k][l] = h + s * ( g - h *tau );
}

bool Jacobi( double a[3][3], double v[3][3], double d[3] )
{
    int n = 3;
    int i, j, iq, ip;
    double tresh, theta, tau, t, sm, s, h, g, c, b[3], z[3];
    
    for( ip = 0; ip < n; ip++ ){
        for( iq = 0; iq < n; iq++ ) v[ip][iq] = 0.0f;
        v[ip][ip] = 1.0f;
    }
    for( ip = 0; ip < n; ip++ ){
        b[ip] = d[ip] = a[ip][ip];
        z[ip] = 0.0f;
    }
    for( i = 0; i < 50; i++ ){
        sm = 0.0f;
        for( ip = 0; ip < n - 1; ip++ ){
            for( iq = ip + 1; iq < n; iq++ ) sm += fabs(a[ip][iq]);
        }
        
        if( sm == 0.0f ) return true;
        if( i < 3 ) tresh = 0.2f * sm / ( n * n );
        else tresh = 0.0f;
        for( ip = 0; ip < n - 1; ip++ ){
            for( iq = ip + 1; iq < n; iq++ ){
                g = 100.0f * fabs( a[ip][iq] );
                if( i > 3 && ( fabs( d[ip] ) + g ) == fabs( d[ip] )
                    && ( fabs( d[iq] ) + g ) == fabs( d[iq] ) ) a[ip][iq] = 0.0f;
                else if( fabs( a[ip][iq] ) > tresh ){
                    h = d[iq] - d[ip];
                    if( ( fabs( h ) + g ) == fabs( h ) ) t = a[ip][iq] / h;
                    else{
                        theta = 0.5f * h / a[ip][iq];
                        t = 1.0f / ( fabs( theta ) + sqrt( 1.0f + theta * theta ) );
                        if( theta < 0.0f ) t = -t;
                    }
                    c = 1.0f / sqrt( 1 + t * t );
                    s = t * c;
                    tau = s / ( 1.0f + c );
                    h = t * a[ip][iq];
                    z[ip] -= h;
                    z[iq] += h;
                    d[ip] -= h;
                    d[iq] += h;
                    a[ip][iq] = 0.0f;
                    
                    for( j = 0; j < ip; j++ ) rotate( a, s, tau, j, ip, j, iq );
                    for( j = ip + 1; j < iq; j++ ) rotate( a, s, tau, ip, j, j, iq );
                    for( j = iq + 1; j < n; j++ ) rotate( a, s, tau, ip, j, iq, j );
                    for( j = 0; j < n; j++ ) rotate( v, s, tau, j, ip, j, iq );
                }
            }
        }
        for( ip = 0; ip < n; ip++ ){
            b[ip] += z[ip];
            d[ip] = b[ip];
            z[ip] = 0.0f;
        }
    }
    
    return false;
}

// 与えられたポリゴン群を包含するOBBを作成
void createOBB( const std::vector < vector3 > & vertices,
                vector3& p, vector3 axis[3], double length[3])
{
    unsigned int Size = (int)vertices.size();
    // average
    vector3 m( 0, 0, 0 );
    for( int i = 0; i < Size; ++i ){
        m += vertices[i];
    }
    m /= Size;

    // covariant matrix
    double C11 = 0, C22 = 0, C33 = 0, C12 = 0, C13 = 0,  C23 = 0;
    for( int i = 0; i < Size; ++i ){
        C11 += ( vertices[i][0] - m[0] ) * ( vertices[i][0] - m[0] );
        C22 += ( vertices[i][1] - m[1] ) * ( vertices[i][1] - m[1] );
        C33 += ( vertices[i][2] - m[2] ) * ( vertices[i][2] - m[2] );
        C12 += ( vertices[i][0] - m[0] ) * ( vertices[i][1] - m[1] );
        C13 += ( vertices[i][0] - m[0] ) * ( vertices[i][2] - m[2] );
        C23 += ( vertices[i][1] - m[1] ) * ( vertices[i][2] - m[2] );
    }
    C11 /= Size;
    C22 /= Size;
    C33 /= Size;
    C12 /= Size;
    C13 /= Size;
    C23 /= Size;

    double Matrix[3][3] = {{C11, C12, C13},
                           {C12, C22, C23},
                           {C13, C23, C33}};

    // jacobi法で固有値 & 固有ベクトルを算出
    double EigenVectors[3][3];
    double EigenValue[3];
    Jacobi( Matrix, EigenVectors, EigenValue );

    // 固有値を降順でソート
    struct SORT{
        int ID;
        double Value;
    } Sort[3] = { { 0, EigenValue[0] }, { 1, EigenValue[1] }, { 2, EigenValue[2] } };

    for( int j = 0; j < 2; ++j ){
        for( int i = 2; i > j; --i ){
            if( Sort[i - 1].Value < Sort[i].Value ){
                SORT a = Sort[i];
                Sort[i] = Sort[i - 1];
                Sort[i - 1] = a;
            }
        }
    }

    for( int i = 0; i < 3; ++i ){
        axis[i][0] = EigenVectors[0][Sort[i].ID];
        axis[i][1] = EigenVectors[1][Sort[i].ID];
        axis[i][2] = EigenVectors[2][Sort[i].ID];
    }

    // 境界ボックスを算出
    double min[3] = {  DBL_MAX,  DBL_MAX,  DBL_MAX };
    double max[3] = { -DBL_MAX, -DBL_MAX, -DBL_MAX };
    for( int j = 0; j < 3; ++j ){
        for( int i = 0; i < Size; ++i ){
            double a = dot( vertices[i], axis[j] );
            if( min[j] > a ) min[j] = a;
            if( max[j] < a ) max[j] = a;
        }
    }

    p = axis[0] * ( ( min[0] + max[0] ) / 2 )
        + axis[1] * ( ( min[1] + max[1] ) / 2 )
        + axis[2] * ( ( min[2] + max[2] ) / 2 );

    for( int i = 0; i < 3; ++i ) length[i] = max[i] - min[i];
}


bool isInside(const vector3& v, const std::vector<vector3>& vs)
{
    for (unsigned int i=0; i<vs.size(); i++){
        vector3 dv1(vs[(i+1)%vs.size()] - vs[i]);
        vector3 dv2(v - vs[i]);
        vector3 v(cross(dv1, dv2));
        if (v[2] < 0) return false;
    }
    return true;
}

void projectToTheNearestEdge(const vector3& v, const std::vector<vector3>& vs,
                             vector3& p)
{
    double minD2, D2;
    for (unsigned int i=0; i<vs.size(); i++){
        vector3 edge(vs[(i+1)%vs.size()]-vs[i]);
        vector3 dv(v - vs[i]);
        double l = norm2(edge);
        double l2 = dot(dv,edge)/l;
        D2 = dot(dv,dv) - l2*l2;
        if (i==0||D2 < minD2){
            minD2 = D2;
            p = vs[i] + l2/l*edge;
        }
    }
}

void solveOverlap()
{
    std::vector<vector3> vs_sup, vs_swg;
    for (unsigned int i=0; i<foot_shape.size(); i++){
        vector3 v(p_sup + R_sup*foot_shape[i]);
        vs_sup.push_back(v);
    }
    for (unsigned int i=0; i<foot_shape.size(); i++){
        vector3 v(p_swg + R_swg*foot_shape[i]);
        vs_swg.push_back(v);
    }

    msum.clear();
    minkowskiSum(vs_sup, vs_swg, msum);

    vector3 o(0);
    if (isInside(o, msum)){
        vector3 t;
        projectToTheNearestEdge(o, msum, t);
        p_swg_mod = p_swg + t;
    }else{
        p_swg_mod = p_swg;
    }
}

void drawFoot(const vector3& p, const matrix33& R)
{
    glBegin(GL_LINE_LOOP);
    for (unsigned int i=0; i<foot_shape.size(); i++){
        vector3 v( p + R*foot_shape[i] );
        glVertex2d(v[0], v[1]); 
    }
    glEnd();
}

void drawCross(const vector3& p)
{
    glBegin(GL_LINES);
    glVertex2d(p[0]+0.05,p[1]+0.05);
    glVertex2d(p[0]-0.05,p[1]-0.05);
    glVertex2d(p[0]+0.05,p[1]-0.05);
    glVertex2d(p[0]-0.05,p[1]+0.05);
    glEnd();
}

void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT);

    glColor3d(1,0,0);
    drawFoot(p_sup, R_sup);

    glColor3d(0,1,0);
    drawFoot(p_swg, R_swg);

    glColor3d(0,0,1);
    drawFoot(p_swg_mod, R_swg);

    glColor3d(1,1,1);
    glBegin(GL_LINE_LOOP);
    for (int i=0; i<msum.size(); i++){
        glVertex2d(msum[i][0], msum[i][1]);
    }
    glEnd();
    drawCross(vector3(0));
    drawCross(p1);
    drawCross(p2);
    if (msum.size()) drawCross(msum[0]);

    glutSwapBuffers();
}

void init(void)
{
    glClearColor(0.0, 0.0, 0.0, 1.0);

    foot_shape.push_back(vector3( 0.12, 0.26,0));
    foot_shape.push_back(vector3(-0.15, 0.26,0));
    foot_shape.push_back(vector3(-0.16,-0.20,0));
    foot_shape.push_back(vector3( 0.11,-0.20,0));

    p_sup = 0.0;
    p_swg = 0.0; p_swg[0] = 0.38;
    p_swg_mod = p_swg;
    
    R_sup = R_swg = tvmet::identity<matrix33>();
}

void mouse(int _button, int state, int x, int y)
{
    prev_x = x; 
    prev_y = y;
    button = _button;
}

void idle(void)
{
    std::cout << "idle" << std::endl;
    glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y)
{
    switch (key) {
    case 'q':
    case 'Q':
    case '\033':  /* '\033' は ESC の ASCII コード */
        exit(0);
    default:
        break;
    }
}

void motion(int x, int y)
{
    int dx = x - prev_x;
    int dy = y - prev_y; 

    if (button == GLUT_LEFT_BUTTON){ 
        p_swg[0] += 0.003*dx;
        p_swg[1] -= 0.003*dy;
    }else{
        matrix33 R(tvmet::identity<matrix33>());
        double th = 0.003*dy;
        R(0,0) = cos(th); R(0,1) = sin(th); 
        R(1,0) = -sin(th); R(1,1) = cos(th); 
        R(2,2) = 1.0;
        matrix33 newR(R_swg*R);
        R_swg = newR;
    }

    prev_x = x;
    prev_y = y;

    solveOverlap();

    glutPostRedisplay();
}

int main(int argc, char *argv[])
{
    glutInitWindowSize(W, H);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);
    //glutIdleFunc(idle);
    //glutReshapeFunc(resize);
    init();
    glutMainLoop();
    return 0;
}
