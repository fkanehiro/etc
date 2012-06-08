#include <iostream>
#include <fstream>
/* fbotest.c */

#include    <GL/glew.h>
#include    <GL/glut.h>

#include    <stdio.h>
#include    <stdlib.h>

/* window */

#define WINDOW_WIDTH        640
#define WINDOW_HEIGHT       480
#define WINDOW_POSITION_X   0
#define WINDOW_POSITION_Y   0

/* projection */

#define NEAR_CLIPPING_LENGTH    1.0
#define FAR_CLIPPING_LENGTH     10000.0

#define FIELD_OF_VIEW       40.0

/* camera position */

#define EYE_X       0.0
#define EYE_Y       0.0
#define EYE_Z       200.0

#define TARGET_X    0.0
#define TARGET_Y    0.0
#define TARGET_Z    0.0

#define UP_X        0.0
#define UP_Y        1.0
#define UP_Z        0.0

/* global variables */

int window_width;
int window_height;
int counter_cube;
int counter_teapot;
int is_rotate_cube;
int is_rotate_teapot;

/* OpenGL settings */

void
InitMiscGL( void )
{
    /* shading model */

    glShadeModel( GL_SMOOTH );

    /* depth test */
    
    glEnable( GL_DEPTH_TEST );

    /* culling */

    glCullFace( GL_BACK );
    glEnable( GL_CULL_FACE );

    /* normalize */

    glEnable( GL_NORMALIZE );
}

void
SetLight( void )
{
    GLfloat light_position0[] = { 100.0, 150.0, 200.0, 1.0 };
    GLfloat light_ambient0[] = { 0.2, 0.2, 0.2, 1.0 };
    GLfloat light_diffuse0[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat light_specular0[] = { 1.0, 1.0, 1.0, 1.0 };

    glLightfv( GL_LIGHT0, GL_POSITION, light_position0 );
    glLightfv( GL_LIGHT0, GL_AMBIENT, light_ambient0 );
    glLightfv( GL_LIGHT0, GL_DIFFUSE, light_diffuse0 );
    glLightfv( GL_LIGHT0, GL_SPECULAR, light_specular0 );

    glEnable( GL_LIGHT0 );
    glEnable( GL_LIGHTING );
}

void
SetMaterial( void )
{
    GLfloat mat_ambient0[] = { 0.1, 0.1, 0.2, 1.0 };
    GLfloat mat_diffuse0[] = { 0.3, 0.4, 1.0, 1.0 };
    GLfloat mat_specular0[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat mat_shininess0[] = { 50.0 };
    
    glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient0 );
    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse0 );
    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular0 );
    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess0 );
}

/* draw cube with texture coordinates */

void
DrawCubeSub( float l )
{
    glPushMatrix();

    glTranslatef( l, 0.0f, 0.0f );
    glBegin( GL_QUADS );
    glNormal3f( 1.0f, 0.0f, 0.0f );
    glTexCoord2f( 0.0f, 0.0f );
    glVertex3f( 0.0f, -l, -l );
    glTexCoord2f( 1.0f, 0.0f );
    glVertex3f( 0.0f, l, -l );
    glTexCoord2f( 1.0f, 1.0f );
    glVertex3f( 0.0f, l, l );
    glTexCoord2f( 0.0f, 1.0f );
    glVertex3f( 0.0f, -l, l );
    glEnd();
	
    glPopMatrix();
}

void
DrawCube( float l )
{
    int	i;
	
    glPushMatrix();

    for ( i = 0; i < 4; i++ ) {
        DrawCubeSub( l );
        glRotatef( 90.0f, 0.0f, 0.0f, 1.0f );
    }
    glRotatef( 90.0f, 0.0f, 1.0f, 0.0f );
    DrawCubeSub( l );
    glRotatef( 180.0f, 0.0f, 1.0f, 0.0f );
    DrawCubeSub( l );
	
    glPopMatrix();
}



/* framebuffer object ********************************************************/

#define	FRAMEBUFFER_WIDTH		256
#define	FRAMEBUFFER_HEIGHT		256

#define	FRAMEBUFFER_NEAR_CLIPPING_LENGTH	1.0
#define	FRAMEBUFFER_FAR_CLIPPING_LENGTH		1000.0

#define	FRAMEBUFFER_FIELD_OF_VIEW			40.0

#define FRAMEBUFFER_EYE_X       0.0
#define FRAMEBUFFER_EYE_Y       0.0
#define FRAMEBUFFER_EYE_Z       100.0

#define FRAMEBUFFER_TARGET_X    0.0
#define FRAMEBUFFER_TARGET_Y    0.0
#define FRAMEBUFFER_TARGET_Z    0.0

#define FRAMEBUFFER_UP_X        0.0
#define FRAMEBUFFER_UP_Y        1.0
#define FRAMEBUFFER_UP_Z        0.0

#define	TEXTURE_WIDTH			FRAMEBUFFER_WIDTH
#define	TEXTURE_HEIGHT			FRAMEBUFFER_HEIGHT

#define	RENDERBUFFER_WIDTH		FRAMEBUFFER_WIDTH
#define	RENDERBUFFER_HEIGHT		FRAMEBUFFER_HEIGHT

GLuint	texture_name;
GLuint	renderbuffer_name;
GLuint	framebuffer_name;

GLint	viewport[ 4 ];

void
SaveFramebufferStatus( void )
{
    glGetIntegerv( GL_VIEWPORT, viewport );
    glMatrixMode( GL_PROJECTION );
    glPushMatrix();
    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
}

void
RestoreFramebufferStatus( void )
{
    glViewport( viewport[ 0 ], viewport[ 1 ], viewport[ 2 ], viewport[ 3 ] );
    glMatrixMode( GL_PROJECTION );
    glPopMatrix();
    glMatrixMode( GL_MODELVIEW );
    glPopMatrix();
}

void
InitTexture( void )
{
    glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
    glGenTextures( 1, &texture_name );
    glBindTexture( GL_TEXTURE_2D, texture_name );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, TEXTURE_WIDTH, TEXTURE_HEIGHT,
                  0, GL_RGBA, GL_UNSIGNED_BYTE, 0 );
}

void
InitRenderbuffer( void )
{
    glGenRenderbuffersEXT( 1, &renderbuffer_name );
    glBindRenderbufferEXT( GL_RENDERBUFFER_EXT, renderbuffer_name );
    glRenderbufferStorageEXT( GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT,
                              RENDERBUFFER_WIDTH, RENDERBUFFER_HEIGHT );
}

void
InitFramebuffer( void )
{
    glGenFramebuffersEXT( 1, &framebuffer_name );
    glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, framebuffer_name );

    glFramebufferTexture2DEXT( GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
                               GL_TEXTURE_2D, texture_name, 0 );
    glFramebufferRenderbufferEXT( GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
                                  GL_RENDERBUFFER_EXT, renderbuffer_name );

    glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, 0 );
}

void
RenderToTexture( void )
{
    /* switch to framebuffer object */
    
    SaveFramebufferStatus();
    glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, framebuffer_name );

    /* clear buffer */
    
    glClearColor( 0.2, 0.3, 0.6, 1.0 );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    /* set viewport */

    glViewport( 0, 0, FRAMEBUFFER_WIDTH, FRAMEBUFFER_HEIGHT );

    /* set projection */
    
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective( FRAMEBUFFER_FIELD_OF_VIEW,
                    FRAMEBUFFER_WIDTH / ( double ) FRAMEBUFFER_HEIGHT,
                    FRAMEBUFFER_NEAR_CLIPPING_LENGTH,
                    FRAMEBUFFER_FAR_CLIPPING_LENGTH );
    
    /* set camera */

    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    gluLookAt( FRAMEBUFFER_EYE_X, FRAMEBUFFER_EYE_Y, FRAMEBUFFER_EYE_Z,
               FRAMEBUFFER_TARGET_X, FRAMEBUFFER_TARGET_Y, FRAMEBUFFER_TARGET_Z,
               FRAMEBUFFER_UP_X, FRAMEBUFFER_UP_Y, FRAMEBUFFER_UP_Z );
    
    /* set light (in world coordinates) */

    SetLight();
    
    /* set material */
    
    SetMaterial();
    
    /* put object */
    
    glPushMatrix();
    glRotatef( counter_teapot * 0.1, 0.0, 1.0, 0.0 );
    glRotatef( counter_teapot * 0.07, 1.0, 0.0, 0.0 );
    glScalef( -1.0f, 1.0f, 1.0f );
    glutSolidTeapot( 20.0 );
    glPopMatrix();
    
    /* execute drawing */
    
    glFlush();

#if 1
    unsigned char image[TEXTURE_WIDTH*TEXTURE_HEIGHT*3];
    glGetTexImage( GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
    std::ofstream ofs("test.ppm", std::ios::out | std::ios::trunc | std::ios::binary );
    char buf[10];
    sprintf(buf, "%d %d", TEXTURE_WIDTH, TEXTURE_HEIGHT);
    ofs << "P6" << std::endl << buf << std::endl << "255" << std::endl;
    for (int i=TEXTURE_HEIGHT; i>=0; i--){
        ofs.write((char *)(image+i*TEXTURE_WIDTH*3), TEXTURE_WIDTH*3);
    }
#endif    
    /* switch to default buffer */
    
    glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, 0 );
    RestoreFramebufferStatus();
}

/*****************************************************************************/



/* callback functions */

void
Display( void )
{
    /* rendering teapot on texture */
	
    RenderToTexture();

    /* clear buffer */
    
    glClearColor( 0.0, 0.0, 0.0, 0.0 );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    /* set camera */

    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    gluLookAt( EYE_X, EYE_Y, EYE_Z,
               TARGET_X, TARGET_Y, TARGET_Z,
               UP_X, UP_Y, UP_Z );

    /* set light (in world coordinates) */

    SetLight();

    /* set material */

    glEnable( GL_COLOR_MATERIAL );
    glColor3f( 1.0f, 1.0f, 1.0f );

    /* put object */

    glEnable( GL_TEXTURE_2D );
    glBindTexture( GL_TEXTURE_2D, texture_name );
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
	
    glPushMatrix();
    glRotatef( counter_cube * 0.1, 0.0, 1.0, 0.0 );
    glRotatef( counter_cube * 0.07, 0.0, 0.0, 1.0 );

    DrawCube( 30.0 );

    glPopMatrix();

    glDisable( GL_COLOR_MATERIAL );
    glDisable( GL_TEXTURE_2D );

    /* swap buffers */

    glutSwapBuffers();
}

void
Reshape( int w, int h )
{
    window_width = w;
    window_height = h;

    glViewport( 0, 0, window_width, window_height );
    
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective( FIELD_OF_VIEW, window_width / ( double ) window_height,
                    NEAR_CLIPPING_LENGTH, FAR_CLIPPING_LENGTH );

    glMatrixMode( GL_MODELVIEW );
}

void
Keyboard( unsigned char key, int x, int y )
{
    if ( key == 'q' || key == 3 || key == 27 ) {    /* 3: Ctrl-C, 27: ESC */
        exit( 0 );
    }
}

void
MouseButton( int button, int state, int x, int y )
{
    if ( button == GLUT_LEFT_BUTTON && state == GLUT_DOWN ) {
        is_rotate_cube = 1 - is_rotate_cube;
    }
    if ( button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN ) {}
    if ( button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN ) {
        is_rotate_teapot = 1 - is_rotate_teapot;
    }
}

void
Idle( void )
{
    if ( is_rotate_cube ) {
        counter_cube++;
    }
    if ( is_rotate_teapot ) {
        counter_teapot++;
    }
    if ( is_rotate_cube || is_rotate_teapot ) {
        glutPostRedisplay();
    }
}

/* main */

int
main( int argc, char **argv )
{
    /* initialize variables */
    
    window_width = WINDOW_WIDTH;
    window_height = WINDOW_HEIGHT;
    counter_cube = counter_teapot = 0;
    is_rotate_cube = is_rotate_teapot = 1;

    /* initialize GLUT */
    
    glutInit( &argc, argv );
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );
    glutInitWindowSize( window_width, window_height );
    glutInitWindowPosition( WINDOW_POSITION_X, WINDOW_POSITION_Y );
    glutCreateWindow( argv[ 0 ] );

    /* initialize GLEW */

    glewInit();

    /* set callback functions */

    glutDisplayFunc( Display );
    glutReshapeFunc( Reshape );
    glutKeyboardFunc( Keyboard );
    glutMouseFunc( MouseButton );
    glutIdleFunc( Idle );

    /* initialize OpenGL settings */

    InitMiscGL();

    InitTexture();
    InitRenderbuffer();
    InitFramebuffer();

    /* main loop */
    
    //glutMainLoop();
    RenderToTexture();

    /* never reach here */

    return 0;
}
