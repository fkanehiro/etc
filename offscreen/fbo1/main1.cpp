#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#if defined(WIN32)
//#  pragma comment(linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"")
#  include "glut.h"
#  include "glext.h"
PFNGLMULTTRANSPOSEMATRIXDPROC glMultTransposeMatrixd;
PFNGLGENFRAMEBUFFERSEXTPROC glGenFramebuffersEXT;
PFNGLBINDFRAMEBUFFEREXTPROC glBindFramebufferEXT;
PFNGLFRAMEBUFFERTEXTURE2DEXTPROC glFramebufferTexture2DEXT;
PFNGLGENRENDERBUFFERSEXTPROC glGenRenderbuffersEXT;
PFNGLBINDRENDERBUFFEREXTPROC glBindRenderbufferEXT;
PFNGLRENDERBUFFERSTORAGEEXTPROC glRenderbufferStorageEXT;
PFNGLFRAMEBUFFERRENDERBUFFEREXTPROC glFramebufferRenderbufferEXT;
PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC glCheckFramebufferStatusEXT;
#elif defined(__APPLE__) || defined(MACOSX)
#  include <GLUT/glut.h>
#else
#  define GL_GLEXT_PROTOTYPES
#  include <GL/glut.h>
#endif
#include "glsl.h"

#define CHECK_FRAMEBUFFER_STATUS() \
{ \
  GLenum status; \
  status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT); \
  switch(status) { \
  case GL_FRAMEBUFFER_COMPLETE_EXT: \
    break; \
  case GL_FRAMEBUFFER_UNSUPPORTED_EXT: \
    fprintf(stderr, "choose different formats\n"); \
    break; \
  default: \
    fprintf(stderr, "programming error; will fail on all hardware: %04x\n", status); \
  } \
}

/*
** �����������֥�������
*/
static GLuint vertShader;
static GLuint fragShader;
static GLuint gl2Program;

/*
** ����
*/
static const GLfloat lightpos[] = { 4.0, 9.0, 5.0, 1.0 }; /* ���֡������������� */
static const GLfloat lightcol[] = { 1.0, 1.0, 1.0, 1.0 }; /* ľ�ܸ����١������� */
static const GLfloat lightdim[] = { 0.2, 0.2, 0.2, 1.0 }; /* ����γȻ�ȿ�Ͷ��� */
static const GLfloat lightblk[] = { 0.0, 0.0, 0.0, 1.0 }; /* ����ζ���ȿ�Ͷ��� */
static const GLfloat lightamb[] = { 0.1, 0.1, 0.1, 1.0 }; /* �Ķ������١������� */

/*
** �ƥ�������
*/
#define TEXWIDTH  1024                                    /* �ƥ�������������� */
#define TEXHEIGHT 1024                                    /* �ƥ�������ι⤵�� */

/*
** �ƥ������㥪�֥������ȡ��ե졼��Хåե����֥�������
*/
static GLuint tex, fb, rb;

/*
** �����
*/
static void init(void)
{
  /* ���������ץ����Υ���ѥ��롿��󥯷�̤������ѿ� */
  GLint compiled, linked;
  
#if defined(WIN32)
  glGenFramebuffersEXT =
    (PFNGLGENFRAMEBUFFERSEXTPROC)wglGetProcAddress("glGenFramebuffersEXT");
  glBindFramebufferEXT =
    (PFNGLBINDFRAMEBUFFEREXTPROC)wglGetProcAddress("glBindFramebufferEXT");
  glFramebufferTexture2DEXT =
    (PFNGLFRAMEBUFFERTEXTURE2DEXTPROC)wglGetProcAddress("glFramebufferTexture2DEXT");
  glGenRenderbuffersEXT =
    (PFNGLGENRENDERBUFFERSEXTPROC)wglGetProcAddress("glGenRenderbuffersEXT");
  glBindRenderbufferEXT =
    (PFNGLBINDRENDERBUFFEREXTPROC)wglGetProcAddress("glBindRenderbufferEXT");
  glRenderbufferStorageEXT =
    (PFNGLRENDERBUFFERSTORAGEEXTPROC)wglGetProcAddress("glRenderbufferStorageEXT");
  glFramebufferRenderbufferEXT =
    (PFNGLFRAMEBUFFERRENDERBUFFEREXTPROC)wglGetProcAddress("glFramebufferRenderbufferEXT");
  glCheckFramebufferStatusEXT =
    (PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC)wglGetProcAddress("glCheckFramebufferStatusEXT");
#endif
  
  /* �ƥ������㥪�֥������Ȥ��������Ʒ�礹�� */
  glGenTextures(1, &tex);
  glBindTexture(GL_TEXTURE_2D, tex);

  /* �ƥ�������γ������ */
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, TEXWIDTH, TEXHEIGHT, 0,
    GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, 0);
  
  /* �ƥ����������硦�̾�������ˡ�λ��� */
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  
  /* �ƥ�������η����֤���ˡ�λ��� */
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  
  /* �񤭹���ݥꥴ��Υƥ��������ɸ�ͤΣҤȥƥ�������Ȥ���Ӥ�Ԥ��褦�ˤ��� */
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
  
  /* �⤷�Ҥ��ͤ��ƥ���������Ͱʲ��ʤ鿿�ʤĤޤ������� */
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
  
  /* ��Ӥη�̤����ͤȤ������� */
  glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_LUMINANCE);
  
#if 0
  /* �ƥ��������ɸ�˻�����ɸ�Ϥˤ�����ʪ�Τκ�ɸ�ͤ��Ѥ��� */
  glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
  glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
  glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
  glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);

  /* ���������ƥ��������ɸ�򤽤Τޤ� (S, T, R, Q) �˻Ȥ� */
  static const GLdouble genfunc[][4] = {
    { 1.0, 0.0, 0.0, 0.0 },
    { 0.0, 1.0, 0.0, 0.0 },
    { 0.0, 0.0, 1.0, 0.0 },
    { 0.0, 0.0, 0.0, 1.0 },
  };
  glTexGendv(GL_S, GL_EYE_PLANE, genfunc[0]);
  glTexGendv(GL_T, GL_EYE_PLANE, genfunc[1]);
  glTexGendv(GL_R, GL_EYE_PLANE, genfunc[2]);
  glTexGendv(GL_Q, GL_EYE_PLANE, genfunc[3]);
#endif

  /* �ƥ������㥪�֥������Ȥη��������� */
  glBindTexture(GL_TEXTURE_2D, 0);

  /* �ե졼��Хåե����֥������Ȥ��������Ʒ�礹�� */
  glGenFramebuffersEXT(1, &fb);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb);

  /* �ե졼��Хåե����֥������Ȥ˥ǥץ��Хåե��ѤΥƥ���������礹�� */
  glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
    GL_TEXTURE_2D, tex, 0);

  /* ���顼�Хåե���̵���Τ��ɤ߽񤭤��ʤ� */
  glDrawBuffer(GL_NONE);
  glReadBuffer(GL_NONE);

  CHECK_FRAMEBUFFER_STATUS();

  /* �ե졼��Хåե����֥������Ȥη��������� */
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

  /* ������� */
  glClearColor(0.3, 0.3, 1.0, 1.0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  
  /* �����ν������ */
  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_AMBIENT, lightamb);

  /* GLSL �ν���� */
  if (glslInit()) exit(1);
  
  /* �����������֥������Ȥκ��� */
  vertShader = glCreateShader(GL_VERTEX_SHADER);
  fragShader = glCreateShader(GL_FRAGMENT_SHADER);
  
  /* ���������Υ������ץ������ɤ߹��� */
  if (readShaderSource(vertShader, "shadow.vert")) exit(1);
  if (readShaderSource(fragShader, "shadow.frag")) exit(1);
  
  /* �С��ƥå������������Υ������ץ����Υ���ѥ��� */
  glCompileShader(vertShader);
  glGetShaderiv(vertShader, GL_COMPILE_STATUS, &compiled);
  printShaderInfoLog(vertShader);
  if (compiled == GL_FALSE) {
    fprintf(stderr, "Compile error in vertex shader.\n");
    exit(1);
  }
  
  /* �ե饰���ȥ��������Υ������ץ����Υ���ѥ��� */
  glCompileShader(fragShader);
  glGetShaderiv(fragShader, GL_COMPILE_STATUS, &compiled);
  printShaderInfoLog(fragShader);
  if (compiled == GL_FALSE) {
    fprintf(stderr, "Compile error in fragment shader.\n");
    exit(1);
  }
  
  /* �ץ���४�֥������Ȥκ��� */
  gl2Program = glCreateProgram();
  
  /* �����������֥������ȤΥ��������ץ����ؤ���Ͽ */
  glAttachShader(gl2Program, vertShader);
  glAttachShader(gl2Program, fragShader);
  
  /* �����������֥������Ȥκ�� */
  glDeleteShader(vertShader);
  glDeleteShader(fragShader);
  
  /* ���������ץ����Υ�� */
  glLinkProgram(gl2Program);
  glGetProgramiv(gl2Program, GL_LINK_STATUS, &linked);
  printProgramInfoLog(gl2Program);
  if (linked == GL_FALSE) {
    fprintf(stderr, "Link error.\n");
    exit(1);
  }
  
  /* ���������ץ�����Ŭ�� */
  glUseProgram(gl2Program);

  /* �ƥ��������˥åȣ�����ꤹ�� */
  glUniform1i(glGetUniformLocation(gl2Program, "texture"), 0);
  
#if defined(WIN32)
  glMultTransposeMatrixd =
    (PFNGLMULTTRANSPOSEMATRIXDPROC)wglGetProcAddress("glMultTransposeMatrixd");
#endif
}


/****************************
** GLUT �Υ�����Хå��ؿ� **
****************************/

/* �ȥ�å��ܡ�������Ѵؿ������ */
#include "trackball.h"

/* ������������ؿ������ */
#include "scene.h"

/* ���˥᡼�����Υ������� */
#define FRAMES 600

static void display(void)
{
  GLint viewport[4];       /* �ӥ塼�ݡ��Ȥ���¸�ѡ������� */
  GLdouble modelview[16];  /* ��ǥ�ӥ塼�Ѵ��������¸�� */
  GLdouble projection[16]; /* Ʃ���Ѵ��������¸�ѡ������� */
  static int frame = 0;    /* �ե졼����Υ�����ȡ������� */
  double t = (double)frame / (double)FRAMES; /* �в���֡� */

  if (++frame >= FRAMES) frame = 0;

  /*
  ** �裱���ƥåס��ǥץ��ƥ�������κ���
  */
  
  /* �ե졼��Хåե����֥������ȤؤΥ�����󥰳��� */
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb);

  /* �ǥץ��Хåե��򥯥ꥢ���� */
  glClear(GL_DEPTH_BUFFER_BIT);
  
  /* ���ߤΥӥ塼�ݡ��Ȥ���¸���Ƥ��� */
  glGetIntegerv(GL_VIEWPORT, viewport);
  
  /* �ӥ塼�ݡ��Ȥ�ƥ�������Υ����������ꤹ�� */
  glViewport(0, 0, TEXWIDTH, TEXHEIGHT);
  
  /* ���ߤ�Ʃ���Ѵ��������¸���Ƥ��� */
  glGetDoublev(GL_PROJECTION_MATRIX, projection);
  
  /* Ʃ���Ѵ������ñ�̹�������ꤹ�� */
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  
  /* �������֤�����Ȥ������󤬻���˼��ޤ�褦��ǥ�ӥ塼�Ѵ���������ꤹ�� */
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluPerspective(40.0, (GLdouble)TEXWIDTH / (GLdouble)TEXHEIGHT, 1.0, 20.0);
  gluLookAt(lightpos[0], lightpos[1], lightpos[2], 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

  /* ���ꤷ����ǥ�ӥ塼�Ѵ��������¸���Ƥ��� */
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);

  /* �ǥץ��Хåե������Ƥ������������Τǥե졼��Хåե��ˤϽ񤭹��ޤʤ� */
  glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

  /* �������äƱ����դ������פʤΤǥ饤�ƥ��󥰤򥪥դˤ��� */
  glDisable(GL_LIGHTING);

  /* �ǥץ��Хåե��ˤ����̤Υݥꥴ��α��Ԥ���Ͽ����褦�ˤ��� */
  glCullFace(GL_FRONT);

  /* ����������褹�� */
  scene(t);

  /* �ե졼��Хåե����֥������ȤؤΥ�����󥰽�λ */
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

#if 0
  /* �ǥץ��Хåե������Ƥ�ƥ�����������ž������ */
  glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, TEXWIDTH, TEXHEIGHT);
#endif

  /* �̾�������������᤹ */
  glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixd(projection);
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glEnable(GL_LIGHTING);
  glCullFace(GL_BACK);
  
  /*
  ** �裲���ƥåס����Τ�����
  */
  
  /* �ե졼��Хåե��ȥǥץ��Хåե��򥯥ꥢ���� */
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  /* ��ǥ�ӥ塼�Ѵ���������� */
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  /* �����ΰ��֤����ꤹ���ʪ�Τ�������˰�ư�����*/
  glTranslated(0.0, 0.0, -10.0);
  
  /* �ȥ�å��ܡ��뼰�β�ž��Ϳ���� */
  glMultMatrixd(trackballRotation());
  
  /* �����ΰ��֤����ꤹ�� */
  glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
  
  /* �ƥ��������Ѵ���������ꤹ�� */
  glMatrixMode(GL_TEXTURE);
  glLoadIdentity();
  
  /* �ƥ��������ɸ�� [-1,1] ���ϰϤ� [0,1] ���ϰϤ˼���� */
  glTranslated(0.5, 0.5, 0.5);
  glScaled(0.5, 0.5, 0.5);
  
  /* �ƥ�������Υ�ǥ�ӥ塼�Ѵ������Ʃ���Ѵ�������Ѥ򤫤��� */
  glMultMatrixd(modelview);
  
  /* ���ߤΥ�ǥ�ӥ塼�Ѵ��ε��Ѵ��򤫤��Ƥ��� */
  glMultTransposeMatrixd(trackballRotation());
  glTranslated(0.0, 0.0, 10.0);
  
  /* ��ǥ�ӥ塼�Ѵ�������᤹ */
  glMatrixMode(GL_MODELVIEW);
  
  /* �ƥ������㥪�֥������Ȥ��礹�� */
  glBindTexture(GL_TEXTURE_2D, tex);

  /* �ƥ�������ޥåԥ󥰤ȥƥ��������ɸ�μ�ư������ͭ���ˤ��� */
  glEnable(GL_TEXTURE_2D);
#if 0
  glEnable(GL_TEXTURE_GEN_S);
  glEnable(GL_TEXTURE_GEN_T);
  glEnable(GL_TEXTURE_GEN_R);
  glEnable(GL_TEXTURE_GEN_Q);
#endif
  
  /* ���������뤵����������ʬ�Ǥ����뤵������ */
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightcol);
  glLightfv(GL_LIGHT0, GL_SPECULAR, lightcol);
  
  /* ����������褹�� */
  scene(t);
  
  /* �ƥ�������ޥåԥ󥰤ȥƥ��������ɸ�μ�ư������̵���ˤ��� */
#if 0
  glDisable(GL_TEXTURE_GEN_S);
  glDisable(GL_TEXTURE_GEN_T);
  glDisable(GL_TEXTURE_GEN_R);
  glDisable(GL_TEXTURE_GEN_Q);
#endif
  glDisable(GL_TEXTURE_2D);
  
  /* �ƥ������㥪�֥������Ȥη��������� */
  glBindTexture(GL_TEXTURE_2D, 0);
  
  /* ���֥�Хåե���� */
  glutSwapBuffers();
}

static void resize(int w, int h)
{
#if 0
  /* ������ɥ��������ν̾������¤��� */
  if (w < TEXWIDTH || h < TEXHEIGHT) {
    if (w < TEXWIDTH) w = TEXWIDTH;
    if (h < TEXHEIGHT) h = TEXHEIGHT;
    glutReshapeWindow(w, h);
  }
#endif

  /* �ȥ�å��ܡ��뤹���ϰ� */
  trackballRegion(w, h);
  
  /* ������ɥ����Τ�ӥ塼�ݡ��Ȥˤ��� */
  glViewport(0, 0, w, h);
  
  /* Ʃ���Ѵ�����λ��� */
  glMatrixMode(GL_PROJECTION);
  
  /* Ʃ���Ѵ�����ν���� */
  glLoadIdentity();
  gluPerspective(40.0, (double)w / (double)h, 1.0, 100.0);
}

static void idle(void)
{
  /* ���̤������ؤ� */
  glutPostRedisplay();
}

static void mouse(int button, int state, int x, int y)
{
  switch (button) {
  case GLUT_LEFT_BUTTON:
    switch (state) {
    case GLUT_DOWN:
      /* �ȥ�å��ܡ��볫�� */
      trackballStart(x, y);
      break;
    case GLUT_UP:
      /* �ȥ�å��ܡ������ */
      trackballStop(x, y);
      break;
    default:
      break;
    }
    break;
    default:
      break;
  }
}

static void motion(int x, int y)
{
  /* �ȥ�å��ܡ����ư */
  trackballMotion(x, y);
}

static void keyboard(unsigned char key, int x, int y)
{
  switch (key) {
  case 'q':
  case 'Q':
  case '\033':
    /* ESC �� q �� Q �򥿥��פ����齪λ */
    exit(0);
  default:
    break;
  }
}

/*
** �ᥤ��ץ����
*/
int main(int argc, char *argv[])
{
  glutInit(&argc, argv);
#if 0
  glutInitWindowSize(TEXWIDTH, TEXHEIGHT);
#endif
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
  glutCreateWindow(argv[0]);
  glutDisplayFunc(display);
  glutReshapeFunc(resize);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutKeyboardFunc(keyboard);
  glutIdleFunc(idle);
  init();
  glutMainLoop();
  return 0;
}
