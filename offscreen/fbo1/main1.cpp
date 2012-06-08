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
** シェーダオブジェクト
*/
static GLuint vertShader;
static GLuint fragShader;
static GLuint gl2Program;

/*
** 光源
*/
static const GLfloat lightpos[] = { 4.0, 9.0, 5.0, 1.0 }; /* 位置　　　　　　　 */
static const GLfloat lightcol[] = { 1.0, 1.0, 1.0, 1.0 }; /* 直接光強度　　　　 */
static const GLfloat lightdim[] = { 0.2, 0.2, 0.2, 1.0 }; /* 影内の拡散反射強度 */
static const GLfloat lightblk[] = { 0.0, 0.0, 0.0, 1.0 }; /* 影内の鏡面反射強度 */
static const GLfloat lightamb[] = { 0.1, 0.1, 0.1, 1.0 }; /* 環境光強度　　　　 */

/*
** テクスチャ
*/
#define TEXWIDTH  1024                                    /* テクスチャの幅　　 */
#define TEXHEIGHT 1024                                    /* テクスチャの高さ　 */

/*
** テクスチャオブジェクト・フレームバッファオブジェクト
*/
static GLuint tex, fb, rb;

/*
** 初期化
*/
static void init(void)
{
  /* シェーダプログラムのコンパイル／リンク結果を得る変数 */
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
  
  /* テクスチャオブジェクトを生成して結合する */
  glGenTextures(1, &tex);
  glBindTexture(GL_TEXTURE_2D, tex);

  /* テクスチャの割り当て */
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, TEXWIDTH, TEXHEIGHT, 0,
    GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, 0);
  
  /* テクスチャを拡大・縮小する方法の指定 */
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  
  /* テクスチャの繰り返し方法の指定 */
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  
  /* 書き込むポリゴンのテクスチャ座標値のＲとテクスチャとの比較を行うようにする */
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
  
  /* もしＲの値がテクスチャの値以下なら真（つまり日向） */
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
  
  /* 比較の結果を輝度値として得る */
  glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_LUMINANCE);
  
#if 0
  /* テクスチャ座標に視点座標系における物体の座標値を用いる */
  glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
  glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
  glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
  glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);

  /* 生成したテクスチャ座標をそのまま (S, T, R, Q) に使う */
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

  /* テクスチャオブジェクトの結合を解除する */
  glBindTexture(GL_TEXTURE_2D, 0);

  /* フレームバッファオブジェクトを生成して結合する */
  glGenFramebuffersEXT(1, &fb);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb);

  /* フレームバッファオブジェクトにデプスバッファ用のテクスチャを結合する */
  glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
    GL_TEXTURE_2D, tex, 0);

  /* カラーバッファが無いので読み書きしない */
  glDrawBuffer(GL_NONE);
  glReadBuffer(GL_NONE);

  CHECK_FRAMEBUFFER_STATUS();

  /* フレームバッファオブジェクトの結合を解除する */
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

  /* 初期設定 */
  glClearColor(0.3, 0.3, 1.0, 1.0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  
  /* 光源の初期設定 */
  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_AMBIENT, lightamb);

  /* GLSL の初期化 */
  if (glslInit()) exit(1);
  
  /* シェーダオブジェクトの作成 */
  vertShader = glCreateShader(GL_VERTEX_SHADER);
  fragShader = glCreateShader(GL_FRAGMENT_SHADER);
  
  /* シェーダのソースプログラムの読み込み */
  if (readShaderSource(vertShader, "shadow.vert")) exit(1);
  if (readShaderSource(fragShader, "shadow.frag")) exit(1);
  
  /* バーテックスシェーダのソースプログラムのコンパイル */
  glCompileShader(vertShader);
  glGetShaderiv(vertShader, GL_COMPILE_STATUS, &compiled);
  printShaderInfoLog(vertShader);
  if (compiled == GL_FALSE) {
    fprintf(stderr, "Compile error in vertex shader.\n");
    exit(1);
  }
  
  /* フラグメントシェーダのソースプログラムのコンパイル */
  glCompileShader(fragShader);
  glGetShaderiv(fragShader, GL_COMPILE_STATUS, &compiled);
  printShaderInfoLog(fragShader);
  if (compiled == GL_FALSE) {
    fprintf(stderr, "Compile error in fragment shader.\n");
    exit(1);
  }
  
  /* プログラムオブジェクトの作成 */
  gl2Program = glCreateProgram();
  
  /* シェーダオブジェクトのシェーダプログラムへの登録 */
  glAttachShader(gl2Program, vertShader);
  glAttachShader(gl2Program, fragShader);
  
  /* シェーダオブジェクトの削除 */
  glDeleteShader(vertShader);
  glDeleteShader(fragShader);
  
  /* シェーダプログラムのリンク */
  glLinkProgram(gl2Program);
  glGetProgramiv(gl2Program, GL_LINK_STATUS, &linked);
  printProgramInfoLog(gl2Program);
  if (linked == GL_FALSE) {
    fprintf(stderr, "Link error.\n");
    exit(1);
  }
  
  /* シェーダプログラムの適用 */
  glUseProgram(gl2Program);

  /* テクスチャユニット０を指定する */
  glUniform1i(glGetUniformLocation(gl2Program, "texture"), 0);
  
#if defined(WIN32)
  glMultTransposeMatrixd =
    (PFNGLMULTTRANSPOSEMATRIXDPROC)wglGetProcAddress("glMultTransposeMatrixd");
#endif
}


/****************************
** GLUT のコールバック関数 **
****************************/

/* トラックボール処理用関数の宣言 */
#include "trackball.h"

/* シーンを描く関数の宣言 */
#include "scene.h"

/* アニメーションのサイクル */
#define FRAMES 600

static void display(void)
{
  GLint viewport[4];       /* ビューポートの保存用　　　　 */
  GLdouble modelview[16];  /* モデルビュー変換行列の保存用 */
  GLdouble projection[16]; /* 透視変換行列の保存用　　　　 */
  static int frame = 0;    /* フレーム数のカウント　　　　 */
  double t = (double)frame / (double)FRAMES; /* 経過時間　 */

  if (++frame >= FRAMES) frame = 0;

  /*
  ** 第１ステップ：デプステクスチャの作成
  */
  
  /* フレームバッファオブジェクトへのレンダリング開始 */
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb);

  /* デプスバッファをクリアする */
  glClear(GL_DEPTH_BUFFER_BIT);
  
  /* 現在のビューポートを保存しておく */
  glGetIntegerv(GL_VIEWPORT, viewport);
  
  /* ビューポートをテクスチャのサイズに設定する */
  glViewport(0, 0, TEXWIDTH, TEXHEIGHT);
  
  /* 現在の透視変換行列を保存しておく */
  glGetDoublev(GL_PROJECTION_MATRIX, projection);
  
  /* 透視変換行列を単位行列に設定する */
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  
  /* 光源位置を視点としシーンが視野に収まるようモデルビュー変換行列を設定する */
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluPerspective(40.0, (GLdouble)TEXWIDTH / (GLdouble)TEXHEIGHT, 1.0, 20.0);
  gluLookAt(lightpos[0], lightpos[1], lightpos[2], 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

  /* 設定したモデルビュー変換行列を保存しておく */
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);

  /* デプスバッファの内容だけを取得するのでフレームバッファには書き込まない */
  glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

  /* したがって陰影付けも不要なのでライティングをオフにする */
  glDisable(GL_LIGHTING);

  /* デプスバッファには背面のポリゴンの奥行きを記録するようにする */
  glCullFace(GL_FRONT);

  /* シーンを描画する */
  scene(t);

  /* フレームバッファオブジェクトへのレンダリング終了 */
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

#if 0
  /* デプスバッファの内容をテクスチャメモリに転送する */
  glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, TEXWIDTH, TEXHEIGHT);
#endif

  /* 通常の描画の設定に戻す */
  glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixd(projection);
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glEnable(GL_LIGHTING);
  glCullFace(GL_BACK);
  
  /*
  ** 第２ステップ：全体の描画
  */
  
  /* フレームバッファとデプスバッファをクリアする */
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  /* モデルビュー変換行列の設定 */
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  /* 視点の位置を設定する（物体の方を奥に移動する）*/
  glTranslated(0.0, 0.0, -10.0);
  
  /* トラックボール式の回転を与える */
  glMultMatrixd(trackballRotation());
  
  /* 光源の位置を設定する */
  glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
  
  /* テクスチャ変換行列を設定する */
  glMatrixMode(GL_TEXTURE);
  glLoadIdentity();
  
  /* テクスチャ座標の [-1,1] の範囲を [0,1] の範囲に収める */
  glTranslated(0.5, 0.5, 0.5);
  glScaled(0.5, 0.5, 0.5);
  
  /* テクスチャのモデルビュー変換行列と透視変換行列の積をかける */
  glMultMatrixd(modelview);
  
  /* 現在のモデルビュー変換の逆変換をかけておく */
  glMultTransposeMatrixd(trackballRotation());
  glTranslated(0.0, 0.0, 10.0);
  
  /* モデルビュー変換行列に戻す */
  glMatrixMode(GL_MODELVIEW);
  
  /* テクスチャオブジェクトを結合する */
  glBindTexture(GL_TEXTURE_2D, tex);

  /* テクスチャマッピングとテクスチャ座標の自動生成を有効にする */
  glEnable(GL_TEXTURE_2D);
#if 0
  glEnable(GL_TEXTURE_GEN_S);
  glEnable(GL_TEXTURE_GEN_T);
  glEnable(GL_TEXTURE_GEN_R);
  glEnable(GL_TEXTURE_GEN_Q);
#endif
  
  /* 光源の明るさを日向の部分での明るさに設定 */
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightcol);
  glLightfv(GL_LIGHT0, GL_SPECULAR, lightcol);
  
  /* シーンを描画する */
  scene(t);
  
  /* テクスチャマッピングとテクスチャ座標の自動生成を無効にする */
#if 0
  glDisable(GL_TEXTURE_GEN_S);
  glDisable(GL_TEXTURE_GEN_T);
  glDisable(GL_TEXTURE_GEN_R);
  glDisable(GL_TEXTURE_GEN_Q);
#endif
  glDisable(GL_TEXTURE_2D);
  
  /* テクスチャオブジェクトの結合を解除する */
  glBindTexture(GL_TEXTURE_2D, 0);
  
  /* ダブルバッファリング */
  glutSwapBuffers();
}

static void resize(int w, int h)
{
#if 0
  /* ウィンドウサイズの縮小を制限する */
  if (w < TEXWIDTH || h < TEXHEIGHT) {
    if (w < TEXWIDTH) w = TEXWIDTH;
    if (h < TEXHEIGHT) h = TEXHEIGHT;
    glutReshapeWindow(w, h);
  }
#endif

  /* トラックボールする範囲 */
  trackballRegion(w, h);
  
  /* ウィンドウ全体をビューポートにする */
  glViewport(0, 0, w, h);
  
  /* 透視変換行列の指定 */
  glMatrixMode(GL_PROJECTION);
  
  /* 透視変換行列の初期化 */
  glLoadIdentity();
  gluPerspective(40.0, (double)w / (double)h, 1.0, 100.0);
}

static void idle(void)
{
  /* 画面の描き替え */
  glutPostRedisplay();
}

static void mouse(int button, int state, int x, int y)
{
  switch (button) {
  case GLUT_LEFT_BUTTON:
    switch (state) {
    case GLUT_DOWN:
      /* トラックボール開始 */
      trackballStart(x, y);
      break;
    case GLUT_UP:
      /* トラックボール停止 */
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
  /* トラックボール移動 */
  trackballMotion(x, y);
}

static void keyboard(unsigned char key, int x, int y)
{
  switch (key) {
  case 'q':
  case 'Q':
  case '\033':
    /* ESC か q か Q をタイプしたら終了 */
    exit(0);
  default:
    break;
  }
}

/*
** メインプログラム
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
