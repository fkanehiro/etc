#include <stdio.h>
#include <stdlib.h>
#include <SDL.h>
#include <SDL_opengl.h>

GLuint atmark_texture = 0;          // @のテクスチャのテクスチャ名

// x以上の最小の2の冪乗を返す
int ToPow2(int x)
{
    int y = 1;
    while (y < x)
        y <<= 1;
    return y;
}

// テクスチャを読み込んで、テクスチャ名を返す
// 失敗したら0を返す
GLuint LoadTexture(const char* filename)
{
    // 画像を読み込む
    SDL_Surface* surface1 = SDL_LoadBMP(filename);
    if (!surface1) {
        fprintf(stderr, "%sを読み込めませんでした: %s\n", SDL_GetError());
        return 0;
    }

    // フォーマットをRGBAに変換する
    SDL_Surface* surface2 = SDL_CreateRGBSurface(SDL_SWSURFACE, 
                                                 ToPow2(surface1->w), ToPow2(surface1->h), 32,
                                                 0x000000ff, 0x0000ff00, 0x00ff0000, 0xff000000);
    if (!surface2) {
        fprintf(stderr, "変換用サーフィスを確保できませんでした: %s\n", SDL_GetError());
        SDL_FreeSurface(surface1);
        return 0;
    }
    SDL_BlitSurface(surface1, NULL, surface2, NULL);

    // テクスチャを作る
    GLuint name;
    glGenTextures(1, &name);
    glBindTexture(GL_TEXTURE_2D, name);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, 4, surface2->w, surface2->h, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, surface2->pixels);

    // 後片付け
    SDL_FreeSurface(surface2);
    SDL_FreeSurface(surface1);

    return name;
}

bool initializeSDL(int flags) {
	// SDLを初期化する
	if (SDL_Init(flags) < 0) {
		fprintf(stderr, "%s\n", SDL_GetError());
		return false;
	}
	atexit(SDL_Quit);

	return true;
}

bool initializeVideo(int width, int height, int flags) {
	// ビデオモードの設定をする
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	if (0 == SDL_SetVideoMode(width, height, 0, flags)) {
		fprintf(stderr, "%s\n", SDL_GetError());
		return false;
	}

	return true;
}

bool initializeOpenGL(int width, int height) {
	if (!initializeVideo(width, height, SDL_OPENGL)) {
		return false;
	}

	// ビューポートを設定する
	glViewport(0, 0, width, height);
	glClearColor( 1.0, 0.5, 0.0, 0.0);
	glEnable(GL_DEPTH_TEST);

	// 射影行列を設定する
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (GLdouble) width / (GLdouble) height, 2.0, 200.0);

	// 照明を設定する
	static GLfloat position[] = {-10.0f, 10.0f, 10.0f, 1.0f};
	static GLfloat ambient [] = { 1.0f, 1.0f, 1.0f, 1.0f};
	static GLfloat diffuse [] = { 1.0f, 1.0f, 1.0f, 1.0f};
	static GLfloat specular[] = { 0.0f, 0.0f, 0.0f, 0.0f};
	glLightfv(GL_LIGHT0, GL_POSITION, position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

        // テクスチャを読み込む
        atmark_texture = LoadTexture("atmark.bmp");

	return true;
}

void draw() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// 視点を設定する
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt( 0.0f,  0.0f,-30.0f,
	           0.0f,  0.0f, 0.0f,
	           0.0f, -1.0f, 0.0f);

	// マテリアルを設定する
	GLfloat ambient  [] = { 0.1f, 0.1f, 0.1f, 1.0f};
	GLfloat diffuse  [] = { 1.0f, 0.0f, 0.0f, 1.0f};
	GLfloat specular [] = { 1.0f, 1.0f, 1.0f, 1.0f};
	GLfloat shininess[] = { 0.0f};
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, shininess);

	// 球を描画する
	GLUquadric* quadric=gluNewQuadric();
	gluSphere(quadric, 10.0, 30, 30);
	gluDeleteQuadric(quadric);
}

void Draw()
{
    glClear(GL_COLOR_BUFFER_BIT);

    // テクスチャを描画する
    glBindTexture(GL_TEXTURE_2D, atmark_texture);
    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glTexCoord2f(1.0f, 0.0f);
    glVertex3f(64.0f, 0.0f, 0.0f);
    glTexCoord2f(1.0f, 1.0f);
    glVertex3f(64.0f, 64.0f, 0.0f);
    glTexCoord2f(0.0f, 1.0f);
    glVertex3f(0.0f, 64.0f, 0.0f);
    glEnd();
}


int main(int argc, char** args) {
	if (!initializeSDL(SDL_INIT_VIDEO)) {
		return 1;
	}

	if (!initializeOpenGL(400, 400)) {
		return 1;
	}

	while (true) {
		// イベントを処理する
		SDL_Event event;
		while (SDL_PollEvent(&event)) {
			if (event.type == SDL_QUIT) {
				return 0;
			}
		}

		draw();
		//Draw();

		SDL_GL_SwapBuffers();
	}
}
