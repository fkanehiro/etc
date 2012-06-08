#include <SDL.h>
#include <SDL_opengl.h>
#include <stdio.h>

const int SCREEN_WIDTH = 640;       // 画面の幅
const int SCREEN_HEIGHT = 480;      // 画面の高さ

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

// 初期化する
// 成功したときは0を、失敗したときは-1を返す
int Initialize()
{
    // SDLを初期化する
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDLの初期化に失敗しました: %s\n", SDL_GetError());
        return -1;
    }

    // タイトルを設定する
    SDL_WM_SetCaption("OpenGL Sample", NULL);

    // 画面を初期化する
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_Surface* screen = SDL_SetVideoMode(SCREEN_WIDTH, SCREEN_HEIGHT, 32, SDL_OPENGL);
    if (screen == NULL) {
        fprintf(stderr, "画面の初期化に失敗しました: %s\n", SDL_GetError());
        SDL_Quit();
        return -1;
    }

    // 画面をクリアする色を指定する
    glClearColor(1.0f, 0.5f, 0.0f, 0.0f);

    // 射影マトリクスを設定する
    glOrtho(0.0, SCREEN_WIDTH, SCREEN_HEIGHT, 0.0, -1.0, 1.0);

    // テクスチャを有効にする
    glEnable(GL_TEXTURE_2D);

    // テクスチャを読み込む
    atmark_texture = LoadTexture("atmark.bmp");
    if (atmark_texture == 0) {
        return -1;
    }

    return 0;
}

// 更新する
void Update()
{
}

// 描画する
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

    // 更新を画面に反映する
    SDL_GL_SwapBuffers();
}

// メインループ
void MainLoop()
{
    SDL_Event event;
    double next_frame = SDL_GetTicks();
    double wait = 1000.0 / 60;

    for (;;) {
        // すべてのイベントを処理する
        while (SDL_PollEvent(&event)) {
            // QUIT イベントが発生するか、ESC キーが押されたら終了する
            if ((event.type == SDL_QUIT) ||
                (event.type == SDL_KEYUP && event.key.keysym.sym == SDLK_ESCAPE))
                return;
        }
        // 1秒間に60回Updateされるようにする
        if (SDL_GetTicks() >= next_frame) {
            Update();
            // 時間がまだあるときはDrawする
            if (SDL_GetTicks() < next_frame + wait)
                Draw();
            next_frame += wait;
            SDL_Delay(0);
        }
    }
}

// 終了処理を行なう
void Finalize()
{
    // テクスチャを削除する
    glDeleteTextures(1, &atmark_texture);
    // SDLを終了する
    SDL_Quit();
}

// メイン関数
int main(int argc, char* argv[])
{
    if (Initialize() < 0)
        return 1;
    MainLoop();
    Finalize();
    return 0;
}

