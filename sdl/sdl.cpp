#include <SDL.h>

int main(int argc,char *argv[])
{
    /* 初期化 */
    if(SDL_Init(SDL_INIT_VIDEO)<0) {
        fprintf(stderr,"failed to initialize SDL.\n");
        return -1;
    }

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER,1);
    SDL_Surface *screen;
    screen=SDL_SetVideoMode(640,480,32,SDL_OPENGL);
    if(!screen) {
        fprintf(stderr,"failed to set video mode to 640x480x16.\n");
        SDL_Quit();
        exit(1);
    }
    SDL_WM_SetCaption("OpenGL Test Window", NULL);

    /* ループ */
    while(1) {
        SDL_Event event;
        while(SDL_PollEvent(&event)){
            switch(event.type){
            case SDL_QUIT:      //ウィンドウのボタン
                goto QUIT;
            case SDL_KEYDOWN:   //escキー
                if(event.key.keysym.sym==SDLK_ESCAPE) goto QUIT;
            }
#if 0
            Uint8 *keys=SDL_GetKeyState(NULL);
            if(keys[SDLK_RETURN]==SDL_PRESSED)
                printf("return key is pressed.\n");
#endif
        }
    }

    /* 終了 */
  QUIT:
    SDL_Quit();
    return 0;
}

