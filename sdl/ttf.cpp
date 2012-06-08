#include <iostream>
#include <SDL/SDL.h>
#include <SDL/SDL_ttf.h>

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480
#define BPP 32

int main(int argc, char* argv[]){
    SDL_Surface* image;
    SDL_Rect rect, scr_rect;
    SDL_Event event;
    SDL_Color white = {0xff, 0xff, 0xff};
    TTF_Font* font;
    int exit_prg = 0;

    SDL_Init(SDL_INIT_EVERYTHING);

    TTF_Init();

    SDL_SetVideoMode(WINDOW_WIDTH, WINDOW_HEIGHT, BPP, SDL_HWSURFACE);

    /* フォント読み込み */
    font = TTF_OpenFont("arial.ttf", 24);

    /* 文字作成 */
    image = TTF_RenderUTF8_Blended(font, "abcd", white);

    /* 画像の矩形情報設定 */
    rect.x = 0;
    rect.y = 0;
    rect.w = image->w;
    rect.h = image->h;

    /* 画像配置位置情報の設定 */
    scr_rect.x = 0;
    scr_rect.y = 0;

    /* 描画 */
    /* サーフェスの複写 */
    SDL_BlitSurface(image, &rect, SDL_GetVideoSurface(), &scr_rect);

    /* サーフェスフリップ */
    SDL_Flip(SDL_GetVideoSurface());

    /* イベントループ */
    while(exit_prg == 0){
        if(SDL_PollEvent(&event)){
            switch(event.type){
            case SDL_KEYDOWN:
                exit_prg = 1;
                break;
            default:
                break;
            }
        }
        SDL_Delay(1);
    }

    SDL_FreeSurface(image);

    TTF_CloseFont(font);

    TTF_Quit();

    SDL_Quit();

    return 0;
}
