#include <stdio.h>

int main(void)
{
   /* 背景色  \x1b[0m と \x1b[49m はデフォルトに戻す*/
   printf("\x1b[40m 背景が黒 \x1b[0m\n");
   printf("\x1b[41m 背景が赤 \x1b[0m\n");
   printf("\x1b[42m 背景が緑 \x1b[0m\n");
   printf("\x1b[43m 背景が黄 \x1b[0m\n");
   printf("\x1b[44m 背景が青 \x1b[0m\n");
   printf("\x1b[45m 背景が紫 \x1b[49m\n");
   printf("\x1b[46m 背景が水 \x1b[49m\n");
   printf("\x1b[47m 背景が白 \x1b[49m\n");
   printf("\n");

   /* 前景色の指定  \x1b[0m と \x1b[39m はデフォルとに戻す*/
   printf("\x1b[30m 前景色が黒 \x1b[0m\n");
   printf("\x1b[31m 前景色が赤 \x1b[0m\n");
   printf("\x1b[32m 前景色が緑 \x1b[0m\n");
   printf("\x1b[33m 前景色が黄 \x1b[0m\n");
   printf("\x1b[34m 前景色が青 \x1b[0m\n");
   printf("\x1b[35m 前景色が紫 \x1b[39m\n");
   printf("\x1b[36m 前景色が水 \x1b[39m\n");
   printf("\x1b[37m 前景色が白 \x1b[39m\n");
   printf("\n");

   printf("\x1b[1m 高輝度 \x1b[0m\n");
   printf("\x1b[2m 暗転 \x1b[0m\n");
   printf("\x1b[4m 下線 \x1b[0m\n");
   printf("\x1b[5m 点滅(blink)だが点滅しない \x1b[0m\n");
   printf("\x1b[7m 背景と前景の入替 \x1b[0m\n");
   printf("\x1b[8m 隠す \x1b[0m\n");
   printf("\x1b[9m 取消線 \x1b[0m\n");

   return 0;
}
