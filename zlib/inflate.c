#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <zlib.h>

#define GZ_MODE "rb6f"
#define BUF_SIZE (512)

int
main(int argc, char *argv[])
{
    if (argc != 2)
        {
            fprintf(stderr, "Usage : %s filename\n", argv[0]);
            exit(1);
        }

    int fd = open(argv[1], O_RDONLY);
    if (fd < 0)
        {
            perror("failed to open");
            exit(1);
        }

    gzFile inFile = gzdopen(fd, GZ_MODE);
    if (inFile == NULL)
        {
            fprintf(stderr, "failed to gzdopen.\n");
            exit(1);
        }
    int ret;
    unsigned long sum = 0;
    char buf[BUF_SIZE];
    while((ret = gzread(inFile, buf, sizeof(buf))) != 0
          && ret != -1)
        {
            sum += ret;
        }
    if(ret == -1)
        {
            // gzerrorでエラーメッセージを取得する
            const char *msg = gzerror(inFile, &ret);
            if (ret == Z_ERRNO)
                {
                    msg = strerror(ret);
                }
            fprintf(stderr, "gzread failed. %s\n", msg);
            exit(1);
        }
    else
        {
            printf("read %lu\n", sum);
        }

    if ((ret = gzclose(inFile)) != Z_OK)
        {
            // gzcloseに失敗した場合,ファイルは閉じられるのでgzerrorできない
            fprintf(stderr, "gzclose failed.\n");
            exit(1);
        }

    return 0;
}
