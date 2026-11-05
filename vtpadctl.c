// vtpadctl — dms-vtpad-helper 的调试客户端。
//
// 从文件或 stdin 读协议行，逐行发给 helper。行与行之间默认留 16ms
// （一次 vsync），否则 libinput 会把连续 move 合成一帧，看不出运动。
//
//   vtpadctl -d 16 <<'EOF'
//   hello 806 496 6.18
//   down 0 400 250
//   move 0 440 250
//   up 0
//   EOF
//
// 用法：vtpadctl [-s socket] [-d delay_ms] [-t hold_ms] [file]

#define _GNU_SOURCE

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

static void msleep(int ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

int main(int argc, char **argv) {
    const char *path = NULL;
    const char *file = NULL;
    int delay_ms = 16;
    int hold_ms = 400;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-s") == 0 && i + 1 < argc)
            path = argv[++i];
        else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc)
            delay_ms = atoi(argv[++i]);
        else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc)
            hold_ms = atoi(argv[++i]);
        else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("usage: %s [-s socket] [-d delay_ms] [-t hold_ms] [file]\n", argv[0]);
            return 0;
        } else
            file = argv[i];
    }

    char default_path[256];
    if (!path) {
        const char *runtime = getenv("XDG_RUNTIME_DIR");
        if (!runtime || !*runtime) {
            fprintf(stderr, "XDG_RUNTIME_DIR not set, pass -s <path>\n");
            return 1;
        }
        snprintf(default_path, sizeof(default_path), "%s/dms-vtpad.sock", runtime);
        path = default_path;
    }

    FILE *in = stdin;
    if (file) {
        in = fopen(file, "r");
        if (!in) {
            fprintf(stderr, "cannot open %s: %s\n", file, strerror(errno));
            return 1;
        }
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    if (strlen(path) >= sizeof(addr.sun_path)) {
        fprintf(stderr, "socket path too long\n");
        return 1;
    }
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);

    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("socket");
        return 1;
    }
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "connect %s: %s\n", path, strerror(errno));
        close(fd);
        return 1;
    }

    char *line = NULL;
    size_t cap = 0;
    ssize_t n;
    int sent = 0;
    bool saw_hello = false;

    while ((n = getline(&line, &cap, in)) > 0) {
        // 跳过空行和注释
        char *p = line;
        while (*p == ' ' || *p == '\t')
            p++;
        if (*p == '\n' || *p == '\r' || *p == '\0' || *p == '#')
            continue;
        if (!strchr(p, '\n')) {
            // 最后一行没有换行，补上
            size_t L = strlen(p);
            p[L] = '\n';
            p[L + 1] = '\0';
        }

        if (write(fd, p, strlen(p)) < 0) {
            fprintf(stderr, "write: %s\n", strerror(errno));
            break;
        }
        sent++;

        if (!saw_hello && strncmp(p, "hello", 5) == 0) {
            saw_hello = true;
            // 等 helper 把设备建出来，udev + libinput 还要认一遍
            msleep(300);
        } else {
            msleep(delay_ms);
        }
    }

    free(line);

    // 保持连接，让事件走完 libinput → compositor
    msleep(hold_ms);

    close(fd);
    fprintf(stderr, "vtpadctl: sent %d lines\n", sent);
    return 0;
}
