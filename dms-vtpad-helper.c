// dms-vtpad-helper — 虚拟触控板注入守护进程。
//
// 用一个真实的多点触摸 uinput 设备冒充触控板，让 libinput 自己去做指针加速、
// 双指滚动、点按和拖拽。我们只负责把「手指在哪」翻译成 ABS_MT 事件。
//
// 设备类型由 udev 判定，判据是 BTN_TOOL_FINGER + 有 ABS 坐标 + 没有 INPUT_PROP_DIRECT：
// 少 BTN_TOOL_FINGER 就不是触控板，多 INPUT_PROP_DIRECT 就变成触摸屏。
// libinput 1.32 只看这个 udev 标签（evdev.c:1985），所以能力位满足判据之后
// 分类就一定对。
//
// 协议（Unix socket，一行一条）：
//   hello <panelW> <panelH> <pxPerMm>   连接后第一行，用来建设备
//   down <slot> <x> <y>                 手指按下（x,y 是面板逻辑像素）
//   move <slot> <x> <y>                 手指移动
//   up <slot>                           手指抬起
//   btn <left|right> <1|0>              独立按键
//   quit                                主动退出

#define _GNU_SOURCE

// 版本号由 Makefile 注入；插件侧硬编码期望版本，对不上就提示用户更新
#ifndef VTPAD_VERSION
#define VTPAD_VERSION "0.0.0"
#endif

#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <linux/uinput.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#define DEVICE_NAME "DMS Virtual Touchpad"

// ABS 范围放大 10 倍：input_absinfo.resolution 是整数，直接拿
// 「面板逻辑像素 / 毫米」（平板约 6.18）当 resolution 只有一位有效数字，
// 放大之后是 62，误差从 ~3% 降到 ~0.3%。
#define ABS_SCALE 10

#define MAX_SLOTS 10

struct pad {
    int fd;                    // uinput 触控板 fd，-1 = 没建设备
    int kbd_fd;                // uinput 修饰键键盘 fd，跟触控板同生共死
    int w;                     // 面板逻辑宽（像素）
    int h;                     // 面板逻辑高（像素）
    double px_per_mm;          // 屏幕 DPI（逻辑像素 / 毫米）
    bool slot_active[MAX_SLOTS];
    int next_tracking_id;
    int down_count;            // 当前按着几根手指，用来维护 BTN_TOOL_*
};

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig) {
    (void)sig;
    g_stop = 1;
}

// 保证 0/1/2 都被占住，这样 /dev/uinput 拿到的 fd 一定 >= 3。
// 否则在「ssh + setsid 后台」这种标准流不完整的会话里，uinput 会拿到 fd 2，
// 事件就一路写进了 stderr —— 表现是日志里出现一大片 NUL（SYN 事件全是 0）。
static void ensure_stdio(void) {
    for (int fd = 0; fd <= 2; fd++) {
        if (fcntl(fd, F_GETFD) == -1)
            open("/dev/null", fd == 0 ? O_RDONLY : O_WRONLY);
    }
}

static void logmsg(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "[vtpad] ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    fflush(stderr);
    va_end(ap);
}

// ---------------------------------------------------------------- uinput

static void emit(int fd, int type, int code, int value) {
    // 双保险：绝不往标准流里写事件
    if (fd < 3)
        return;

    struct input_event ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = type;
    ev.code = code;
    ev.value = value;
    if (write(fd, &ev, sizeof(ev)) < 0)
        logmsg("write event failed: %s", strerror(errno));
}

static void syn(int fd) {
    emit(fd, EV_SYN, SYN_REPORT, 0);
}

static void abs_setup(int fd, int code, int min, int max, int resolution) {
    struct uinput_abs_setup setup;
    memset(&setup, 0, sizeof(setup));
    setup.code = code;
    setup.absinfo.minimum = min;
    setup.absinfo.maximum = max;
    setup.absinfo.resolution = resolution;
    if (ioctl(fd, UI_ABS_SETUP, &setup) < 0)
        logmsg("UI_ABS_SETUP code=%d failed: %s", code, strerror(errno));
}

// 修饰键键盘：触控板的 BTN_LEFT 本身带不了修饰键，必须由**键盘设备**先把
// Super/Shift 按下，合成器才会把那个点击算成 Super+Click。
// 触控板设备上发 KEY_* 没用 —— libinput 的 touchpad dispatch 只认 BTN_TOOL_*/BTN_LEFT/RIGHT，
// 其它 KEY 一律忽略，不会送到合成器。
static int create_keyboard(void) {
    int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (fd < 0) {
        logmsg("open /dev/uinput for keyboard failed: %s", strerror(errno));
        return -1;
    }

    ioctl(fd, UI_SET_EVBIT, EV_KEY);
    // 声明整段标准键位（0x01..0xff）：修饰键、字母数字、功能键、方向键、编辑键都在里面。
    // 只声明 4 个修饰键的话，普通按键还是得走 ydotool —— 那条路每次 25ms，
    // 而且没法「按住不放让系统自己重复」。
    for (int k = 1; k < 256; k++)
        ioctl(fd, UI_SET_KEYBIT, k);

    struct uinput_setup setup;
    memset(&setup, 0, sizeof(setup));
    setup.id.bustype = BUS_VIRTUAL;
    setup.id.vendor = 0x1209;
    setup.id.product = 0x0002;
    setup.id.version = 0x0001;
    snprintf(setup.name, UINPUT_MAX_NAME_SIZE, "%s Modifiers", DEVICE_NAME);

    if (ioctl(fd, UI_DEV_SETUP, &setup) < 0 || ioctl(fd, UI_DEV_CREATE) < 0) {
        logmsg("keyboard UI_DEV_SETUP/CREATE failed: %s", strerror(errno));
        close(fd);
        return -1;
    }

    logmsg("modifier keyboard created");
    return fd;
}

// 销毁设备：fd 关掉内核即回收，不需要别的清理
static void destroy_device(struct pad *p) {
    if (p->fd >= 0) {
        // 兜底把按着的东西松开，免得下游以为还按着
        for (int i = 0; i < MAX_SLOTS; i++) {
            if (!p->slot_active[i])
                continue;

            emit(p->fd, EV_ABS, ABS_MT_SLOT, i);
            emit(p->fd, EV_ABS, ABS_MT_TRACKING_ID, -1);
            p->slot_active[i] = false;
        }
        if (p->down_count > 0) {
            emit(p->fd, EV_KEY, BTN_TOOL_FINGER, 0);
            emit(p->fd, EV_KEY, BTN_TOOL_DOUBLETAP, 0);
            p->down_count = 0;
        }
        syn(p->fd);

        ioctl(p->fd, UI_DEV_DESTROY);
        close(p->fd);
        p->fd = -1;
    }

    if (p->kbd_fd >= 0) {
        // 别把修饰键留在按下状态里
        const int mods[] = {KEY_LEFTCTRL, KEY_LEFTSHIFT, KEY_LEFTALT, KEY_LEFTMETA};
        for (size_t i = 0; i < sizeof(mods) / sizeof(mods[0]); i++)
            emit(p->kbd_fd, EV_KEY, mods[i], 0);
        syn(p->kbd_fd);

        ioctl(p->kbd_fd, UI_DEV_DESTROY);
        close(p->kbd_fd);
        p->kbd_fd = -1;
    }

    logmsg("device destroyed");
}

static int create_device(struct pad *p, int w, int h, double px_per_mm) {
    if (w <= 0 || h <= 0 || px_per_mm <= 0.0) {
        logmsg("bad geometry %dx%d @%.3f px/mm", w, h, px_per_mm);
        return -1;
    }

    int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (fd < 0) {
        logmsg("open /dev/uinput failed: %s", strerror(errno));
        return -1;
    }

    ioctl(fd, UI_SET_EVBIT, EV_SYN);
    ioctl(fd, UI_SET_EVBIT, EV_KEY);
    ioctl(fd, UI_SET_EVBIT, EV_ABS);

    // BTN_TOOL_FINGER 是 udev 判 touchpad 的唯一正面标志，缺它必然不是触控板
    ioctl(fd, UI_SET_KEYBIT, BTN_TOOL_FINGER);
    ioctl(fd, UI_SET_KEYBIT, BTN_TOOL_DOUBLETAP);
    ioctl(fd, UI_SET_KEYBIT, BTN_TOOL_TRIPLETAP);
    // BTN_TOUCH 只是能力声明：libinput 的 tp_pass_sanity_check 硬性要求它存在
    // （缺了直接 "device failed touchpad sanity checks" 把设备拒掉）。运行时不必发。
    ioctl(fd, UI_SET_KEYBIT, BTN_TOUCH);
    // 有 BTN_RIGHT 才不会被 libinput 当成 clickpad（否则会打 kernel bug 警告）
    ioctl(fd, UI_SET_KEYBIT, BTN_LEFT);
    ioctl(fd, UI_SET_KEYBIT, BTN_RIGHT);

    // 加这个属性才是「间接设备」。千万不要加 INPUT_PROP_DIRECT —— 那会变触摸屏。
    ioctl(fd, UI_SET_PROPBIT, INPUT_PROP_POINTER);

    const int abs_w = w * ABS_SCALE;
    const int abs_h = h * ABS_SCALE;
    // resolution 必须 > 0：libinput 拿它做 units → mm 的换算，0 会除零
    const int res = (int)(px_per_mm * ABS_SCALE + 0.5);

    // ABS_X/Y 必须**声明**（sanity check 要求有 ABS_X；absinfo 还要给 libinput 算
    // units → mm 的 resolution），但运行时**绝不发**这两个轴的事件 —— 发了就会被
    // 当成绝对坐标，手指点到哪指针就跳到哪。
    abs_setup(fd, ABS_X, 0, abs_w, res);
    abs_setup(fd, ABS_Y, 0, abs_h, res);
    abs_setup(fd, ABS_PRESSURE, 0, 255, 0);
    abs_setup(fd, ABS_MT_SLOT, 0, MAX_SLOTS - 1, 0);
    abs_setup(fd, ABS_MT_POSITION_X, 0, abs_w, res);
    abs_setup(fd, ABS_MT_POSITION_Y, 0, abs_h, res);
    abs_setup(fd, ABS_MT_TRACKING_ID, 0, 65535, 0);
    abs_setup(fd, ABS_MT_PRESSURE, 0, 255, 0);

    struct uinput_setup setup;
    memset(&setup, 0, sizeof(setup));
    setup.id.bustype = BUS_VIRTUAL;
    setup.id.vendor = 0x1209;
    setup.id.product = 0x0001;
    setup.id.version = 0x0001;
    snprintf(setup.name, UINPUT_MAX_NAME_SIZE, "%s", DEVICE_NAME);

    if (ioctl(fd, UI_DEV_SETUP, &setup) < 0) {
        logmsg("UI_DEV_SETUP failed: %s", strerror(errno));
        close(fd);
        return -1;
    }

    if (ioctl(fd, UI_DEV_CREATE) < 0) {
        logmsg("UI_DEV_CREATE failed: %s", strerror(errno));
        close(fd);
        return -1;
    }

    p->fd = fd;
    p->w = w;
    p->h = h;
    p->px_per_mm = px_per_mm;
    p->next_tracking_id = 1;
    p->down_count = 0;
    memset(p->slot_active, 0, sizeof(p->slot_active));

    logmsg("device created: %dx%d px, %.2f px/mm -> %dx%d units, res=%d, physical %.1fx%.1f mm",
           w, h, px_per_mm, abs_w, abs_h, res, w / px_per_mm, h / px_per_mm);

    // 修饰键键盘跟触控板同生共死：没有键盘就发不出 Super+Click
    p->kbd_fd = create_keyboard();
    return 0;
}

// ---------------------------------------------------------------- 事件翻译

static void touch_down(struct pad *p, int slot, int x, int y) {
    if (p->fd < 0 || slot < 0 || slot >= MAX_SLOTS)
        return;

    const int abs_x = x * ABS_SCALE;
    const int abs_y = y * ABS_SCALE;

    emit(p->fd, EV_ABS, ABS_MT_SLOT, slot);
    emit(p->fd, EV_ABS, ABS_MT_TRACKING_ID, p->next_tracking_id++);
    emit(p->fd, EV_ABS, ABS_MT_POSITION_X, abs_x);
    emit(p->fd, EV_ABS, ABS_MT_POSITION_Y, abs_y);
    emit(p->fd, EV_ABS, ABS_MT_PRESSURE, 120);

    if (!p->slot_active[slot]) {
        p->slot_active[slot] = true;
        p->down_count++;
    }
    if (p->next_tracking_id > 65535)
        p->next_tracking_id = 1;

    emit(p->fd, EV_KEY, BTN_TOOL_FINGER, 1);
    emit(p->fd, EV_KEY, BTN_TOOL_DOUBLETAP, p->down_count >= 2 ? 1 : 0);
    emit(p->fd, EV_KEY, BTN_TOOL_TRIPLETAP, p->down_count >= 3 ? 1 : 0);
    syn(p->fd);
}

static void touch_move(struct pad *p, int slot, int x, int y) {
    if (p->fd < 0 || slot < 0 || slot >= MAX_SLOTS)
        return;

    const int abs_x = x * ABS_SCALE;
    const int abs_y = y * ABS_SCALE;

    emit(p->fd, EV_ABS, ABS_MT_SLOT, slot);
    emit(p->fd, EV_ABS, ABS_MT_POSITION_X, abs_x);
    emit(p->fd, EV_ABS, ABS_MT_POSITION_Y, abs_y);
    syn(p->fd);
}

static void touch_up(struct pad *p, int slot) {
    if (p->fd < 0 || slot < 0 || slot >= MAX_SLOTS)
        return;

    emit(p->fd, EV_ABS, ABS_MT_SLOT, slot);
    emit(p->fd, EV_ABS, ABS_MT_TRACKING_ID, -1);

    if (p->slot_active[slot]) {
        p->slot_active[slot] = false;
        p->down_count--;
        if (p->down_count < 0)
            p->down_count = 0;
    }

    if (p->down_count == 0) {
        emit(p->fd, EV_KEY, BTN_TOOL_FINGER, 0);
        emit(p->fd, EV_KEY, BTN_TOOL_DOUBLETAP, 0);
        emit(p->fd, EV_KEY, BTN_TOOL_TRIPLETAP, 0);
    } else {
        emit(p->fd, EV_KEY, BTN_TOOL_DOUBLETAP, p->down_count >= 2 ? 1 : 0);
        emit(p->fd, EV_KEY, BTN_TOOL_TRIPLETAP, p->down_count >= 3 ? 1 : 0);
    }
    syn(p->fd);
}

static void button(struct pad *p, const char *name, int down) {
    if (p->fd < 0)
        return;

    int code;
    if (strcmp(name, "left") == 0)
        code = BTN_LEFT;
    else if (strcmp(name, "right") == 0)
        code = BTN_RIGHT;
    else
        return;

    emit(p->fd, EV_KEY, code, down ? 1 : 0);
    syn(p->fd);
}

// ---------------------------------------------------------------- 协议

// 返回 1 = 继续，0 = 客户端要求断开，-1 = 解析错误
static int handle_line(struct pad *p, char *line, int client_fd) {
    // 去掉行尾
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r'))
        line[--len] = '\0';

    if (len == 0)
        return 1;

    char cmd[32] = {0};
    int consumed = 0;

    if (sscanf(line, "%31s%n", cmd, &consumed) != 1)
        return -1;

    const char *rest = line + consumed;

    if (strcmp(cmd, "hello") == 0) {
        int w = 0, h = 0;
        double px_per_mm = 0.0;
        if (sscanf(rest, "%d %d %lf", &w, &h, &px_per_mm) != 3) {
            logmsg("bad hello: %s", rest);
            return -1;
        }
        // 参数变了就重建设备：ABS 范围在设备创建后改不了
        if (p->fd >= 0 && (p->w != w || p->h != h || p->px_per_mm != px_per_mm)) {
            logmsg("geometry changed, recreating device");
            destroy_device(p);
        }
        if (p->fd < 0)
            return create_device(p, w, h, px_per_mm) == 0 ? 1 : -1;
        return 1;
    }

    if (strcmp(cmd, "down") == 0 || strcmp(cmd, "move") == 0) {
        int slot = 0, x = 0, y = 0;
        if (sscanf(rest, "%d %d %d", &slot, &x, &y) != 3)
            return -1;
        if (cmd[0] == 'd')
            touch_down(p, slot, x, y);
        else
            touch_move(p, slot, x, y);
        return 1;
    }

    if (strcmp(cmd, "up") == 0) {
        int slot = 0;
        if (sscanf(rest, "%d", &slot) != 1)
            return -1;
        touch_up(p, slot);
        return 1;
    }

    if (strcmp(cmd, "btn") == 0) {
        char name[16] = {0};
        int down = 0;
        if (sscanf(rest, "%15s %d", name, &down) != 2)
            return -1;
        button(p, name, down);
        return 1;
    }

    // 键盘事件（修饰键和普通按键都走这里）。
    // 修饰键用 "mod"、普通键用 "key"，实现完全一样 —— 分开只是为了日志好读。
    // 必须是独立的键盘设备：触控板设备上的 KEY_* 会被 libinput 丢掉。
    if (strcmp(cmd, "key") == 0 || strcmp(cmd, "mod") == 0) {
        int code = 0, down = 0;
        if (sscanf(rest, "%d %d", &code, &down) != 2)
            return -1;
        if (p->kbd_fd >= 0) {
            emit(p->kbd_fd, EV_KEY, code, down ? 1 : 0);
            syn(p->kbd_fd);
        }
        logmsg("%s %d %d", cmd, code, down);
        return 1;
    }

    // bye：只销毁设备、不关连接。切走触控板页时用 —— socket 留着给键盘按键注入用，
    // 但没必要一直挂着一个指针设备在系统里。
    if (strcmp(cmd, "bye") == 0) {
        destroy_device(p);
        return 1;
    }

    // ping → pong：客户端靠这个判断连接是不是「假活」。
    // helper 重启会 unlink+bind 新 socket，旧连接在客户端看来还是 ESTAB，
    // 写进去的数据却没人收 —— 只有真回一次话才能确认。
    if (strcmp(cmd, "ping") == 0) {
        static const char pong[] = "pong\n";
        if (client_fd >= 0)
            (void)!write(client_fd, pong, sizeof(pong) - 1);
        return 1;
    }

    if (strcmp(cmd, "quit") == 0)
        return 0;

    logmsg("unknown command: %s", cmd);
    return 1;
}

// ---------------------------------------------------------------- socket

// 能连上说明这个 socket 后面真的有人在 accept —— 那就是另一个实例。
// 先探测再 unlink，才能保住「Unix socket 天然只有一个 server」这条性质：
// 无条件 unlink 会把别人的 socket 删掉、自己绑上去，两个进程互相不知道。
static bool socket_alive(const char *path) {
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    if (strlen(path) >= sizeof(addr.sun_path))
        return false;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);

    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0)
        return false;

    bool alive = connect(fd, (struct sockaddr *)&addr, sizeof(addr)) == 0;
    close(fd);
    return alive;
}

static int make_server(const char *path) {
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    if (strlen(path) >= sizeof(addr.sun_path)) {
        logmsg("socket path too long: %s", path);
        return -1;
    }
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);

    unlink(path);   // 上次没清干净的残留

    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        logmsg("socket failed: %s", strerror(errno));
        return -1;
    }
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        logmsg("bind %s failed: %s", path, strerror(errno));
        close(fd);
        return -1;
    }
    chmod(path, 0600);
    if (listen(fd, 4) < 0) {
        logmsg("listen failed: %s", strerror(errno));
        close(fd);
        return -1;
    }
    return fd;
}

int main(int argc, char **argv) {
    // 必须在任何 open() 之前：否则 uinput 可能抢到 fd 1/2
    ensure_stdio();
    // 默认写 stderr：systemd 会把它收进 journal。
    // 只有显式设了 VTPAD_LOG 才落到文件（手动跑、或 ssh + setsid 这种
    // stderr 不可靠的会话里调试用 —— 那种情况 ensure_stdio 会把 stderr 补成 /dev/null）。
    const char *log_path = getenv("VTPAD_LOG");
    if (log_path && *log_path)
        freopen(log_path, "a", stderr);

    const char *path = NULL;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-s") == 0 && i + 1 < argc)
            path = argv[++i];
        else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("usage: %s [-s socket-path] [--version]\n", argv[0]);
            printf("  default socket: $XDG_RUNTIME_DIR/dms-vtpad.sock\n");
            return 0;
        } else if (strcmp(argv[i], "-V") == 0 || strcmp(argv[i], "--version") == 0) {
            // 前端/设置页用它做版本检查，比读二进制省事
            printf("%s\n", VTPAD_VERSION);
            return 0;
        }
    }

    // 把版本写进 runtime dir：设置页读它做版本检查。
    // addon 是单客户端设计（新连接顶掉旧的），设置页没法自己连去问版本。
    {
        const char *runtime = getenv("XDG_RUNTIME_DIR");
        if (runtime && *runtime) {
            char vpath[300];
            snprintf(vpath, sizeof(vpath), "%s/dms-vtpad.version", runtime);
            FILE *vf = fopen(vpath, "w");
            if (vf) {
                fprintf(vf, "%s\n", VTPAD_VERSION);
                fclose(vf);
            }
        }
    }

    char default_path[256];
    if (!path) {
        const char *runtime = getenv("XDG_RUNTIME_DIR");
        if (!runtime || !*runtime) {
            logmsg("XDG_RUNTIME_DIR not set, pass -s <path>");
            return 1;
        }
        snprintf(default_path, sizeof(default_path), "%s/dms-vtpad.sock", runtime);
        path = default_path;
    }

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    // 单实例锁：重复启动直接退出。
    // 没有这把锁的话，重启时若旧进程没杀干净，新进程会 unlink+bind 一个新 socket，
    // 而客户端还连着旧的 —— 表现是「设备建了又销毁、按键时有时无」，极难排查。
    // 锁跟着进程生命周期走（fd 关闭即释放），残留文件不会挡下一次启动。
    char lock_path[300];
    snprintf(lock_path, sizeof(lock_path), "%s.lock", path);
    int lock_fd = open(lock_path, O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (lock_fd < 0 || flock(lock_fd, LOCK_EX | LOCK_NB) < 0) {
        logmsg("another instance is already running (%s)", lock_path);
        return 1;
    }

    struct pad p;
    memset(&p, 0, sizeof(p));
    p.fd = -1;
    p.kbd_fd = -1;   // memset 出来的 0 是 stdin，必须显式设 -1

    if (socket_alive(path)) {
        logmsg("another instance is already serving %s", path);
        return 1;
    }

    int server = make_server(path);
    if (server < 0)
        return 1;

    logmsg("listening on %s", path);

    while (!g_stop) {
        struct pollfd pfd = {.fd = server, .events = POLLIN};
        int r = poll(&pfd, 1, 500);
        if (r < 0) {
            if (errno == EINTR)
                continue;

            logmsg("poll failed: %s", strerror(errno));
            break;
        }
        if (r == 0)
            continue;

        int client = accept4(server, NULL, NULL, SOCK_CLOEXEC);
        if (client < 0) {
            if (errno == EINTR || errno == EAGAIN)
                continue;

            logmsg("accept failed: %s", strerror(errno));
            continue;
        }

        logmsg("client connected");
        // 连上先报版本：和 addon 的 version:<x.y.z> 帧同格式，前端拿它比对期望版本
        {
            char vbuf[64];
            int vn = snprintf(vbuf, sizeof(vbuf), "version:%s\n", VTPAD_VERSION);
            if (vn > 0)
                (void)!write(client, vbuf, (size_t)vn);
        }
        FILE *fp = fdopen(client, "r");
        if (!fp) {
            close(client);
            continue;
        }

        char *line = NULL;
        size_t cap = 0;
        ssize_t n;
        while (!g_stop && (n = getline(&line, &cap, fp)) > 0) {
            int keep = handle_line(&p, line, client);
            if (keep <= 0) {
                if (keep < 0)
                    logmsg("protocol error, closing client");
                break;
            }
        }

        free(line);
        fclose(fp);
        destroy_device(&p);
        logmsg("client disconnected");
    }

    destroy_device(&p);
    close(server);
    unlink(path);
    logmsg("bye");
    return 0;
}
