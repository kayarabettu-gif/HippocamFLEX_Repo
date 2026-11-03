/*
 * board_server: HippocamFLEX recall over TCP for the live demo.
 *
 * At startup it resets the core, checks that it answers a status query and
 * trains the five demo sequences. Then it serves one JSON request per
 * connection:
 *   request:  {"ctx": 0..4, "evt": 0..6}
 *   reply:    {"ok":true,"present":"D3","future1":"D5","future2":null,
 *              "events":{...},"timestamps":{...}}
 *
 * The DMA and packet code is the same as sw/app/hippocampex_7seg_test_final.c.
 *
 * build: aarch64-linux-gnu-gcc -O1 -Wall -static -o board_server board_server.c
 * run:   sudo ./board_server --port 9000 2>&1 | tee server.log
 * test:  echo '{"ctx":3,"evt":1}' | nc <board-ip> 9000      (BLUE/D2)
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define DMA_PHYS   0xA0000000UL
#define GPIO_PHYS  0xA0010000UL
#define TX_PHYS    0x6BA00000UL
#define RX_PHYS    0x6BA01000UL

#define MM2S_DMACR  (0x00/4)
#define MM2S_DMASR  (0x04/4)
#define MM2S_SA     (0x18/4)
#define MM2S_SA_MSB (0x1C/4)
#define MM2S_LEN    (0x28/4)
#define S2MM_DMACR  (0x30/4)
#define S2MM_DMASR  (0x34/4)
#define S2MM_DA     (0x48/4)
#define S2MM_DA_MSB (0x4C/4)
#define S2MM_LEN    (0x58/4)
#define DMA_RESET   (1u << 2)

#define GPIO_DATA   (0x000/4)
#define GPIO_TRI    (0x004/4)
#define SW_RST_N    (1u << 0)

#define NUM_EVENTS      64
#define HV_WORDS        4
#define INFER_RX_BYTES  140
#define RX_OFF_VOLT     4
#define RX_OFF_TS       76

#define VP   0x40u
#define VF1  0x80u
#define VF2  0xC0u

/* slots must match hippocamflex_hv.py */
#define CTX_RED     0   /* 7->5->3->1 */
#define CTX_MAGENTA 1   /* 1->3->5->7 */
#define CTX_GREEN   2   /* 2->4->6 */
#define CTX_BLUE    3   /* 1->2->3->5 */
#define CTX_WHITE   4   /* 2->3->5->7 */
#define N_CTX       5

#define EVT_D1 0
#define EVT_D2 1
#define EVT_D3 2
#define EVT_D4 3
#define EVT_D5 4
#define EVT_D6 5
#define EVT_D7 6
#define N_EVT  7

static const uint32_t CTX_HV[N_CTX][HV_WORDS] = {
    [CTX_RED]     = {0xe682c9a3, 0x21a3feff, 0x92ba194a, 0xc6647f01},
    [CTX_MAGENTA] = {0xf5808b11, 0x81497ed7, 0xe2283d0e, 0xcef57b41},
    [CTX_GREEN]   = {0x8ff86ad0, 0xebe6d881, 0xb0818a19, 0xe1584820},
    [CTX_BLUE]    = {0xdda09711, 0xc4593447, 0xe468751e, 0xebb5204f},
    [CTX_WHITE]   = {0xcfa0cb91, 0xe1e3fcc7, 0xb0a8191a, 0xe3746801},
};
static const uint32_t EVT_HV[N_EVT][HV_WORDS] = {
    [EVT_D1] = {0x7f99b528, 0xbccb3593, 0x0a179ab4, 0x6023c479},
    [EVT_D2] = {0xd7ec5123, 0x7e4b5073, 0x7a170604, 0x444bcbd4},
    [EVT_D3] = {0xd5bc55ae, 0x7c4b103b, 0x6e973690, 0x4443cfd8},
    [EVT_D4] = {0xd5bffca0, 0x1c433002, 0x6f9636b4, 0xe0539910},
    [EVT_D5] = {0xd5a659a7, 0x7c4b305f, 0x6e9676d1, 0x4452dbf8},
    [EVT_D6] = {0xd72e59a3, 0x7ccb317b, 0x6e963e11, 0x6463dbf0},
    [EVT_D7] = {0xe7fbb4ae, 0x3c4b10bb, 0x6a379680, 0x6443c5fc},
};

static volatile uint32_t *dma = NULL, *gpio = NULL, *tx_buf = NULL, *rx_buf = NULL;
static int mem_fd = -1;
static uint8_t rx_volt[NUM_EVENTS];
static uint8_t rx_timestamps[NUM_EVENTS];

static volatile uint32_t *map_phys(uint64_t phys, size_t sz) {
    void *p = mmap(NULL, sz, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, (off_t)phys);
    if (p == MAP_FAILED) { fprintf(stderr, "[FATAL] mmap 0x%lx: %m\n", phys); exit(1); }
    return (volatile uint32_t *)p;
}

#define RESET_LOOPS 10000000u
static void dma_init(void) {
    uint32_t n;
    dma[MM2S_DMACR] = DMA_RESET;
    n = RESET_LOOPS; while ((dma[MM2S_DMACR] & DMA_RESET) && --n);
    if (!n) { fprintf(stderr,"[FATAL] DMA reset timeout\n"); exit(1); }
    usleep(1000);
    dma[MM2S_DMACR] = 1; dma[S2MM_DMACR] = 1;
    usleep(500);
    fprintf(stderr, "[DMA] MM2S=0x%08x  S2MM=0x%08x\n",
            dma[MM2S_DMASR], dma[S2MM_DMASR]);
}

#define POLL_TIMEOUT_MS 2000
static int wait_magic(uint8_t magic, const char *ctx) {
    volatile uint8_t *b = (volatile uint8_t *)rx_buf;
    int n = POLL_TIMEOUT_MS * 100;
    while (n-- > 0) {
        if (b[3] == magic) return 0;
        usleep(10);
    }
    fprintf(stderr, "[WARN] timeout magic=0x%02x ctx=%s\n", magic, ctx);
    return -1;
}

static int transact(uint32_t tx_bytes, uint32_t rx_max,
                    uint8_t expected_magic, const char *ctx) {
    volatile uint8_t *b = (volatile uint8_t *)rx_buf;
    b[3] = 0x00;
    dma[S2MM_DA] = (uint32_t)RX_PHYS; dma[S2MM_DA_MSB] = 0;
    dma[S2MM_LEN] = rx_max;
    dma[MM2S_SA] = (uint32_t)TX_PHYS; dma[MM2S_SA_MSB] = 0;
    dma[MM2S_LEN] = tx_bytes;
    return wait_magic(expected_magic, ctx);
}

static uint32_t pkt_ctx(int slot, const uint32_t hv[4]) {
    tx_buf[0] = (0x1u << 28) | ((uint32_t)slot & 0x7F);
    tx_buf[1]=hv[0]; tx_buf[2]=hv[1]; tx_buf[3]=hv[2]; tx_buf[4]=hv[3];
    return 20;
}
static uint32_t pkt_evt(int slot, const uint32_t hv[4]) {
    tx_buf[0] = (0x2u << 28) | ((uint32_t)slot & 0x7F);
    tx_buf[1]=hv[0]; tx_buf[2]=hv[1]; tx_buf[3]=hv[2]; tx_buf[4]=hv[3];
    return 20;
}
static uint32_t pkt_assoc(int f, int t) {
    tx_buf[0] = (0x3u<<28)|(((uint32_t)f&0x7F)<<14)|(((uint32_t)t&0x7F)<<7)|1u;
    return 4;
}
static uint32_t pkt_gate(int e, int c) {
    tx_buf[0] = (0x4u<<28)|(((uint32_t)e&0x7F)<<14)|(((uint32_t)c&0x7F)<<7)|1u;
    return 4;
}
static uint32_t pkt_mcam(int e, int c, int t) {
    tx_buf[0] = (0x5u<<28)|(((uint32_t)e&0x7F)<<14)|(((uint32_t)c&0x7F)<<7)|((uint32_t)t&0x3F);
    return 4;
}
static uint32_t pkt_infer(const uint32_t c[4], const uint32_t e[4]) {
    tx_buf[0] = (0x6u<<28);
    tx_buf[1]=c[0]; tx_buf[2]=c[1]; tx_buf[3]=c[2]; tx_buf[4]=c[3];
    tx_buf[5]=e[0]; tx_buf[6]=e[1]; tx_buf[7]=e[2]; tx_buf[8]=e[3];
    return 36;
}

/* every training command is acked with 0xC1 */
static void train_ctx(int s, const uint32_t hv[4])   { transact(pkt_ctx(s, hv),   4, 0xC1, "ctx"); }
static void train_evt(int s, const uint32_t hv[4])   { transact(pkt_evt(s, hv),   4, 0xC1, "evt"); }
static void train_assoc(int f, int t)                { transact(pkt_assoc(f, t),  4, 0xC1, "assoc"); }
static void train_gate(int e, int c)                 { transact(pkt_gate(e, c),   4, 0xC1, "gate"); }
static void train_mcam(int e, int c, int t)          { transact(pkt_mcam(e, c, t), 4, 0xC1, "mcam"); }

static void load_sequences(void) {
    fprintf(stderr, "[TRAIN] Context HVs\n");
    for (int c = 0; c < N_CTX; c++) train_ctx(c, CTX_HV[c]);
    fprintf(stderr, "[TRAIN] Event HVs\n");
    for (int e = 0; e < N_EVT; e++) train_evt(e, EVT_HV[e]);

    fprintf(stderr, "[TRAIN] Sequences\n");

    /* RED 7->5->3->1 */
    train_assoc(EVT_D7,EVT_D5); train_assoc(EVT_D5,EVT_D3);
    train_assoc(EVT_D3,EVT_D1);
    train_gate(EVT_D7,CTX_RED); train_gate(EVT_D5,CTX_RED);
    train_gate(EVT_D3,CTX_RED); train_gate(EVT_D1,CTX_RED);
    train_mcam(EVT_D7,CTX_RED,1); train_mcam(EVT_D5,CTX_RED,2);
    train_mcam(EVT_D3,CTX_RED,3); train_mcam(EVT_D1,CTX_RED,4);

    /* MAGENTA 1->3->5->7 */
    train_assoc(EVT_D1,EVT_D3); train_assoc(EVT_D3,EVT_D5);
    train_assoc(EVT_D5,EVT_D7);
    train_gate(EVT_D1,CTX_MAGENTA); train_gate(EVT_D3,CTX_MAGENTA);
    train_gate(EVT_D5,CTX_MAGENTA); train_gate(EVT_D7,CTX_MAGENTA);
    train_mcam(EVT_D1,CTX_MAGENTA,1); train_mcam(EVT_D3,CTX_MAGENTA,2);
    train_mcam(EVT_D5,CTX_MAGENTA,3); train_mcam(EVT_D7,CTX_MAGENTA,4);

    /* GREEN 2->4->6 */
    train_assoc(EVT_D2,EVT_D4); train_assoc(EVT_D4,EVT_D6);
    train_gate(EVT_D2,CTX_GREEN); train_gate(EVT_D4,CTX_GREEN);
    train_gate(EVT_D6,CTX_GREEN);
    train_mcam(EVT_D2,CTX_GREEN,1); train_mcam(EVT_D4,CTX_GREEN,2);
    train_mcam(EVT_D6,CTX_GREEN,3);

    /* BLUE 1->2->3->5. Links are shared by every context; the gates keep
     * each context on its own sequence. */
    train_assoc(EVT_D1,EVT_D2); train_assoc(EVT_D2,EVT_D3);
    train_assoc(EVT_D3,EVT_D5);
    train_gate(EVT_D1,CTX_BLUE); train_gate(EVT_D2,CTX_BLUE);
    train_gate(EVT_D3,CTX_BLUE); train_gate(EVT_D5,CTX_BLUE);
    train_mcam(EVT_D1,CTX_BLUE,1); train_mcam(EVT_D2,CTX_BLUE,2);
    train_mcam(EVT_D3,CTX_BLUE,3); train_mcam(EVT_D5,CTX_BLUE,4);

    /* WHITE 2->3->5->7. BLUE and MAGENTA already wrote all three links,
     * so only gates and codes are added. */
    train_gate(EVT_D2,CTX_WHITE); train_gate(EVT_D3,CTX_WHITE);
    train_gate(EVT_D5,CTX_WHITE); train_gate(EVT_D7,CTX_WHITE);
    train_mcam(EVT_D2,CTX_WHITE,1); train_mcam(EVT_D3,CTX_WHITE,2);
    train_mcam(EVT_D5,CTX_WHITE,3); train_mcam(EVT_D7,CTX_WHITE,4);

    fprintf(stderr, "[TRAIN] done\n");
}

static int do_infer(int ctx_idx, int evt_idx) {
    if (ctx_idx < 0 || ctx_idx >= N_CTX) return -1;
    if (evt_idx < 0 || evt_idx >= N_EVT) return -1;
    memset(rx_volt, 0, sizeof(rx_volt));
    memset(rx_timestamps, 0x3F, sizeof(rx_timestamps));
    int rc = transact(pkt_infer(CTX_HV[ctx_idx], EVT_HV[evt_idx]),
                      INFER_RX_BYTES, 0xA1, "infer");
    if (rc != 0) return -1;
    const uint8_t *b = (const uint8_t *)rx_buf;
    memcpy(rx_volt, b + RX_OFF_VOLT, NUM_EVENTS);
    memcpy(rx_timestamps, b + RX_OFF_TS, NUM_EVENTS);
    return 0;
}

static const char *evt_name(int e) {
    static const char *names[] = {"D1","D2","D3","D4","D5","D6","D7"};
    return (e >= 0 && e < N_EVT) ? names[e] : "?";
}

/* present/future1/future2 are null when no event has that voltage */
static int build_response_json(char *out, size_t cap, int ok, const char *note)
{
    if (!ok) {
        return snprintf(out, cap, "{\"ok\":false,\"note\":\"%s\"}\n", note);
    }

    int e_present = -1, e_future1 = -1, e_future2 = -1;
    for (int e = 0; e < N_EVT; e++) {
        if (rx_volt[e] == VP)  e_present = e;
        if (rx_volt[e] == VF1) e_future1 = e;
        if (rx_volt[e] == VF2) e_future2 = e;
    }

    size_t off = 0;
    off += snprintf(out + off, cap - off, "{\"ok\":true");
    off += snprintf(out + off, cap - off, ",\"present\":%s%s%s",
                    e_present >= 0 ? "\"" : "",
                    e_present >= 0 ? evt_name(e_present) : "null",
                    e_present >= 0 ? "\"" : "");
    off += snprintf(out + off, cap - off, ",\"future1\":%s%s%s",
                    e_future1 >= 0 ? "\"" : "",
                    e_future1 >= 0 ? evt_name(e_future1) : "null",
                    e_future1 >= 0 ? "\"" : "");
    off += snprintf(out + off, cap - off, ",\"future2\":%s%s%s",
                    e_future2 >= 0 ? "\"" : "",
                    e_future2 >= 0 ? evt_name(e_future2) : "null",
                    e_future2 >= 0 ? "\"" : "");

    off += snprintf(out + off, cap - off, ",\"events\":{");
    int first = 1;
    for (int e = 0; e < N_EVT; e++) {
        if (rx_volt[e] == 0) continue;
        off += snprintf(out + off, cap - off, "%s\"%s\":%u",
                        first ? "" : ",", evt_name(e), rx_volt[e]);
        first = 0;
    }
    off += snprintf(out + off, cap - off, "}");

    /* 0x3F = not placed by CA1 */
    off += snprintf(out + off, cap - off, ",\"timestamps\":{");
    first = 1;
    for (int e = 0; e < N_EVT; e++) {
        if (rx_timestamps[e] == 0x3F) continue;
        off += snprintf(out + off, cap - off, "%s\"%s\":%u",
                        first ? "" : ",", evt_name(e), rx_timestamps[e]);
        first = 0;
    }
    off += snprintf(out + off, cap - off, "}}\n");
    return (int)off;
}

/* not a real JSON parser: takes the integers after "ctx" and "evt" */
static int parse_request(const char *buf, int *ctx_idx, int *evt_idx) {
    const char *p;
    *ctx_idx = *evt_idx = -1;
    p = strstr(buf, "\"ctx\"");
    if (!p) return -1;
    p = strchr(p, ':'); if (!p) return -1;
    *ctx_idx = atoi(p + 1);
    p = strstr(buf, "\"evt\"");
    if (!p) return -1;
    p = strchr(p, ':'); if (!p) return -1;
    *evt_idx = atoi(p + 1);
    return 0;
}

static void handle_client(int fd) {
    char req[512], resp[4096];
    ssize_t n = read(fd, req, sizeof(req) - 1);
    if (n <= 0) { close(fd); return; }
    req[n] = '\0';

    int ctx_idx, evt_idx;
    int len;
    if (parse_request(req, &ctx_idx, &evt_idx) != 0) {
        len = build_response_json(resp, sizeof(resp), 0, "bad request");
    } else if (ctx_idx < 0 || ctx_idx >= N_CTX ||
               evt_idx < 0 || evt_idx >= N_EVT) {
        len = build_response_json(resp, sizeof(resp), 0, "index out of range");
    } else {
        fprintf(stderr, "[REQ] ctx=%d evt=%d\n", ctx_idx, evt_idx);
        if (do_infer(ctx_idx, evt_idx) != 0) {
            len = build_response_json(resp, sizeof(resp), 0, "infer failed");
        } else {
            len = build_response_json(resp, sizeof(resp), 1, "");
            fprintf(stderr, "[RESP] %s", resp);
        }
    }
    ssize_t wn = write(fd, resp, len);
    (void)wn;
    close(fd);
}

static int serve(int port) {
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { perror("socket"); return 1; }
    int on = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    struct sockaddr_in a = {0};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons(port);
    if (bind(srv, (struct sockaddr*)&a, sizeof(a)) < 0) {
        perror("bind"); return 1;
    }
    if (listen(srv, 4) < 0) { perror("listen"); return 1; }
    fprintf(stderr, "[SRV] listening on port %d\n", port);

    while (1) {
        struct sockaddr_in ca; socklen_t cl = sizeof(ca);
        int c = accept(srv, (struct sockaddr*)&ca, &cl);
        if (c < 0) {
            if (errno == EINTR) continue;
            perror("accept"); break;
        }
        handle_client(c);
    }
    close(srv);
    return 0;

}


int main(int argc, char **argv)
{
    int port = 9000;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--help")) {
            printf("Usage: %s [--port <N>]\n", argv[0]);
            return 0;
        }
    }

    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) { perror("/dev/mem (need sudo)"); return 1; }

    dma    = map_phys(DMA_PHYS,  0x1000);
    gpio   = map_phys(GPIO_PHYS, 0x1000);
    tx_buf = map_phys(TX_PHYS,   0x1000);
    rx_buf = map_phys(RX_PHYS,   0x1000);

    /* clear both pages so an old magic byte can't match */
    memset((void *)tx_buf, 0, 0x1000);
    memset((void *)rx_buf, 0, 0x1000);

    /* hold the core in reset while the DMA resets, so no stray stream data
     * lands in RX, then give it 10 ms to reach idle */
    gpio[GPIO_TRI]  = 0x00000000;
    gpio[GPIO_DATA] = 0x00000000;
    usleep(5000);

    dma_init();

    gpio[GPIO_DATA] = SW_RST_N;
    usleep(10000);
    fprintf(stderr, "[GPIO] core released (sw_rst_n=1)\n");

    /* status query (CMD 0x7, reply 0xB1): stop if the core doesn't answer */
    tx_buf[0] = (0x7u << 28);
    if (transact(4, 4, 0xB1, "status_query") != 0) {
        fprintf(stderr, "[FATAL] status query failed - core not ready\n");
        return 1;
    }
    uint32_t sr = rx_buf[0];
    fprintf(stderr, "[STATUS] response=0x%08x  core_ready=%u\n",
            sr, sr & 0x1);
    if (!(sr & 0x1)) {
        fprintf(stderr, "[FATAL] core reports not ready (core_ready bit 0)\n");
        return 1;
    }

    load_sequences();
    return serve(port);
}