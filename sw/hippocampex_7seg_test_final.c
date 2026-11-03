/*
 * hippocampex_7seg_test: trains the HippocamFLEX IP on the 7-segment digit
 * sequences over AXI DMA, then checks the recalls. Same cases as
 * tb_hippocampex_7seg.sv.
 *
 * build: aarch64-linux-gnu-gcc -O1 -Wall -static \
 *            -o hippocampex_7seg_test hippocampex_7seg_test_final.c
 *
 * Run as root once /sys/class/fpga_manager/fpga0/state says "operating".
 * If the kernel DMA driver is bound, unbind it first:
 *   echo a0000000.dma > /sys/bus/platform/drivers/xilinx_dma/unbind
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

/* DMA and GPIO from the block design, CMA base from dmesg | grep cma */
#define DMA_PHYS   0xA0000000UL
#define GPIO_PHYS  0xA0010000UL
#define TX_PHYS    0x6BA00000UL   /* CMA base */
#define RX_PHYS    0x6BA01000UL   /* CMA base + 4 KB */

/* AXI DMA registers, simple mode (PG021), as uint32_t indices */
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

/* AXI GPIO (PG144). Bit 0 drives the core's sw_rst_n (active low). */
#define GPIO_DATA   (0x000/4)
#define GPIO_TRI    (0x004/4)
#define SW_RST_N    (1u << 0)

/*
 * NUM_EVENTS must match the wrapper parameter (64).
 * Inference reply (magic 0xA1), 35 beats = 140 bytes:
 *   beat 0       header
 *   beats 1-16   merged voltages, 4 per beat    bytes 4..67
 *   beats 17-18  CA1 ordered one-hot, 64 bits   bytes 68..75
 *   beats 19-34  CA1 timestamps, 4 per beat     bytes 76..139
 */
#define NUM_EVENTS          64
#define HV_WORDS            4
#define INFER_RX_BYTES      140

/* byte offsets in rx_buf */
#define RX_OFF_VOLT         4
#define RX_OFF_ONEHOT       68    /* 2 x uint32, little endian */
#define RX_OFF_TS           76

/* recall voltages: present, future-1, future-2 */
#define VP   0x40u
#define VF1  0x80u
#define VF2  0xC0u

/* AP-CAM slots, same as the testbench */
#define CTX_RED     0
#define CTX_GREEN   1
#define CTX_BLUE    2
#define CTX_MAGENTA 3
#define CTX_WHITE   4

#define EVT_D1      0
#define EVT_D2      1
#define EVT_D3      2
#define EVT_D4      3
#define EVT_D5      4
#define EVT_D6      5
#define EVT_D7      6

/*
 * Hypervectors from tb_hippocampex_7seg.sv. Word 0 is HV[31:0] and word 3 is
 * HV[127:96], the same order the beats go out in CMD 0x1/0x2/0x6.
 */
static const uint32_t CTX_HV[5][HV_WORDS] = {
    [CTX_RED]     = {0xe682c9a3, 0x21a3feff, 0x92ba194a, 0xc6647f01},
    [CTX_GREEN]   = {0x8ff86ad0, 0xebe6d881, 0xb0818a19, 0xe1584820},
    [CTX_BLUE]    = {0xdda09711, 0xc4593447, 0xe468751e, 0xebb5204f},
    [CTX_MAGENTA] = {0xf5808b11, 0x81497ed7, 0xe2283d0e, 0xcef57b41},
    [CTX_WHITE]   = {0xcfa0cb91, 0xe1e3fcc7, 0xb0a8191a, 0xe3746801},
};

static const uint32_t EVT_HV[7][HV_WORDS] = {
    [EVT_D1] = {0x7f99b528, 0xbccb3593, 0x0a179ab4, 0x6023c479},
    [EVT_D2] = {0xd7ec5123, 0x7e4b5073, 0x7a170604, 0x444bcbd4},
    [EVT_D3] = {0xd5bc55ae, 0x7c4b103b, 0x6e973690, 0x4443cfd8},
    [EVT_D4] = {0xd5bffca0, 0x1c433002, 0x6f9636b4, 0xe0539910},
    [EVT_D5] = {0xd5a659a7, 0x7c4b305f, 0x6e9676d1, 0x4452dbf8},
    [EVT_D6] = {0xd72e59a3, 0x7ccb317b, 0x6e963e11, 0x6463dbf0},
    [EVT_D7] = {0xe7fbb4ae, 0x3c4b10bb, 0x6a379680, 0x6443c5fc},
};

static volatile uint32_t *dma  = NULL;
static volatile uint32_t *gpio = NULL;
static volatile uint32_t *tx_buf = NULL;
static volatile uint32_t *rx_buf = NULL;
static int mem_fd = -1;
static int pass_cnt = 0, fail_cnt = 0;
static uint8_t  rx_volt[NUM_EVENTS];
static uint64_t rx_ordered_onehot;          /* bit i = event i */
static uint8_t  rx_timestamps[NUM_EVENTS];  /* CA1 position, 0 = furthest future, 0x3F = not placed */

/* map a physical range through /dev/mem (opened O_SYNC, so uncached) */
static volatile uint32_t *map_phys(uint64_t phys, size_t sz)
{
    void *p = mmap(NULL, sz, PROT_READ|PROT_WRITE, MAP_SHARED,
                   mem_fd, (off_t)phys);
    if (p == MAP_FAILED) {
        fprintf(stderr, "[FATAL] mmap 0x%lx size %zu: %m\n", phys, sz);
        exit(1);
    }
    return (volatile uint32_t *)p;
}

/*
 * Reset one DMA channel and restart it. Not called; main uses dma_init().
 * Writing 0x1 instead of OR-ing in RS also clears bit 1 (keyhole/cyclic),
 * which the xilinx_dma driver can leave set. Soft reset doesn't clear it.
 */
#define RESET_LOOPS 10000000u

static void dma_reset_ch(int cr, int sr, const char *name)
{
    uint32_t n;
    dma[cr] = 0;
    usleep(500);
    dma[cr] = DMA_RESET;
    n = RESET_LOOPS;
    while ((dma[cr] & DMA_RESET) && --n);
    if (!n) {
        fprintf(stderr, "[FATAL] %s reset self-clear timeout CR=0x%08x\n",
                name, dma[cr]);
        exit(1);
    }
    usleep(500);
    dma[cr] = 0x00000001;   /* RS only */
    usleep(200);
    fprintf(stderr, "[DMA] %-8s CR=0x%08x SR=0x%08x\n",
            name, dma[cr], dma[sr]);
}

static void dma_init(void)
{
    uint32_t n;
    /* a reset on one channel resets the whole DMA */
    dma[MM2S_DMACR] = DMA_RESET;
    n = RESET_LOOPS;
    while ((dma[MM2S_DMACR] & DMA_RESET) && --n);
    if (!n) { fprintf(stderr,"[FATAL] DMA reset timeout\n"); exit(1); }
    usleep(1000);
    dma[MM2S_DMACR] = 0x00000001;
    dma[S2MM_DMACR] = 0x00000001;
    usleep(500);
    fprintf(stderr,"[DMA] MM2S CR=0x%08x SR=0x%08x\n", dma[MM2S_DMACR], dma[MM2S_DMASR]);
    fprintf(stderr,"[DMA] S2MM CR=0x%08x SR=0x%08x\n", dma[S2MM_DMACR], dma[S2MM_DMASR]);
}

/*
 * Poll byte 3 of rx_buf for the reply magic. The magic is tdata[31:24] and
 * S2MM writes little endian, so it lands in byte 3. This works the same for
 * 1-beat and 35-beat replies, so DMASR isn't polled. A timeout means the
 * core stopped answering, and later commands would fail too, so it exits.
 */
#define POLL_TIMEOUT_MS  2000

static void wait_magic(uint8_t magic, const char *ctx)
{
    volatile uint8_t *b = (volatile uint8_t *)rx_buf;
    int n = POLL_TIMEOUT_MS * 100;   /* 10 us per loop */
    while (n-- > 0) {
        if (b[3] == magic) return;
        usleep(10);
    }
    fprintf(stderr,
        "[FATAL] timeout waiting for magic=0x%02x ctx=%s\n"
        "  b[0..7] = %02x %02x %02x %02x %02x %02x %02x %02x\n"
        "  MM2S_SR=0x%08x  S2MM_SR=0x%08x\n"
        "  Checklist:\n"
        "  1. fpga_manager state = operating?\n"
        "       cat /sys/class/fpga_manager/fpga0/state\n"
        "  2. xilinx_dma driver unbound?\n"
        "       ls /sys/bus/platform/devices/a0000000.dma/driver\n"
        "  3. CMA base still 0x6BA00000?\n"
        "       dmesg | grep cma\n",
        magic, ctx,
        b[0],b[1],b[2],b[3],b[4],b[5],b[6],b[7],
        dma[MM2S_DMASR], dma[S2MM_DMASR]);
    exit(1);
}

/* one command and its reply: clear the old magic, arm S2MM, start MM2S, wait */
static void transact(uint32_t tx_bytes, uint32_t rx_max,
                     uint8_t expected_magic, const char *ctx)
{
    volatile uint8_t *b = (volatile uint8_t *)rx_buf;
    b[3] = 0x00;

    dma[S2MM_DA]     = (uint32_t)RX_PHYS;
    dma[S2MM_DA_MSB] = 0;
    dma[S2MM_LEN]    = rx_max;            /* arms S2MM */

    dma[MM2S_SA]     = (uint32_t)TX_PHYS;
    dma[MM2S_SA_MSB] = 0;
    dma[MM2S_LEN]    = tx_bytes;          /* starts MM2S */

    wait_magic(expected_magic, ctx);
}

/* command packets; field layout is in hippocampex_axis_wrapper.sv */

/* CMD 0x1: context HV, 5 beats */
static uint32_t pkt_ctx(int slot, const uint32_t hv[4])
{
    tx_buf[0] = (0x1u << 28) | ((uint32_t)slot & 0x7F);
    tx_buf[1] = hv[0]; tx_buf[2] = hv[1];
    tx_buf[3] = hv[2]; tx_buf[4] = hv[3];
    return 20;
}

/* CMD 0x2: event HV, 5 beats */
static uint32_t pkt_evt(int slot, const uint32_t hv[4])
{
    tx_buf[0] = (0x2u << 28) | ((uint32_t)slot & 0x7F);
    tx_buf[1] = hv[0]; tx_buf[2] = hv[1];
    tx_buf[3] = hv[2]; tx_buf[4] = hv[3];
    return 20;
}

/* CMD 0x3: crossbar link. [20:14] from, [13:7] to, [0] = 1 */
static uint32_t pkt_assoc(int from_e, int to_e)
{
    tx_buf[0] = (0x3u << 28)
              | (((uint32_t)from_e & 0x7F) << 14)
              | (((uint32_t)to_e   & 0x7F) <<  7)
              | 1u;
    return 4;
}

/* CMD 0x4: context gate. [20:14] event, [13:7] context, [0] = 1 */
static uint32_t pkt_gate(int evt, int ctx)
{
    tx_buf[0] = (0x4u << 28)
              | (((uint32_t)evt & 0x7F) << 14)
              | (((uint32_t)ctx & 0x7F) <<  7)
              | 1u;
    return 4;
}

/* CMD 0x5: MCAM time code. [20:14] event, [13:7] context, [5:0] code */
static uint32_t pkt_mcam(int evt, int ctx, int t)
{
    tx_buf[0] = (0x5u << 28)
              | (((uint32_t)evt & 0x7F) << 14)
              | (((uint32_t)ctx & 0x7F) <<  7)
              | ((uint32_t)t & 0x3F);
    return 4;
}

/* CMD 0x6: inference, context HV then event HV, 9 beats */
static uint32_t pkt_infer(const uint32_t c[4], const uint32_t e[4])
{
    tx_buf[0] = (0x6u << 28);
    tx_buf[1] = c[0]; tx_buf[2] = c[1]; tx_buf[3] = c[2]; tx_buf[4] = c[3];
    tx_buf[5] = e[0]; tx_buf[6] = e[1]; tx_buf[7] = e[2]; tx_buf[8] = e[3];
    return 36;
}

/* CMD 0x7: status query, 1 beat */
static uint32_t pkt_status(void)
{
    tx_buf[0] = (0x7u << 28);
    return 4;
}

/*
 * Header: [31:24] 0xA1, [23:17] context, [16:10] event, [7:0] active count
 * (the RTL currently sends a recall counter there). Voltage i is byte 4 + i.
 */
static void parse_infer(void)
{
    const uint8_t *b = (const uint8_t *)rx_buf;
    uint32_t h = rx_buf[0];
    printf("         ctx_idx=%u  evt_idx=%u  active=%u\n",
           (h >> 17) & 0x7F,
           (h >> 10) & 0x7F,
           h & 0xFF);

    for (int i = 0; i < NUM_EVENTS; i++)
        rx_volt[i] = b[RX_OFF_VOLT + i];

    uint32_t oh_lo, oh_hi;
    memcpy(&oh_lo, b + RX_OFF_ONEHOT,     4);
    memcpy(&oh_hi, b + RX_OFF_ONEHOT + 4, 4);
    rx_ordered_onehot = ((uint64_t)oh_hi << 32) | oh_lo;

    for (int i = 0; i < NUM_EVENTS; i++)
        rx_timestamps[i] = b[RX_OFF_TS + i];
}

/* training commands all answer with a 0xC1 ack */
static void train_ctx(int s, const uint32_t hv[4], const char *n)
{
    printf("  [CTX] slot=%d '%s'\n", s, n); fflush(stdout);
    transact(pkt_ctx(s, hv), 4, 0xC1, n);
}
static void train_evt(int s, const uint32_t hv[4], const char *n)
{
    printf("  [EVT] slot=%d '%s'\n", s, n); fflush(stdout);
    transact(pkt_evt(s, hv), 4, 0xC1, n);
}
static void train_assoc(int f, int t)
{
    transact(pkt_assoc(f, t), 4, 0xC1, "assoc");
}
static void train_gate(int e, int c)
{
    transact(pkt_gate(e, c), 4, 0xC1, "gate");
}
static void train_mcam(int e, int c, int t)
{
    transact(pkt_mcam(e, c, t), 4, 0xC1, "mcam");
}

static void infer(const uint32_t c[4], const uint32_t e[4], const char *l)
{
    printf("  [INF] %s\n", l); fflush(stdout);
    memset(rx_volt, 0, sizeof(rx_volt));
    transact(pkt_infer(c, e), INFER_RX_BYTES, 0xA1, l);
    parse_infer();
}

static void chk(const char *t, int e, uint8_t exp, const char *role)
{
    if (rx_volt[e] == exp) {
        printf("    [PASS] %-4s D%d=0x%02x  %s\n", t, e+1, rx_volt[e], role);
        pass_cnt++;
    } else {
        printf("    [FAIL] %-4s D%d=0x%02x (exp=0x%02x)  %s\n",
               t, e+1, rx_volt[e], exp, role);
        fail_cnt++;
    }
}
static void dump(void)
{
    for (int i = 0; i < NUM_EVENTS; i++) {
        if (!rx_volt[i]) continue;
        const char *r = (rx_volt[i]==VP)  ? "Present"  :
                        (rx_volt[i]==VF1) ? "Future-1" :
                        (rx_volt[i]==VF2) ? "Future-2" : "Decay";
        printf("    D%d(E%d)=0x%02x %s\n", i+1, i, rx_volt[i], r);
    }
}

/* CA1 must place future-2 before future-1 before present */
static void chk_order(const char *t,
                      int evt_f2, int evt_f1, int evt_p)
{
    uint8_t ts_f2 = rx_timestamps[evt_f2];
    uint8_t ts_f1 = rx_timestamps[evt_f1];
    uint8_t ts_p  = rx_timestamps[evt_p];
    if (ts_f2 < ts_f1 && ts_f1 < ts_p) {
        printf("    [PASS] %-4s CA1-order: ts[D%d]=%u < ts[D%d]=%u < ts[D%d]=%u\n",
               t, evt_f2+1, ts_f2, evt_f1+1, ts_f1, evt_p+1, ts_p);
        pass_cnt++;
    } else {
        printf("    [FAIL] %-4s CA1-order: ts[D%d]=%u  ts[D%d]=%u  ts[D%d]=%u\n",
               t, evt_f2+1, ts_f2, evt_f1+1, ts_f1, evt_p+1, ts_p);
        fail_cnt++;
    }
}

/* same check for two events */
static void chk_ts_lt(const char *t,
                      int evt_earlier, int evt_later,
                      const char *role)
{
    uint8_t ts_e = rx_timestamps[evt_earlier];
    uint8_t ts_l = rx_timestamps[evt_later];
    if (ts_e < ts_l) {
        printf("    [PASS] %-4s CA1-order: ts[D%d]=%u < ts[D%d]=%u  %s\n",
               t, evt_earlier+1, ts_e, evt_later+1, ts_l, role);
        pass_cnt++;
    } else {
        printf("    [FAIL] %-4s CA1-order: ts[D%d]=%u NOT < ts[D%d]=%u  %s\n",
               t, evt_earlier+1, ts_e, evt_later+1, ts_l, role);
        fail_cnt++;
    }
}

/* CA1 must not have placed the event (0x3F with CNTR_WIDTH=6) */
#define CA1_INACTIVE  0x3Fu
static void chk_ts_inactive(const char *t, int evt, const char *role)
{
    uint8_t ts = rx_timestamps[evt];
    if (ts == CA1_INACTIVE) {
        printf("    [PASS] %-4s CA1-inactive: ts[D%d]=0x%02x  %s\n",
               t, evt+1, ts, role);
        pass_cnt++;
    } else {
        printf("    [FAIL] %-4s CA1-inactive: ts[D%d]=0x%02x (exp=0x%02x)  %s\n",
               t, evt+1, ts, CA1_INACTIVE, role);
        fail_cnt++;
    }
}

int main(void)
{
    printf("-----------------------------------------\n");
    printf("  HippocamPEX 7-Segment Test  (final)\n");
    printf("  DMA=0x%08lx  GPIO=0x%08lx\n",
           (unsigned long)DMA_PHYS, (unsigned long)GPIO_PHYS);
    printf("  TX =0x%08lx  RX =0x%08lx\n",
           (unsigned long)TX_PHYS,  (unsigned long)RX_PHYS);
    printf("-----------------------------------------\n\n");
    fflush(stdout);

    /* the PL must be programmed, or the first access to 0xA0000000 hangs the CPU */
    {
        FILE *f = fopen("/sys/class/fpga_manager/fpga0/state", "r");
        if (f) {
            char state[32] = {0};
            (void)fgets(state, sizeof(state), f);
            fclose(f);
            char *nl = strchr(state, '\n');
            if (nl) *nl = '\0';
            printf("[FPGA] state = '%s'\n", state);
            fflush(stdout);
            if (strcmp(state, "operating") != 0) {
                fprintf(stderr,
                    "[FATAL] PL not programmed (state='%s').\n"
                    "  BOOT.BIN must include the bitstream:\n"
                    "    petalinux-package --boot --fsbl ... --fpga design_c2m_ip_wrapper.bit --u-boot\n",
                    state);
                return 1;
            }
        } else {
            fprintf(stderr,
                "[WARN] Cannot read fpga_manager state. Proceeding - "
                "if devmem hangs, PL is not programmed.\n");
        }
    }

    /* warn if the CMA base isn't the one TX_PHYS assumes */
    {
        FILE *f = popen("dmesg 2>/dev/null | grep 'cma: Reserved' | tail -1", "r");
        if (f) {
            char line[128] = {0};
            if (fgets(line, sizeof(line), f)) {
                fprintf(stderr, "[CMA] %s", line);
                if (!strstr(line, "6ba00000") && !strstr(line, "6BA00000")) {
                    fprintf(stderr,
                        "[WARN] CMA base in dmesg does not match TX_PHYS=0x%08lx.\n"
                        "  Recompile with correct TX_PHYS / RX_PHYS.\n",
                        (unsigned long)TX_PHYS);
                }
            }
            pclose(f);
        }
    }

    mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if (mem_fd < 0) {
        perror("[FATAL] open /dev/mem");
        fprintf(stderr, "  Run as root: sudo ./hippocampex_7seg_test\n");
        return 1;
    }

    dma    = map_phys(DMA_PHYS,  0x10000);
    gpio   = map_phys(GPIO_PHYS, 0x1000);
    tx_buf = map_phys(TX_PHYS,   0x2000);   /* 8 KB, covers the RX page too */
    rx_buf = map_phys(RX_PHYS,   0x1000);

    memset((void *)tx_buf, 0, 0x1000);
    memset((void *)rx_buf, 0, 0x1000);

    /* GPIO all outputs, hold the core in reset */
    gpio[GPIO_TRI]  = 0x00;
    gpio[GPIO_DATA] = 0x00;
    usleep(5000);

    /* reset the DMA while the core is held, so no stray stream data comes in */
    dma_init();

    gpio[GPIO_DATA] = SW_RST_N;
    usleep(10000);  /* let the core FSM reach idle */
    printf("[GPIO] Core released (sw_rst_n=1)\n\n");
    fflush(stdout);

    /* status query: if this times out, fix the DMA path before anything else */
    printf("[STATUS] Sending CMD 0x7 status query...\n");
    fflush(stdout);
    transact(pkt_status(), 4, 0xB1, "status_query");
    {
        uint32_t sr = rx_buf[0];
        printf("[STATUS] Response=0x%08x  core_ready=%u\n\n",
               sr, sr & 0x1);
        fflush(stdout);
    }

    printf("---- Phase 1: Context HVs ----\n");
    train_ctx(CTX_RED,     CTX_HV[CTX_RED],     "Red");
    train_ctx(CTX_GREEN,   CTX_HV[CTX_GREEN],   "Green");
    train_ctx(CTX_BLUE,    CTX_HV[CTX_BLUE],    "Blue");
    train_ctx(CTX_MAGENTA, CTX_HV[CTX_MAGENTA], "Magenta");
    train_ctx(CTX_WHITE,   CTX_HV[CTX_WHITE],   "White");

    printf("\n---- Phase 2: Event HVs ----\n");
    train_evt(EVT_D1, EVT_HV[EVT_D1], "Digit1_bc");
    train_evt(EVT_D2, EVT_HV[EVT_D2], "Digit2_abdeg");
    train_evt(EVT_D3, EVT_HV[EVT_D3], "Digit3_abcdg");
    train_evt(EVT_D4, EVT_HV[EVT_D4], "Digit4_bcfg");
    train_evt(EVT_D5, EVT_HV[EVT_D5], "Digit5_acdfg");
    train_evt(EVT_D6, EVT_HV[EVT_D6], "Digit6_acdefg");
    train_evt(EVT_D7, EVT_HV[EVT_D7], "Digit7_abc");

    /* links, gates and time codes, same as the testbench */
    printf("\n---- Phase 3: Crossbar + MCAM ----\n");

    /* BLUE 1->2->3->4->5, codes 1-5 */
    printf("  [BLUE] 1->2->3->4->5\n"); fflush(stdout);
    train_assoc(EVT_D1, EVT_D2); train_assoc(EVT_D2, EVT_D3);
    train_assoc(EVT_D3, EVT_D4); train_assoc(EVT_D4, EVT_D5);
    train_gate(EVT_D1, CTX_BLUE); train_gate(EVT_D2, CTX_BLUE);
    train_gate(EVT_D3, CTX_BLUE); train_gate(EVT_D4, CTX_BLUE);
    train_gate(EVT_D5, CTX_BLUE);
    train_mcam(EVT_D1, CTX_BLUE, 1); train_mcam(EVT_D2, CTX_BLUE, 2);
    train_mcam(EVT_D3, CTX_BLUE, 3); train_mcam(EVT_D4, CTX_BLUE, 4);
    train_mcam(EVT_D5, CTX_BLUE, 5);

    /* RED 7->5->3, codes 1-3 */
    printf("  [RED] 7->5->3\n"); fflush(stdout);
    train_assoc(EVT_D7, EVT_D5); train_assoc(EVT_D5, EVT_D3);
    train_gate(EVT_D7, CTX_RED); train_gate(EVT_D5, CTX_RED);
    train_gate(EVT_D3, CTX_RED);
    train_mcam(EVT_D7, CTX_RED, 1); train_mcam(EVT_D5, CTX_RED, 2);
    train_mcam(EVT_D3, CTX_RED, 3);

    /* GREEN 2->4->6, codes 1-3 */
    printf("  [GREEN] 2->4->6\n"); fflush(stdout);
    train_assoc(EVT_D2, EVT_D4); train_assoc(EVT_D4, EVT_D6);
    train_gate(EVT_D2, CTX_GREEN); train_gate(EVT_D4, CTX_GREEN);
    train_gate(EVT_D6, CTX_GREEN);
    train_mcam(EVT_D2, CTX_GREEN, 1); train_mcam(EVT_D4, CTX_GREEN, 2);
    train_mcam(EVT_D6, CTX_GREEN, 3);

    /* MAGENTA 1->3->5->7, codes 1-4 */
    printf("  [MAGENTA] 1->3->5->7\n"); fflush(stdout);
    train_assoc(EVT_D1, EVT_D3); train_assoc(EVT_D3, EVT_D5);
    train_assoc(EVT_D5, EVT_D7);
    train_gate(EVT_D1, CTX_MAGENTA); train_gate(EVT_D3, CTX_MAGENTA);
    train_gate(EVT_D5, CTX_MAGENTA); train_gate(EVT_D7, CTX_MAGENTA);
    train_mcam(EVT_D1, CTX_MAGENTA, 1); train_mcam(EVT_D3, CTX_MAGENTA, 2);
    train_mcam(EVT_D5, CTX_MAGENTA, 3); train_mcam(EVT_D7, CTX_MAGENTA, 4);

    /* WHITE 3->5->7, codes 1-3. MAGENTA already wrote the D3->D5 and D5->D7
     * links, so only gates and codes are added. */
    printf("  [WHITE] 3->5->7 (shared assoc edges, gating isolates)\n");
    fflush(stdout);
    train_gate(EVT_D3, CTX_WHITE); train_gate(EVT_D5, CTX_WHITE);
    train_gate(EVT_D7, CTX_WHITE);
    train_mcam(EVT_D3, CTX_WHITE, 1); train_mcam(EVT_D5, CTX_WHITE, 2);
    train_mcam(EVT_D7, CTX_WHITE, 3);

    printf("\n---- Training complete ----\n\n");
    fflush(stdout);

    /* recall cases T1-T7 from tb_hippocampex_7seg.sv */
    printf("==== Inference Tests ====\n");

    printf("\n[T1] Blue/D1\n");
    infer(CTX_HV[CTX_BLUE], EVT_HV[EVT_D1], "Blue/D1"); dump();
    chk("T1", EVT_D1, VP,  "Present");
    chk("T1", EVT_D2, VF1, "Future-1 D2");
    chk("T1", EVT_D3, VF2, "Future-2 D3");
    chk_order("T1", EVT_D3, EVT_D2, EVT_D1);

    printf("\n[T2] Red/D7\n");
    infer(CTX_HV[CTX_RED], EVT_HV[EVT_D7], "Red/D7"); dump();
    chk("T2", EVT_D7, VP,  "Present");
    chk("T2", EVT_D5, VF1, "Future-1 D5");
    chk("T2", EVT_D3, VF2, "Future-2 D3");
    chk_order("T2", EVT_D3, EVT_D5, EVT_D7);

    printf("\n[T3] Green/D2\n");
    infer(CTX_HV[CTX_GREEN], EVT_HV[EVT_D2], "Green/D2"); dump();
    chk("T3", EVT_D2, VP,  "Present");
    chk("T3", EVT_D4, VF1, "Future-1 D4");
    chk("T3", EVT_D6, VF2, "Future-2 D6");
    chk_order("T3", EVT_D6, EVT_D4, EVT_D2);

    printf("\n[T4] Magenta/D1\n");
    infer(CTX_HV[CTX_MAGENTA], EVT_HV[EVT_D1], "Magenta/D1"); dump();
    chk("T4", EVT_D1, VP,  "Present");
    chk("T4", EVT_D3, VF1, "Future-1 D3");
    chk("T4", EVT_D5, VF2, "Future-2 D5");
    chk_order("T4", EVT_D5, EVT_D3, EVT_D1);

    printf("\n[T5] White/D3\n");
    infer(CTX_HV[CTX_WHITE], EVT_HV[EVT_D3], "White/D3"); dump();
    chk("T5", EVT_D3, VP,  "Present");
    chk("T5", EVT_D5, VF1, "Future-1 D5");
    chk("T5", EVT_D7, VF2, "Future-2 D7");
    chk_order("T5", EVT_D7, EVT_D5, EVT_D3);

    /* T6: D2 comes before D3 in blue, so it must not show up as a future */
    printf("\n[T6] Blue/D3 sliding\n");
    infer(CTX_HV[CTX_BLUE], EVT_HV[EVT_D3], "Blue/D3"); dump();
    chk("T6", EVT_D3, VP,   "Present");
    chk("T6", EVT_D4, VF1,  "Future-1 D4");
    chk("T6", EVT_D5, VF2,  "Future-2 D5");
    chk("T6", EVT_D2, 0x00, "D2 NOT future in Blue/D3");
    chk_order("T6", EVT_D5, EVT_D4, EVT_D3);
    chk_ts_inactive("T6", EVT_D2, "D2 not placed by CA1");

    /* T7: D3 is in blue, red, magenta and white; each context must keep its own futures */
    printf("\n[T7a] Isolation Blue/D3\n");
    infer(CTX_HV[CTX_BLUE], EVT_HV[EVT_D3], "Blue/D3 isolate");
    chk("T7a", EVT_D3, VP,   "Present");
    chk("T7a", EVT_D4, VF1,  "F1=D4 (Blue)");
    chk("T7a", EVT_D7, 0x00, "D7 NOT in Blue");
    chk_ts_lt("T7a", EVT_D4, EVT_D3, "F1 before Present in CA1");
    chk_ts_inactive("T7a", EVT_D7, "D7 not placed by CA1 in Blue");

    /* D3 ends the red sequence, so nothing follows it */
    printf("\n[T7b] Isolation Red/D3 (terminus)\n");
    infer(CTX_HV[CTX_RED], EVT_HV[EVT_D3], "Red/D3 terminus"); dump();
    chk("T7b", EVT_D3, VP,   "Present");
    chk("T7b", EVT_D4, 0x00, "D4 NOT in Red");
    chk("T7b", EVT_D2, 0x00, "D2 NOT in Red");
    chk_ts_inactive("T7b", EVT_D4, "D4 not placed by CA1 in Red");
    chk_ts_inactive("T7b", EVT_D2, "D2 not placed by CA1 in Red");

    printf("\n-----------------------------------------\n");
    printf("  RESULTS: %d PASS  |  %d FAIL  (%.0f%%)\n",
           pass_cnt, fail_cnt,
           (pass_cnt + fail_cnt) ?
               100.0 * pass_cnt / (pass_cnt + fail_cnt) : 0.0);
    printf(fail_cnt ? "  *** %d FAILURE(S) ***\n"
                    : "  *** ALL CHECKS PASSED ***\n",
           fail_cnt);
    printf("-----------------------------------------\n");

    munmap((void *)dma,    0x10000);
    munmap((void *)gpio,   0x1000);
    munmap((void *)tx_buf, 0x2000);
    munmap((void *)rx_buf, 0x1000);
    close(mem_fd);
    return fail_cnt ? 1 : 0;
}
