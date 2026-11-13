// Nuvem de pontos ao vivo do Kinect v1 (OpenGL/GLUT, macOS).
// Compilar (veja build.sh, que tambem compila o Syphon):
//   ./build.sh
// Saida Syphon "Kinect": imagem 1920x1080 com transparencia, sem a interface (OBS: fonte Syphon Client)
//
// Controles:
//   arrastar com botao esquerdo  girar
//   arrastar com botao direito   zoom   (ou teclas Z / X)
//   E/C efeito: imagem real / profundidade / holograma / particulas (poeira que sobe) /
//       visao noturna (camera infravermelha: enxerga no escuro total) / mapa limao / mapa ciano / nuvem com aura
//   , . exposicao: escurece / clareia (vale para todos os efeitos, 0 volta ao normal)
//   A   rotacao automatica liga/desliga
//   + - tamanho do ponto
//   [ ] distancia maxima mostrada (corta o fundo)
//   B   remover fundo: conta 5 s (saia da frente), captura a sala vazia; B de novo desliga
//   V   grava/para video H.264 .mp4, fundo preto
//   T   grava/para video HEVC .mov com fundo transparente (canal alfa)
//   P   salva o quadro atual como .ply (abre no Blender)
//   H   mostra/esconde a ajuda na tela
//   O   esconde/mostra toda a interface (janela limpa para capturar no OBS como webcam)
//   R   volta a vista original      F  tela cheia      Q / Esc  sair
// Videos e .ply vao para ~/Documents/kinect
//
// Opcoes de linha de comando (para testes): --gravar | --gravar-alfa, --segundos N, --fundo, --efeito 0-7, --exposicao EV,
//   --trocar-efeito N (aperta E sozinho a cada N s)
#define GL_SILENCE_DEPRECATION
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include "libfreenect.h"
#include "libfreenect_registration.h"

// saida Syphon (syphon_out.m): o OBS no Mac le como fonte "Syphon Client"
int  syphon_start(const char *name);
int  syphon_clients(void);
void syphon_publish_pixels(const void *rgba, int w, int h);
void syphon_stop(void);
static int syphon_ok = 0;

#define W 640
#define H 480
#define N (W*H)
#define FOCAL 580.0f   // foco aproximado da camera infravermelha do Kinect v1 (pixels); a profundidade vem dela
#define REC_W 1920
#define REC_H 1080
#define FFMPEG "/opt/homebrew/bin/ffmpeg"
#define BG_COUNTDOWN_MS 5000
#define BG_FRAMES 30

// ---------- Kinect ----------
static freenect_context *ctx;
static freenect_device *dev;
static volatile int running = 1;
static pthread_t usb_thread;
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static uint8_t  rgb_in[N*3], rgb[N*3];
static uint16_t dep_in[N],   dep[N];
static uint8_t  ir_in[N], ir[N];       // imagem infravermelha (efeito visao noturna)
static int ir_mode = 0;                // 1 = Kinect mandando infravermelho em vez de cor
static volatile int fresh = 0;
static int frames = 0;
static volatile int video_frames = 0;   // quadros de cor recebidos (para detectar camera RGB parada)
static int video_restarts = 0, last_video_ms = 0, video_total_seen = 0;
static volatile int video_total = 0;   // total de quadros de imagem recebidos
// mapa profundidade -> cor (cada camera ve de um ponto diferente; o Kinect guarda a calibracao)
#define REG_X_VAL_SCALE 256            // ponto fixo usado pelo libfreenect na tabela de registro
#define DEPTH_MM_LIMIT 10000           // tamanho da tabela depth_to_rgb_shift
static freenect_registration reg;
static int reg_ok = 0;

// ---------- nuvem ----------
static float verts[N*3];
static uint8_t cols[N*3];
static uint8_t keep[N];
static int pidx[N];      // pixel de origem de cada ponto da nuvem
static int npts = 0;

// ---------- particulas (efeito "poeira") ----------
#define MAXP 80000
typedef struct { float x, y, z, vx, vy, vz, life, max; uint8_t r, g, b; } Particle;
static Particle parts[MAXP];
static int nparts = 0;
static float pverts[MAXP*3], pcols[MAXP*4];

// ---------- vista ----------
static float rot_x = 0, rot_y = 0, zoom = 2000, pivot = 2000;
static int color_mode = 0, auto_rot = 0, fullscreen = 0, show_help = 0;
static int hud_hidden = 0;  // tecla O: janela limpa (so a nuvem), para capturar no OBS
static float point_size = 2, max_depth = 4500;
static float exposure_ev = 0;  // exposicao em pontos (stops): +1 = dobro de luz, -1 = metade
static int drag_btn = -1, last_x, last_y;
static int win_w = 1280, win_h = 960;
#define NEFFECTS 8
#define FX_PARTICLES 3
#define FX_NIGHT 4
#define FX_LIME 5
#define FX_CYAN 6
#define FX_AURA 7
#define IS_MAP(m) ((m) == FX_LIME || (m) == FX_CYAN)
static const char *color_names[NEFFECTS] = { "imagem real", "profundidade", "holograma", "particulas", "visao noturna",
                                             "mapa limao", "mapa ciano", "nuvem com aura" };

// ---------- remocao de fundo ----------
enum { BG_OFF, BG_COUNTDOWN, BG_CAPTURE, BG_ON };
static int bg_state = BG_OFF, bg_t0 = 0, bg_got = 0;
static uint32_t bg_sum[N];
static uint16_t bg_cnt[N], bg[N];

// ---------- gravacao ----------
enum { REC_H264, REC_ALPHA };
static FILE *rec_pipe = NULL;
static int rec_kind = REC_H264, rec_frames = 0;
static char rec_name[1024];
static char out_dir[900];
static char flash_msg[256]; static int flash_until = 0;  // aviso temporario na tela
static GLuint fbo = 0, fbo_color, fbo_depth;
static uint8_t rec_buf[REC_W*REC_H*4];
static int opt_rec = -1, opt_secs = 0, opt_bg = 0, opt_reconnect_at = 0, opt_cycle = 0, last_cycle_ms = 0;
static int last_depth_ms = 0, last_reconnect_ms = 0, reconnects = 0;

static void quit(void);
static volatile sig_atomic_t want_quit = 0;  // Ctrl+C / kill: fecha soltando o Kinect direito
static void on_signal(int sig) { want_quit = 1; }
static void key(unsigned char k, int x, int y);

static void flash(const char *msg) {
    snprintf(flash_msg, sizeof flash_msg, "%s", msg);
    flash_until = glutGet(GLUT_ELAPSED_TIME) + 4000;
    printf("%s\n", msg); fflush(stdout);
}

// ---------- callbacks do Kinect ----------
static void on_video(freenect_device *d, void *buf, uint32_t ts) {
    // infravermelho vem em 640x488 (1 byte por pixel); usa as 480 primeiras linhas
    int is_ir = freenect_get_current_video_mode(d).video_format == FREENECT_VIDEO_IR_8BIT;
    pthread_mutex_lock(&mtx);
    if (is_ir) memcpy(ir_in, buf, sizeof ir_in); else memcpy(rgb_in, buf, sizeof rgb_in);
    video_frames++; video_total++; pthread_mutex_unlock(&mtx);
}
static void on_depth(freenect_device *d, void *buf, uint32_t ts) {
    pthread_mutex_lock(&mtx); memcpy(dep_in, buf, sizeof dep_in); fresh = 1; pthread_mutex_unlock(&mtx);
}
static void *usb_loop(void *arg) {
    // timeout curto: o laco nunca fica preso se o USB travar, e a reconexao consegue encerrar a thread
    struct timeval tv = { 0, 100000 };
    while (running) {
        if (freenect_process_events_timeout(ctx, &tv) < 0) { usleep(10000); }
    }
    return NULL;
}

// O Kinect 1 nao manda cor e infravermelho ao mesmo tempo (mesmo canal USB), mas a imagem pode ser
// trocada com tudo rodando, desde que a profundidade nao mude de modo (trocar o modo de profundidade
// no mesmo processo faz ela vir zerada). Por isso a profundidade fica sempre em milimetros, do ponto
// de vista da camera infravermelha, e o alinhamento com a cor e feito aqui (register_color).
static void dev_ir_note(void);
static void set_video_mode(void) {
    dev_ir_note();
    freenect_set_video_mode(dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM,
        ir_mode ? FREENECT_VIDEO_IR_8BIT : FREENECT_VIDEO_RGB));
}

static int kinect_open(void) {
    if (freenect_init(&ctx, NULL) < 0) { ctx = NULL; return 0; }
    freenect_set_log_level(ctx, FREENECT_LOG_FATAL);
    freenect_select_subdevices(ctx, FREENECT_DEVICE_CAMERA); // sem audio: evita firmware do modelo 1473
    if (freenect_num_devices(ctx) < 1 || freenect_open_device(ctx, &dev, 0) < 0) {
        freenect_shutdown(ctx); ctx = NULL; dev = NULL; return 0;
    }
    set_video_mode();
    freenect_set_depth_mode(dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_MM));
    freenect_set_video_callback(dev, on_video);
    freenect_set_depth_callback(dev, on_depth);
    freenect_start_video(dev); freenect_start_depth(dev);
    reg = freenect_copy_registration(dev); reg_ok = reg.registration_table && reg.depth_to_rgb_shift;
    running = 1;
    pthread_create(&usb_thread, NULL, usb_loop, NULL);
    return 1;
}

static void kinect_close(void) {
    if (!ctx) return;
    running = 0; pthread_join(usb_thread, NULL);
    if (dev) { freenect_stop_depth(dev); freenect_stop_video(dev); freenect_close_device(dev); dev = NULL; }
    if (reg_ok) { freenect_destroy_registration(&reg); reg_ok = 0; }
    freenect_shutdown(ctx); ctx = NULL;
}

// religa so o fluxo de imagem (cor ou infravermelho), sem tocar na profundidade; a thread USB
// fica parada durante a troca para nao disputar o dispositivo
static int is_reconnecting(void);
static void restart_video(void) {
    if (is_reconnecting() || !ctx || !dev) return;
    running = 0; pthread_join(usb_thread, NULL);
    freenect_stop_video(dev);
    set_video_mode();
    freenect_start_video(dev);
    running = 1; pthread_create(&usb_thread, NULL, usb_loop, NULL);
    last_video_ms = glutGet(GLUT_ELAPSED_TIME);
}

static void kinect_set_ir(int on) {
    if (on == ir_mode) return;
    ir_mode = on;
    restart_video();
}

// Reabrir o Kinect pode levar ate ~10 s. Isso roda numa thread separada para a janela nao congelar:
// enquanto isso a ultima nuvem continua na tela e o programa so ignora quadros e trocas de camera.
static volatile int reconnecting = 0, reconnect_done = 0, reconnect_ok = 0;
static pthread_t reconnect_thread;
static int dev_ir = 0;   // modo em que a imagem foi aberta (o efeito pode mudar durante a reconexao)

static void dev_ir_note(void) { dev_ir = ir_mode; }

static void *reconnect_worker(void *arg) {
    kinect_close();
    usleep(300000);
    int ok = 0;
    for (int k = 0; k < 4 && !(ok = kinect_open()); k++) usleep(600000); // a 1a tentativa costuma falhar
    reconnect_ok = ok;
    reconnect_done = 1;
    return NULL;
}

static void kinect_reconnect(const char *why) {
    if (reconnecting) return;
    reconnects++;
    last_reconnect_ms = glutGet(GLUT_ELAPSED_TIME);
    printf("%s Reconectando o Kinect (tentativa %d)...\n", why, reconnects); fflush(stdout);
    flash("Reconectando o Kinect...");
    reconnect_done = 0; reconnecting = 1;
    pthread_create(&reconnect_thread, NULL, reconnect_worker, NULL);
}

static int is_reconnecting(void) { return reconnecting; }

// chamado no laco principal: termina a reconexao quando a thread acabar
static void reconnect_poll(void) {
    if (!reconnecting || !reconnect_done) return;
    pthread_join(reconnect_thread, NULL);
    reconnecting = 0;
    flash(reconnect_ok ? "Kinect reconectado" : "Kinect sem sinal: tentando reconectar... confira cabo e fonte");
    last_depth_ms = last_video_ms = last_reconnect_ms = glutGet(GLUT_ELAPSED_TIME); // da tempo para os quadros voltarem
    if (reconnect_ok && dev_ir != ir_mode) restart_video();
}

// ---------- processamento da profundidade ----------
static void bg_start(void) {
    bg_state = BG_COUNTDOWN; bg_t0 = glutGet(GLUT_ELAPSED_TIME);
    printf("Remover fundo: saia da frente, capturando em 5 s\n"); fflush(stdout);
}

// chamado a cada quadro novo do Kinect
static void bg_update(void) {
    if (bg_state == BG_COUNTDOWN && glutGet(GLUT_ELAPSED_TIME) - bg_t0 >= BG_COUNTDOWN_MS) {
        memset(bg_sum, 0, sizeof bg_sum); memset(bg_cnt, 0, sizeof bg_cnt);
        bg_got = 0; bg_state = BG_CAPTURE;
    }
    if (bg_state != BG_CAPTURE) return;
    for (int i = 0; i < N; i++) if (dep[i]) { bg_sum[i] += dep[i]; bg_cnt[i]++; }
    if (++bg_got >= BG_FRAMES) {
        for (int i = 0; i < N; i++) bg[i] = bg_cnt[i] >= BG_FRAMES / 3 ? bg_sum[i] / bg_cnt[i] : 0;
        bg_state = BG_ON; flash("Fundo capturado: so o que mudou aparece. B desliga.");
    }
}

// decide quais pixels viram ponto: profundidade valida, dentro do corte e (se ligado) fora do fundo
static void compute_keep(void) {
    for (int i = 0; i < N; i++) {
        int d = dep[i];
        keep[i] = d > 0 && d <= max_depth;
        if (keep[i] && bg_state == BG_ON && bg[i]) {
            float thr = 40 + (float)bg[i] * bg[i] / 100000.0f; // ruido do Kinect cresce com a distancia
            if (d > bg[i] - thr) keep[i] = 0;
        }
    }
    if (bg_state != BG_ON) return;
    // limpeza: tira pontos soltos (menos de 4 vizinhos mantidos num 3x3)
    static uint8_t tmp[N];
    memcpy(tmp, keep, sizeof tmp);
    for (int y = 1; y < H-1; y++) for (int x = 1; x < W-1; x++) {
        int i = y*W + x; if (!tmp[i]) continue;
        int n = tmp[i-W-1] + tmp[i-W] + tmp[i-W+1] + tmp[i-1] + tmp[i+1] + tmp[i+W-1] + tmp[i+W] + tmp[i+W+1];
        if (n < 4) keep[i] = 0;
    }
}

static void hue(float t, uint8_t *p) { // perto vermelho -> verde -> azul longe
    float r = t < 0.5f ? 1 - 2*t : 0, g = t < 0.5f ? 2*t : 2 - 2*t, b = t < 0.5f ? 0 : 2*t - 1;
    p[0] = r * 255; p[1] = g * 255; p[2] = b * 255;
}

// visao noturna: a camera infravermelha so enxerga a luz dos pontinhos que o proprio Kinect projeta,
// por isso funciona no escuro total. Suaviza o pontilhado (media 3x3), ajusta o brilho sozinho e
// pinta de verde, como um oculos de visao noturna.
static uint8_t night[N];
static float night_f[N];
static void night_prepare(void) {
    static float gain = 1;
    int hist[256] = {0}, cnt = 0;
    for (int y = 1; y < H-1; y++) for (int x = 1; x < W-1; x++) {
        int i = y*W + x;
        float s = (ir[i-W-1] + ir[i-W] + ir[i-W+1] + ir[i-1] + ir[i] + ir[i+1] + ir[i+W-1] + ir[i+W] + ir[i+W+1]) / 9.0f;
        // a luz projetada enfraquece com a distancia: compensa em parte para o fundo aparecer sem ofuscar
        float dm = dep[i] ? dep[i] / 1000.0f : 1.0f, comp = dm; // compensacao parcial (a fisica pediria distancia^2)
        if (comp < 0.5f) comp = 0.5f; if (comp > 10) comp = 10;
        night_f[i] = s * comp;
        if (keep[i]) { int h = night_f[i]; hist[h > 255 ? 255 : h]++; cnt++; }
    }
    // ganho automatico: o pixel 97% mais claro vira quase branco (muda devagar, sem piscar)
    int acc = 0, p = 255;
    if (cnt) for (int v = 0; v < 256; v++) { acc += hist[v]; if (acc >= cnt * 0.97f) { p = v; break; } }
    float target = 235.0f / (p < 2 ? 2 : p); if (target > 16) target = 16;
    gain += (target - gain) * 0.1f;
    for (int i = 0; i < N; i++) { int v = night_f[i] * gain; night[i] = v > 255 ? 255 : v; }
}

// Para cada pixel de profundidade, acha o pixel correspondente na imagem colorida (mesma conta do
// libfreenect). Uma parte do que a camera infravermelha ve fica escondida para a camera colorida:
// nesses pontos color_at fica -1, senao eles pegariam a cor do objeto da frente.
static int color_at[N];
static void register_color(void) {
    static uint16_t zbuf[N];
    static int target[N];
    memset(zbuf, 0, sizeof zbuf);
    int offset = H * reg.reg_pad_info.start_lines;
    for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) {
        int i = y*W + x, d = dep[i], t = -1;
        if (d > 0 && d < DEPTH_MM_LIMIT) {
            int nx = (reg.registration_table[i][0] + reg.depth_to_rgb_shift[d]) / REG_X_VAL_SCALE;
            int ny = reg.registration_table[i][1];
            t = ny*W + nx - offset;
            if (nx < 0 || nx >= W || t < 0 || t >= N) t = -1;
            else if (!zbuf[t] || zbuf[t] > d) zbuf[t] = d;
        }
        target[i] = t;
    }
    for (int i = 0; i < N; i++) {
        int t = target[i], d = dep[i];
        color_at[i] = (t >= 0 && d - zbuf[t] < 30 + d / 40) ? t : -1; // folga: a mesma superficie nao "esconde" a si mesma
    }
}

// ---------- pessoa em cena (usada pelos mapas e pela nuvem com aura) ----------
// Acha a parede pela distancia mais comum ao fundo; o maior bloco bem a frente dela e a pessoa.
// Com "remover fundo" (B) ligado, usa esse recorte, que e mais preciso.
#define LOW_W 160
#define LOW_H 120
static uint8_t low_mask[LOW_W*LOW_H];
static uint8_t person[N];
static float wall_mm = 0, p_cx = 320, p_top = 100, p_bot = 440, aura_cx = 0;
static int have_person = 0;

static void person_update(void) {
    static int hist[401]; memset(hist, 0, sizeof hist);
    int cnt = 0;
    for (int i = 0; i < N; i++) { int d = dep[i]; if (d > 0 && d < 10000) { hist[d / 25]++; cnt++; } }
    if (!cnt) { have_person = 0; memset(person, 0, sizeof person); return; }
    int acc = 0, w = 400;
    for (int b = 0; b <= 400; b++) { acc += hist[b]; if (acc >= cnt * 0.9f) { w = b; break; } }
    float wmm = w * 25.0f;
    wall_mm = wall_mm ? wall_mm + (wmm - wall_mm) * 0.08f : wmm;

    // versao reduzida: cada celula e um bloco de 4x4 pixels com a profundidade media
    static float cell[LOW_W*LOW_H];
    for (int y = 0; y < LOW_H; y++) for (int x = 0; x < LOW_W; x++) {
        int n = 0, sum = 0;
        for (int b = 0; b < 4; b++) for (int a = 0; a < 4; a++) {
            int i = (y*4+b)*W + x*4+a;
            if (dep[i] > 0 && (bg_state != BG_ON || keep[i])) { n++; sum += dep[i]; }
        }
        cell[y*LOW_W + x] = n >= 8 ? (float)sum / n : 0;
    }
    // separa em blocos: dois vizinhos ficam juntos se nao ha salto de profundidade entre eles.
    // Parede e teto mudam aos poucos, mas entre o corpo e o que esta atras dele ha um salto.
    #define MAX_BLOBS 2048
    static int label[LOW_W*LOW_H], queue[LOW_W*LOW_H];
    static struct { int n, x0, x1, y0, y1; float sum, sx; } blob[MAX_BLOBS];
    memset(label, 0, sizeof label);
    int nb_blobs = 0;
    for (int s = 0; s < LOW_W*LOW_H && nb_blobs < MAX_BLOBS-1; s++) {
        if (!cell[s] || label[s]) continue;
        int id = ++nb_blobs, head = 0, tail = 0;
        memset(&blob[id], 0, sizeof blob[id]); blob[id].x0 = LOW_W; blob[id].y0 = LOW_H;
        queue[tail++] = s; label[s] = id;
        while (head < tail) {
            int c = queue[head++], x = c % LOW_W, y = c / LOW_W;
            blob[id].n++; blob[id].sum += cell[c]; blob[id].sx += x;
            if (x < blob[id].x0) blob[id].x0 = x;
            if (x > blob[id].x1) blob[id].x1 = x;
            if (y < blob[id].y0) blob[id].y0 = y;
            if (y > blob[id].y1) blob[id].y1 = y;
            int nb[4] = { x > 0 ? c-1 : -1, x < LOW_W-1 ? c+1 : -1, y > 0 ? c-LOW_W : -1, y < LOW_H-1 ? c+LOW_W : -1 };
            float lim = 40 + cell[c] / 25;
            for (int k = 0; k < 4; k++) if (nb[k] >= 0 && cell[nb[k]] && !label[nb[k]] && fabsf(cell[nb[k]] - cell[c]) < lim) {
                label[nb[k]] = id; queue[tail++] = nb[k];
            }
        }
    }
    // a pessoa: bloco grande, perto e central, que nao atravessa a imagem como parede ou teto
    int best = 0; float best_score = 0;
    for (int id = 1; id <= nb_blobs; id++) {
        if (blob[id].n < 80) continue;
        int bw = blob[id].x1 - blob[id].x0 + 1;
        int left = blob[id].x0 <= 1, right = blob[id].x1 >= LOW_W-2, top = blob[id].y0 <= 1;
        if ((left && right) || (top && bw > LOW_W * 0.6f)) continue;
        float mean = blob[id].sum / blob[id].n, cx = blob[id].sx / blob[id].n / LOW_W;
        float cent = 1 - fabsf(cx - 0.5f) * 1.2f, near = 2000 / mean;
        float score = blob[id].n * cent * near * near;
        if (score > best_score) { best_score = score; best = id; }
    }
    have_person = best != 0;
    float p_mean = have_person ? blob[best].sum / blob[best].n : 0;
    int bx0 = blob[best].x0 - LOW_W/6, bx1 = blob[best].x1 + LOW_W/6, by0 = blob[best].y0 - LOW_H/5;
    int top = LOW_H, bot = 0, sx = 0, sn = 0;
    for (int y = 0; y < LOW_H; y++) for (int x = 0; x < LOW_W; x++) {
        int c = y*LOW_W + x, id = label[c], on = 0;
        if (have_person && id) {
            if (id == best) on = 1;
            else { // maos e bracos a frente do corpo formam blocos separados
                float m = blob[id].sum / blob[id].n;
                on = blob[id].n >= 6 && m < p_mean + 200 && blob[id].x0 >= bx0 && blob[id].x1 <= bx1 && blob[id].y0 >= by0;
            }
        }
        low_mask[c] = on;
        if (on) { sx += x; sn++; if (y < top) top = y; if (y > bot) bot = y; }
    }
    for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) {
        int i = y*W + x, c = (y/4)*LOW_W + x/4;
        person[i] = low_mask[c] && dep[i] > 0 && fabsf(dep[i] - cell[c]) < 300;
    }
    if (have_person) { // movimento suave: as copias nao tremem junto com o ruido do sensor
        p_cx  += (sx * 4.0f / sn - p_cx) * 0.15f;
        p_top += (top * 4.0f - p_top) * 0.15f;
        p_bot += (bot * 4.0f + 4 - p_bot) * 0.15f;
    }
}

// ---------- mapas coloridos (imagem 2D 16:9) ----------
// Sao duas copias da silhueta: uma gigante e borrada, colorida em degrade, e uma pequena branca na frente.
#define MAP_W 854
#define MAP_H 480
static uint8_t map_px[MAP_W*MAP_H*4];
static GLuint map_tex = 0;
static uint8_t lut_lime[256][3], lut_cyan[256][3];
static float df[N], db[N], mb[N], mbb[N];

typedef struct { float t, r, g, b; } Stop;
static void lut_build(uint8_t lut[256][3], const Stop *st, int n) {
    for (int i = 0; i < 256; i++) {
        float t = i / 255.0f; int k = 0;
        while (k < n-2 && t > st[k+1].t) k++;
        float a = (t - st[k].t) / (st[k+1].t - st[k].t); if (a < 0) a = 0; if (a > 1) a = 1;
        lut[i][0] = st[k].r + (st[k+1].r - st[k].r) * a;
        lut[i][1] = st[k].g + (st[k+1].g - st[k].g) * a;
        lut[i][2] = st[k].b + (st[k+1].b - st[k].b) * a;
    }
}
static void luts_init(void) {
    static int done = 0; if (done) return; done = 1;
    static const Stop lime[] = { {0,0,0,0}, {.14f,6,10,62}, {.30f,8,88,92}, {.48f,18,170,20}, {.66f,150,230,30},
                                 {.82f,236,250,60}, {.92f,255,255,135}, {1,255,255,255} };
    static const Stop cyan[] = { {0,0,0,0}, {.20f,72,0,135}, {.38f,22,40,232}, {.56f,0,178,255}, {.76f,0,255,255}, {1,0,255,255} };
    lut_build(lut_lime, lime, 8); lut_build(lut_cyan, cyan, 6);
}

// media movel horizontal e vertical (custo nao depende do raio)
static void box_blur(const float *in, float *out, int r) {
    static float tmp[N];
    for (int y = 0; y < H; y++) {
        float sum = 0; const float *row = in + y*W;
        for (int x = -r; x <= r; x++) sum += row[x < 0 ? 0 : x];
        for (int x = 0; x < W; x++) {
            tmp[y*W+x] = sum / (2*r+1);
            sum += row[x+r+1 < W ? x+r+1 : W-1] - row[x-r > 0 ? x-r : 0];
        }
    }
    for (int x = 0; x < W; x++) {
        float sum = 0;
        for (int y = -r; y <= r; y++) sum += tmp[(y < 0 ? 0 : y)*W + x];
        for (int y = 0; y < H; y++) {
            out[y*W+x] = sum / (2*r+1);
            sum += tmp[(y+r+1 < H ? y+r+1 : H-1)*W + x] - tmp[(y-r > 0 ? y-r : 0)*W + x];
        }
    }
}
static inline float sample(const float *a, float x, float y) {
    if (x < 0 || y < 0 || x >= W-1 || y >= H-1) return 0;
    int ix = x, iy = y; float fx = x - ix, fy = y - iy; const float *p = a + iy*W + ix;
    return (p[0]*(1-fx) + p[1]*fx)*(1-fy) + (p[W]*(1-fx) + p[W+1]*fx)*fy;
}
static inline float sstep(float a, float b, float x) { float t = (x-a)/(b-a); t = t<0?0:t>1?1:t; return t*t*(3-2*t); }

static void map_build(void) {
    luts_init();
    int cyan = color_mode == FX_CYAN;
    uint8_t (*lut)[3] = cyan ? lut_cyan : lut_lime;
    // buracos da profundidade viram o valor do vizinho a esquerda
    for (int y = 0; y < H; y++) { float last = wall_mm; for (int x = 0; x < W; x++) {
        int i = y*W + x; if (dep[i]) last = dep[i]; df[i] = last; } }
    static float pf[N];
    for (int i = 0; i < N; i++) pf[i] = person[i];
    box_blur(df, db, 5); box_blur(pf, mb, 2); box_blur(pf, mbb, 8);
    float h = p_bot - p_top; if (h < 120) h = 120;
    float sb = 1.32f * MAP_H / h, big_cx = 0.42f * MAP_W, big_top = 0.07f * MAP_H;
    float ss = 0.40f * MAP_H / h, small_cx = 0.48f * MAP_W, small_top = 0.38f * MAP_H;
    float mul = exp2f(exposure_ev);
    uint8_t *o = map_px;
    for (int oy = 0; oy < MAP_H; oy++) for (int ox = 0; ox < MAP_W; ox++, o += 4) {
        float nx = ox / (float)MAP_W, ny = oy / (float)MAP_H;
        float bx = p_cx + (ox - big_cx) / sb, by = p_top + (oy - big_top) / sb;
        float bm = have_person ? sample(mbb, bx, by) : 0;
        bm = sstep(0.32f, 0.62f, bm);
        // corpo gigante: escuro embaixo e a esquerda, clareando para o verde
        float fx = (bx - (p_cx - h*0.32f)) / (h*0.64f); fx = fx<0?0:fx>1?1:fx;
        float fy = (by - p_top) / h; fy = fy<0?0:fy>1?1:fy;
        float t_body = 0.02f + 0.46f*fx + 0.10f*(1-fy) - 0.24f*fy*fy;
        // parede: quase uniforme, com o ruido do sensor e estourando para o branco nas bordas
        float sx0 = ox * (float)W / MAP_W, sy0 = 60 + oy * 360.0f / MAP_H;
        float grain = (sample(df, sx0, sy0) - sample(db, sx0, sy0)) / 40.0f * 0.05f;
        grain = grain < -0.12f ? -0.12f : grain > 0.12f ? 0.12f : grain;
        float ex = (nx-0.5f)/0.5f, ey = (ny-0.44f)/0.5f;
        float t_wall = cyan ? 0.86f : 0.76f + 0.12f*ex*ex + 0.34f*ey*ey + grain;
        float t = bm*t_body + (1-bm)*t_wall;
        float alpha = 1;
        if (cyan) { // area de captura com bordas curvas e fundo preto
            float e0 = (ny-0.5f)/0.5f, L = 0.12f + 0.09f*e0*e0, R = 0.81f + 0.015f*e0;
            float T = 0.05f + 0.05f*fabsf(nx-0.48f)*fabsf(nx-0.48f)*4, B = 0.965f;
            float e = nx-L; if (R-nx < e) e = R-nx; if (ny-T < e) e = ny-T; if (B-ny < e) e = B-ny;
            alpha = sstep(-0.004f, 0.012f, e);
            float lim = 0.18f + e / 0.028f; if (t > lim) t = lim;
        }
        int ti = t * 255; ti = ti<0?0:ti>255?255:ti;
        float r = lut[ti][0], g = lut[ti][1], b = lut[ti][2];
        // copia pequena branca na frente
        if (have_person) {
            float sm = sample(mb, p_cx + (ox - small_cx) / ss, p_top + (oy - small_top) / ss);
            if (sm > 0.5f) { r = g = b = 255; alpha = 1; }
            else if (sm > 0.3f && cyan) { r = 40; g = 255; b = 60; alpha = 1; }
        }
        int v;
        v = r*mul; o[0] = v>255?255:v; v = g*mul; o[1] = v>255?255:v; v = b*mul; o[2] = v>255?255:v;
        o[3] = alpha * 255;
    }
}

// a imagem cobre a tela toda; sobra corta nas laterais ou em cima e embaixo
static void map_draw(int w, int h) {
    if (!map_tex) {
        glGenTextures(1, &map_tex); glBindTexture(GL_TEXTURE_2D, map_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, MAP_W, MAP_H, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    }
    glBindTexture(GL_TEXTURE_2D, map_tex);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, MAP_W, MAP_H, GL_RGBA, GL_UNSIGNED_BYTE, map_px);
    float va = (float)w / (h ? h : 1), ta = (float)MAP_W / MAP_H, cu = 0, cv = 0;
    if (va > ta) cv = (1 - ta/va) / 2; else cu = (1 - va/ta) / 2;
    glDisable(GL_DEPTH_TEST); glDisable(GL_BLEND);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, 1, 0, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    glEnable(GL_TEXTURE_2D); glColor4f(1, 1, 1, 1);
    glBegin(GL_QUADS);
    glTexCoord2f(cu, 1-cv);   glVertex2f(0, 0);
    glTexCoord2f(1-cu, 1-cv); glVertex2f(1, 0);
    glTexCoord2f(1-cu, cv);   glVertex2f(1, 1);
    glTexCoord2f(cu, cv);     glVertex2f(0, 1);
    glEnd();
    glDisable(GL_TEXTURE_2D); glBindTexture(GL_TEXTURE_2D, 0);
}

static void build_cloud(void) {
    if (is_reconnecting()) return;   // a calibracao esta sendo trocada; mantem a ultima nuvem
    compute_keep();
    if (color_mode >= FX_LIME) person_update();
    if (IS_MAP(color_mode)) { map_build(); npts = 0; return; }
    if (color_mode == FX_AURA) {
        for (int i = 0; i < N; i++) if (!person[i]) keep[i] = 0;
        for (int y = 0; y < H; y++) { float last = wall_mm; for (int x = 0; x < W; x++) {
            int i = y*W + x; if (dep[i]) last = dep[i]; df[i] = last; } }
        box_blur(df, db, 2);
    }
    int need_color = color_mode == 0 || color_mode == FX_PARTICLES;
    if (need_color && reg_ok) register_color();
    else if (need_color) memset(color_at, 0xff, sizeof color_at);
    if (color_mode == FX_NIGHT) night_prepare();
    float focal = FOCAL;
    float expo_mul = exp2f(exposure_ev);
    npts = 0;
    for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) {
        int i = y*W + x; if (!keep[i]) continue;
        if (need_color && color_at[i] < 0) continue;  // sem cor conhecida: nao desenha
        float d = dep[i];
        float *v = verts + npts*3; uint8_t *c = cols + npts*3;
        v[0] = (x - W/2) * d / focal;
        v[1] = -(y - H/2) * d / focal;
        v[2] = -d;
        pidx[npts] = i;
        if (color_mode == 0) { memcpy(c, rgb + color_at[i]*3, 3); }
        else if (color_mode == FX_NIGHT) {
            float v = night[i] / 255.0f;
            if (y % 2) v *= 0.8f; // leve textura de linhas, como tela de visor
            c[0] = 60 * v; c[1] = 255 * v; c[2] = 90 * v;
        }
        else if (color_mode == FX_PARTICLES) { // base escurecida; o brilho vem das particulas
            for (int k = 0; k < 3; k++) c[k] = rgb[color_at[i]*3+k] * 0.35f;
        }
        else if (color_mode == FX_AURA) { // relevo: cinza iluminado de cima pela esquerda
            float shade = 0.35f;
            if (x >= 3 && x < W-3 && y >= 3 && y < H-3) {
                float px = 6 * d / focal;  // profundidade suavizada: some o degrau fino do sensor
                float gx = (db[i+3] - db[i-3]) / px, gy = (db[i+3*W] - db[i-3*W]) / px;
                float len = sqrtf(gx*gx + gy*gy + 1);
                float lam = (0.45f*gx - 0.55f*gy + 0.70f) / len; // normal (-gx, gy, 1) x luz (-.45,.55,.7)
                shade = lam < 0 ? 0 : powf(lam, 1.3f);
            }
            int g = 45 + 210 * shade; c[0] = c[1] = c[2] = g > 255 ? 255 : g;
        }
        else if (color_mode == 1) { float t = (d - 500) / 4000.0f; hue(t < 0 ? 0 : t > 1 ? 1 : t, c); }
        else { // holograma: ciano mais forte perto, com linhas horizontais
            float t = 1 - (d - 500) / 4000.0f; if (t < 0.15f) t = 0.15f; if (t > 1) t = 1;
            if ((y / 3) % 2) t *= 0.45f;
            c[0] = 40*t; c[1] = 220*t; c[2] = 255*t;
        }
        if (expo_mul != 1) for (int k = 0; k < 3; k++) { int v = c[k] * expo_mul; c[k] = v > 255 ? 255 : v; }
        npts++;
    }
}

// ---------- particulas ----------
static float frand(void) { return rand() / (float)RAND_MAX; }

// a cada quadro do Kinect, solta particulas de pontos sorteados da nuvem
static void particles_spawn(void) {
    if (!npts) return;
    int n = npts / 90; if (n > 900) n = 900;
    for (int k = 0; k < n && nparts < MAXP; k++) {
        int j = rand() % npts, i = pidx[j];
        Particle *p = &parts[nparts++];
        p->x = verts[j*3]; p->y = verts[j*3+1]; p->z = verts[j*3+2];
        p->vx = (frand() - 0.5f) * 120; p->vy = 100 + frand() * 280; p->vz = (frand() - 0.5f) * 120;
        p->max = p->life = 0.8f + frand() * 1.6f;
        // cor real do pixel, clareada
        float m = exp2f(exposure_ev);
        int t = color_at[i]; if (t < 0) { nparts--; continue; }
        int r = (rgb[t*3] * 1.2f + 15) * m, g = (rgb[t*3+1] * 1.2f + 20) * m, b = (rgb[t*3+2] * 1.2f + 30) * m;
        p->r = r > 255 ? 255 : r; p->g = g > 255 ? 255 : g; p->b = b > 255 ? 255 : b;
    }
}

// aura: nasce no contorno do corpo, um pouco atras dele, e segue um campo de vento comum,
// o que junta as particulas em fios
static void aura_spawn(void) {
    if (!npts) return;
    float sum = 0; for (int j = 0; j < npts; j += 16) sum += verts[j*3];
    aura_cx = sum / (npts / 16 + 1);
    int want = 1700, tries = want * 12;
    while (want > 0 && tries-- > 0 && nparts < MAXP) {
        int j = rand() % npts, i = pidx[j];
        int x = i % W, y = i / W;
        if (x < 3 || x >= W-3 || y < 3 || y >= H-3) continue;
        if (keep[i-3] && keep[i+3] && keep[i-3*W] && keep[i+3*W]) continue; // so a borda
        Particle *p = &parts[nparts++]; want--;
        p->x = verts[j*3]; p->y = verts[j*3+1]; p->z = verts[j*3+2] - 40 - frand() * 160;
        p->vx = p->vy = p->vz = 0;
        p->max = p->life = 1.6f + frand() * 2.2f;
        if (frand() < 0.85f) { p->r = 175 + frand()*65; p->g = 8 + frand()*18; p->b = 20 + frand()*20; }
        else { int g = 90 + frand()*70; p->r = p->g = p->b = g; }
    }
}
static void aura_update(float dt, float t) {
    int w = 0;
    for (int i = 0; i < nparts; i++) {
        Particle p = parts[i];
        p.life -= dt; if (p.life <= 0) continue;
        float side = p.x > aura_cx ? 1 : -1;
        p.vx = sinf(p.y * 0.0035f + p.z * 0.002f + t * 0.6f) * 70 + side * 22;
        p.vy = 260 + cosf(p.x * 0.006f + t * 0.4f) * 80;
        p.vz = sinf(p.x * 0.003f + t * 0.5f) * 40;
        p.x += p.vx * dt; p.y += p.vy * dt; p.z += p.vz * dt;
        parts[w++] = p;
    }
    nparts = w;
}

// move as particulas: sobem, espiralam num campo de "vento" e somem no fim da vida
static void particles_update(float dt, float t) {
    int w = 0;
    for (int i = 0; i < nparts; i++) {
        Particle p = parts[i];
        p.life -= dt; if (p.life <= 0) continue;
        p.vx += sinf(p.y * 0.004f + t * 1.3f) * 320 * dt;
        p.vz += cosf(p.x * 0.004f + t * 1.1f) * 320 * dt;
        p.vy += 80 * dt;
        p.x += p.vx * dt; p.y += p.vy * dt; p.z += p.vz * dt;
        parts[w++] = p;
    }
    nparts = w;
}

static void particles_draw(float ps_scale) {
    for (int i = 0; i < nparts; i++) {
        Particle *p = &parts[i]; float a = p->life / p->max; a = a * a;  // some suave
        pverts[i*3] = p->x; pverts[i*3+1] = p->y; pverts[i*3+2] = p->z;
        pcols[i*4] = p->r / 255.0f; pcols[i*4+1] = p->g / 255.0f; pcols[i*4+2] = p->b / 255.0f; pcols[i*4+3] = a;
    }
    // soma de luz (aditivo); o alfa tambem acumula para a gravacao transparente
    glEnable(GL_BLEND); glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE, GL_ONE, GL_ONE);
    glDepthMask(GL_FALSE); glEnable(GL_POINT_SMOOTH);
    glPointSize(point_size * (color_mode == FX_AURA ? 1.0f : 2.0f) * ps_scale);
    glEnableClientState(GL_VERTEX_ARRAY); glEnableClientState(GL_COLOR_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, pverts); glColorPointer(4, GL_FLOAT, 0, pcols);
    glDrawArrays(GL_POINTS, 0, nparts);
    glDisableClientState(GL_VERTEX_ARRAY); glDisableClientState(GL_COLOR_ARRAY);
    glDepthMask(GL_TRUE); glDisable(GL_POINT_SMOOTH); glDisable(GL_BLEND);
}

// ---------- arquivos ----------
static void out_path(char *dst, size_t n, const char *ext) {
    if (!out_dir[0]) {
        const char *home = getenv("HOME");
        snprintf(out_dir, sizeof out_dir, "%s/Documents/kinect", home ? home : ".");
    }
    mkdir(out_dir, 0755);
    char stamp[32]; time_t now = time(NULL);
    strftime(stamp, sizeof stamp, "%Y%m%d_%H%M%S", localtime(&now));
    snprintf(dst, n, "%s/kinect_%s.%s", out_dir, stamp, ext);
}

static void save_ply(void) {
    if (is_reconnecting()) { flash("Aguarde a reconexao do Kinect para salvar"); return; }
    char name[1024]; out_path(name, sizeof name, "ply");
    FILE *f = fopen(name, "w"); if (!f) { fprintf(stderr, "Nao consegui criar %s\n", name); return; }
    // cor real alinhada; na visao noturna nao ha cor, entao salva o infravermelho em cinza
    int use_rgb = !ir_mode && reg_ok;
    if (use_rgb) register_color();
    int n = 0; for (int i = 0; i < N; i++) n += keep[i] && (!use_rgb || color_at[i] >= 0);
    fprintf(f, "ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\n"
               "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n", n);
    for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) {
        int i = y*W + x; if (!keep[i] || (use_rgb && color_at[i] < 0)) continue; float d = dep[i];
        int r = ir[i], g = ir[i], b = ir[i];
        if (use_rgb) { int t = color_at[i]; r = rgb[t*3]; g = rgb[t*3+1]; b = rgb[t*3+2]; }
        // em metros, eixo Z para cima como no Blender
        fprintf(f, "%.4f %.4f %.4f %d %d %d\n", (x - W/2) * d / FOCAL / 1000, d / 1000,
                -(y - H/2) * d / FOCAL / 1000, r, g, b);
    }
    fclose(f);
    char msg[300]; snprintf(msg, sizeof msg, "Salvo %s (%d pontos)", strrchr(name, '/') + 1, n); flash(msg);
}

// ---------- desenho ----------
static void render_scene(int w, int h, float ps_scale) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    gluPerspective(49, (double)w / (h ? h : 1), 10, 30000); // ~campo vertical do Kinect
    glClearColor(0, 0, 0, 0); // alfa 0: fundo transparente na gravacao com alfa
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (IS_MAP(color_mode)) { map_draw(w, h); return; }
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    glTranslatef(0, 0, -zoom);
    glRotatef(rot_x, 1, 0, 0); glRotatef(rot_y, 0, 1, 0);
    glTranslatef(0, 0, pivot);
    glPointSize(point_size * ps_scale);
    glEnableClientState(GL_VERTEX_ARRAY); glEnableClientState(GL_COLOR_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, verts); glColorPointer(3, GL_UNSIGNED_BYTE, 0, cols);
    glDrawArrays(GL_POINTS, 0, npts);
    glDisableClientState(GL_VERTEX_ARRAY); glDisableClientState(GL_COLOR_ARRAY);
    if (color_mode == FX_PARTICLES || color_mode == FX_AURA) particles_draw(ps_scale);
}

// ---------- interface na tela (so na janela, nao entra no video) ----------
static void text(float x, float y, void *font, const char *s) {
    glRasterPos2f(x, y);
    for (; *s; s++) glutBitmapCharacter(font, *s);
}
static int text_w(void *font, const char *s) { int w = 0; for (; *s; s++) w += glutBitmapWidth(font, *s); return w; }
static void rect(float x0, float y0, float x1, float y1, float r, float g, float b, float a) {
    glColor4f(r, g, b, a);
    glBegin(GL_QUADS); glVertex2f(x0, y0); glVertex2f(x1, y0); glVertex2f(x1, y1); glVertex2f(x0, y1); glEnd();
}
static void circle(float cx, float cy, float rad) {
    glBegin(GL_TRIANGLE_FAN); glVertex2f(cx, cy);
    for (int k = 0; k <= 32; k++) glVertex2f(cx + rad * cosf(k * 6.2832f / 32), cy + rad * sinf(k * 6.2832f / 32));
    glEnd();
}
static void big_text(float cx, float cy, float height, const char *s) { // texto grande centralizado
    float w = 0; for (const char *p = s; *p; p++) w += glutStrokeWidth(GLUT_STROKE_ROMAN, *p);
    float sc = height / 119.0f;
    glPushMatrix(); glTranslatef(cx - w * sc / 2, cy, 0); glScalef(sc, sc, 1);
    glLineWidth(4);
    for (const char *p = s; *p; p++) glutStrokeCharacter(GLUT_STROKE_ROMAN, *p);
    glPopMatrix();
}

static void draw_hud(void) {
    if (hud_hidden) return;
    int t = glutGet(GLUT_ELAPSED_TIME);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); gluOrtho2D(0, win_w, 0, win_h); // y para cima
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    void *f18 = GLUT_BITMAP_HELVETICA_18, *f12 = GLUT_BITMAP_HELVETICA_12;
    char buf[300];

    // gravando: bolinha vermelha piscando + cronometro + formato
    if (rec_pipe) {
        int secs = rec_frames / 30;
        rect(10, win_h - 52, 330, win_h - 10, 0, 0, 0, 0.55f);
        glColor3f(1, 0.15f, 0.15f); if ((t / 500) % 2 == 0) circle(32, win_h - 31, 10);
        snprintf(buf, sizeof buf, "REC  %02d:%02d", secs / 60, secs % 60);
        glColor3f(1, 1, 1); text(52, win_h - 37, f18, buf);
        glColor3f(0.8f, 0.8f, 0.8f); text(170, win_h - 37, f12, rec_kind == REC_ALPHA ? "HEVC transparente" : "H.264 fundo preto");
    }

    // contagem para capturar o fundo
    if (bg_state == BG_COUNTDOWN || bg_state == BG_CAPTURE) {
        rect(0, 0, win_w, win_h, 0, 0, 0, 0.45f);
        glColor3f(1, 1, 1);
        if (bg_state == BG_COUNTDOWN) {
            int left = (BG_COUNTDOWN_MS - (t - bg_t0) + 999) / 1000; if (left < 1) left = 1;
            snprintf(buf, sizeof buf, "%d", left);
            big_text(win_w / 2.0f, win_h / 2.0f - 40, 140, buf);
            const char *m = "Saia da frente do Kinect: capturando o fundo";
            text((win_w - text_w(f18, m)) / 2.0f, win_h / 2.0f - 90, f18, m);
        } else {
            const char *m = "Capturando o fundo... fique fora da cena";
            text((win_w - text_w(f18, m)) / 2.0f, win_h / 2.0f, f18, m);
        }
    }

    // aviso temporario (arquivo salvo, etc.)
    if (t < flash_until) {
        int w = text_w(f18, flash_msg);
        rect((win_w - w) / 2.0f - 14, 52, (win_w + w) / 2.0f + 14, 86, 0, 0, 0, 0.7f);
        glColor3f(1, 1, 1); text((win_w - w) / 2.0f, 63, f18, flash_msg);
    }

    // barra de status embaixo
    snprintf(buf, sizeof buf, "Efeito: %s   |   Exposicao: %+.1f   |   Fundo: %s   |   Corte: %.2f m   |   Syphon: %s   |   H = ajuda   |   O = esconder interface",
             color_names[color_mode], exposure_ev,
             bg_state == BG_ON ? "removido" : bg_state == BG_OFF ? "normal" : "capturando",
             max_depth / 1000.0f,
             !syphon_ok ? "erro" : syphon_clients() ? "enviando" : "aguardando OBS");
    rect(0, 0, win_w, 26, 0, 0, 0, 0.55f);
    glColor3f(0.85f, 0.85f, 0.85f); text(10, 8, f12, buf);

    // ajuda
    if (show_help) {
        static const char *lines[] = {
            "ATALHOS",
            "",
            "V        grava / para  -  H.264 .mp4, fundo preto",
            "T        grava / para  -  HEVC .mov, fundo transparente",
            "B        remover fundo (conta 5 s, saia da frente)  /  desligar",
            "E ou C   efeito: real / profund. / holograma / particulas / noturna / mapas / aura",
            ",  .  0  exposicao: escurece / clareia / normal",
            "A        balanco automatico da camera",
            "[  ]     distancia maxima (corta o fundo)",
            "+  -     tamanho do ponto",
            "arrastar         girar          botao direito / Z X   zoom",
            "R        vista original          F   tela cheia",
            "P        salva quadro 3D (.ply)",
            "O        esconde / mostra a interface (janela limpa p/ OBS)",
            "K        reconecta o Kinect (se a imagem congelar)",
            "Q        sair",
            "",
            "Arquivos em Documentos / kinect",
        };
        int n = sizeof lines / sizeof *lines, lh = 24, bw = 560, bh = n * lh + 30;
        float x0 = (win_w - bw) / 2.0f, y1 = (win_h + bh) / 2.0f;
        rect(x0, y1 - bh, x0 + bw, y1, 0, 0, 0, 0.8f);
        for (int k = 0; k < n; k++) {
            if (k == 0) glColor3f(0.4f, 0.85f, 1); else glColor3f(1, 1, 1);
            text(x0 + 24, y1 - 30 - k * lh, f18, lines[k]);
        }
    }
    glDisable(GL_BLEND);
}

static void display(void) {
    render_scene(win_w, win_h, 1);
    draw_hud();
    glutSwapBuffers();
}

static void reshape(int w, int h) { win_w = w; win_h = h ? h : 1; }

static void update_title(void) {
    glutSetWindowTitle(rec_pipe ? "\xE2\x97\x8F GRAVANDO - Kinect - nuvem de pontos" : "Kinect - nuvem de pontos");
}

// ---------- gravacao ----------
static int ensure_fbo(void) {
    if (fbo) return 1;
    glGenFramebuffersEXT(1, &fbo);
    glGenRenderbuffersEXT(1, &fbo_depth);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    // cor numa textura: serve para gravar (glReadPixels) e para publicar no Syphon
    glGenTextures(1, &fbo_color);
    glBindTexture(GL_TEXTURE_2D, fbo_color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, REC_W, REC_H, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glBindTexture(GL_TEXTURE_2D, 0);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, fbo_color, 0);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, fbo_depth);
    glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, REC_W, REC_H);
    glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, fbo_depth);
    GLenum st = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
    if (st != GL_FRAMEBUFFER_COMPLETE_EXT) { fprintf(stderr, "Erro ao preparar gravacao (FBO %x)\n", st); glDeleteFramebuffersEXT(1, &fbo); fbo = 0; return 0; }
    return 1;
}

static void rec_start(int kind) {
    if (!ensure_fbo()) return;
    rec_kind = kind;
    out_path(rec_name, sizeof rec_name, kind == REC_ALPHA ? "mov" : "mp4");
    // vflip: o OpenGL le de baixo para cima; cores marcadas como Rec.709 para o editor interpretar certo
    const char *tags = "-color_primaries bt709 -color_trc bt709 -colorspace bt709";
    char cmd[1600];
    if (kind == REC_ALPHA)
        snprintf(cmd, sizeof cmd, FFMPEG " -loglevel error -y -f rawvideo -pix_fmt rgba -s %dx%d -r 30 -i - "
                 "-vf vflip,scale=out_color_matrix=bt709:out_range=tv,format=bgra "
                 "-c:v hevc_videotoolbox -q:v 65 -alpha_quality 0.75 -tag:v hvc1 %s '%s'", REC_W, REC_H, tags, rec_name);
    else
        snprintf(cmd, sizeof cmd, FFMPEG " -loglevel error -y -f rawvideo -pix_fmt rgb24 -s %dx%d -r 30 -i - "
                 "-vf vflip,scale=out_color_matrix=bt709:out_range=tv,format=yuv420p "
                 "-c:v h264_videotoolbox -b:v 25M -profile:v high -tag:v avc1 -movflags +faststart %s '%s'",
                 REC_W, REC_H, tags, rec_name);
    rec_pipe = popen(cmd, "w");
    if (!rec_pipe) { flash("Nao consegui abrir o ffmpeg"); return; }
    rec_frames = 0; update_title();
    printf("Gravando: %s\n", rec_name); fflush(stdout);
}

static void rec_stop(void) {
    if (!rec_pipe) return;
    pclose(rec_pipe); rec_pipe = NULL; update_title();
    char msg[300]; snprintf(msg, sizeof msg, "Salvo %s (%.1f s)", strrchr(rec_name, '/') + 1, rec_frames / 30.0);
    flash(msg);
}

static void rec_toggle(int kind) { if (rec_pipe) rec_stop(); else rec_start(kind); }

// tamanho do ponto nas saidas 1920x1080 (gravacao e Syphon), proporcional a janela.
// arredonda para cima com folga: ponto fracionario deixava uma grade de frestas no video
static float out_point_scale(void) { return ceilf(point_size * REC_H / win_h * 1.2f) / point_size; }

// desenha a cena em 1920x1080 na textura da gravacao
static void render_offscreen(void) {
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    render_scene(REC_W, REC_H, out_point_scale());
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

static void rec_frame(void) {
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    int alpha = rec_kind == REC_ALPHA;
    glReadPixels(0, 0, REC_W, REC_H, alpha ? GL_RGBA : GL_RGB, GL_UNSIGNED_BYTE, rec_buf);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
    // particulas somam luz (cor "pre-multiplicada"); o video transparente espera cor pura
    if (alpha && (color_mode == FX_PARTICLES || color_mode == FX_AURA)) {
        for (size_t k = 0; k < (size_t)REC_W * REC_H * 4; k += 4) {
            int a = rec_buf[k+3];
            if (a > 0 && a < 255) for (int c = 0; c < 3; c++) {
                int v = rec_buf[k+c] * 255 / a; rec_buf[k+c] = v > 255 ? 255 : v;
            }
        }
    }
    size_t sz =(size_t)REC_W * REC_H * (alpha ? 4 : 3);
    if (fwrite(rec_buf, 1, sz, rec_pipe) != sz) { fprintf(stderr, "ffmpeg parou de aceitar quadros\n"); rec_stop(); return; }
    rec_frames++;
}

// ---------- laco principal ----------
static void tick(int v) {
    int got = 0;
    if (opt_bg) { opt_bg = 0; bg_start(); }
    if (opt_rec >= 0) { rec_start(opt_rec); opt_rec = -1; }
    if (want_quit || (opt_secs && glutGet(GLUT_ELAPSED_TIME) > opt_secs * 1000)) quit();
    reconnect_poll();
    pthread_mutex_lock(&mtx);
    if (fresh && !reconnecting) {
        memcpy(dep, dep_in, sizeof dep);
        if (ir_mode) memcpy(ir, ir_in, sizeof ir); else memcpy(rgb, rgb_in, sizeof rgb);
        fresh = 0; got = 1;
    }
    pthread_mutex_unlock(&mtx);
    if (auto_rot) rot_y = 25 * sinf(glutGet(GLUT_ELAPSED_TIME) / 1500.0f);
    static int last_ms = 0; int now_ms = glutGet(GLUT_ELAPSED_TIME);
    float dt = last_ms ? (now_ms - last_ms) / 1000.0f : 0; last_ms = now_ms;
    if (dt > 0.1f) dt = 0.1f;
    if (color_mode == FX_PARTICLES) particles_update(dt, now_ms / 1000.0f);
    if (color_mode == FX_AURA) aura_update(dt, now_ms / 1000.0f);
    // vigia: 2 s sem profundidade = conexao USB caiu; reconecta (no maximo a cada 4 s)
    if (got) last_depth_ms = now_ms;
    else if (!last_depth_ms) last_depth_ms = now_ms;
    if (!reconnecting && now_ms - last_depth_ms > 2000 && now_ms - last_reconnect_ms > 4000)
        kinect_reconnect("Kinect parou de mandar imagem.");
    // imagem parada enquanto a profundidade chega (acontece ao ligar): religa so o fluxo de imagem
    if (video_total != video_total_seen) { video_total_seen = video_total; last_video_ms = now_ms; }
    else if (!last_video_ms) last_video_ms = now_ms;
    if (dev && got && now_ms - last_video_ms > 1200) {
        video_restarts++;
        restart_video();
        printf(ir_mode ? "Camera infravermelha sem sinal: religando...\n" : "Camera colorida sem sinal: religando...\n");
    }
    if (opt_cycle && now_ms - last_cycle_ms > opt_cycle * 1000) {
        if (last_cycle_ms) key('e', 0, 0);
        last_cycle_ms = now_ms;
    }
    if (opt_reconnect_at && now_ms > opt_reconnect_at * 1000) { opt_reconnect_at = 0; kinect_reconnect("Teste:"); }
    if (got) {
        bg_update();
        frames++; build_cloud();
        if (color_mode == FX_PARTICLES) particles_spawn();
        if (color_mode == FX_AURA) aura_spawn();
        // 1 quadro de saida por quadro do Kinect; so desenha fora da tela se alguem vai usar
        int to_syphon = syphon_ok && syphon_clients();
        if ((rec_pipe || to_syphon) && ensure_fbo()) {
            render_offscreen();
            if (to_syphon) {
                static uint8_t sy_buf[REC_W*REC_H*4];
                glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
                glPixelStorei(GL_PACK_ALIGNMENT, 1);
                glReadPixels(0, 0, REC_W, REC_H, GL_RGBA, GL_UNSIGNED_BYTE, sy_buf);
                glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
                syphon_publish_pixels(sy_buf, REC_W, REC_H);
            }
            if (rec_pipe) rec_frame();
        }
    }
    glutPostRedisplay();
    static int last_report = 0; int t = glutGet(GLUT_ELAPSED_TIME);
    if (t - last_report > 3000) {
        int vf = video_frames; video_frames = 0;
        printf("%.1f quadros/s, cor %.1f quadros/s, %d pontos\n",
               frames * 1000.0 / (t - last_report), vf * 1000.0 / (t - last_report), npts); fflush(stdout);
        frames = 0; last_report = t;
    }
    glutTimerFunc(16, tick, 0);
}

static void quit(void) {
    rec_stop();
    if (syphon_ok) syphon_stop();
    if (reconnecting) pthread_join(reconnect_thread, NULL);
    kinect_close();
    exit(0);
}

static void key(unsigned char k, int x, int y) {
    switch (k) {
    case 'q': case 'Q': case 27: quit(); break;
    case 'c': case 'C': case 'e': case 'E':
        color_mode = (color_mode + 1) % NEFFECTS; nparts = 0;
        kinect_set_ir(color_mode == FX_NIGHT);
        build_cloud(); break;
    case ',': case '<': case '.': case '>': case '0': {
        if (k == '0') exposure_ev = 0;
        else exposure_ev += (k == '.' || k == '>') ? 0.5f : -0.5f;
        if (exposure_ev > 3) exposure_ev = 3; if (exposure_ev < -3) exposure_ev = -3;
        char m[96]; snprintf(m, sizeof m, "Exposicao %+.1f   ( , escurece   . clareia   0 normal )", exposure_ev);
        flash(m); build_cloud(); break; }
    case 'a': case 'A': auto_rot = !auto_rot; if (!auto_rot) rot_y = 0; break;
    case '+': case '=': point_size += 1; break;
    case '-': if (point_size > 1) point_size -= 1; break;
    case ']': max_depth += 250; build_cloud(); break;
    case '[': if (max_depth > 750) max_depth -= 250; build_cloud(); break;
    case 'z': case 'Z': zoom *= 0.9f; break;
    case 'x': case 'X': zoom *= 1.1f; break;
    case 'p': case 'P': save_ply(); break;
    case 'v': case 'V': rec_toggle(REC_H264); break;
    case 't': case 'T': rec_toggle(REC_ALPHA); break;
    case 'b': case 'B':
        if (bg_state == BG_OFF) bg_start();
        else { bg_state = BG_OFF; build_cloud(); flash("Remocao de fundo desligada"); }
        break;
    case 'k': case 'K': kinect_reconnect("Pedido manual."); break;
    case 'h': case 'H': show_help = !show_help; if (show_help) hud_hidden = 0; break;
    case 'o': case 'O': hud_hidden = !hud_hidden; if (hud_hidden) show_help = 0;
        printf(hud_hidden ? "Interface escondida (O mostra de novo)\n" : "Interface visivel\n"); fflush(stdout); break;
    case 'r': case 'R': rot_x = rot_y = 0; zoom = pivot; auto_rot = 0; break;
    case 'f': case 'F': fullscreen = !fullscreen;
        if (fullscreen) glutFullScreen(); else { glutReshapeWindow(1280, 960); glutPositionWindow(80, 80); } break;
    }
}
static void mouse(int b, int s, int x, int y) { drag_btn = s == GLUT_DOWN ? b : -1; last_x = x; last_y = y; }
static void motion(int x, int y) {
    int dx = x - last_x, dy = y - last_y; last_x = x; last_y = y;
    if (drag_btn == GLUT_LEFT_BUTTON) { rot_y += dx * 0.4f; rot_x += dy * 0.4f; }
    else if (drag_btn == GLUT_RIGHT_BUTTON) { zoom *= 1 + dy * 0.005f; if (zoom < 100) zoom = 100; }
}

int main(int argc, char **argv) {
    signal(SIGPIPE, SIG_IGN); // se o ffmpeg cair, nao derruba o programa
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    setvbuf(stdout, NULL, _IOLBF, 0); // mensagens saem na hora, mesmo indo para arquivo
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--gravar")) opt_rec = REC_H264;
        else if (!strcmp(argv[i], "--gravar-alfa")) opt_rec = REC_ALPHA;
        else if (!strcmp(argv[i], "--fundo")) opt_bg = 1;
        else if (!strcmp(argv[i], "--reconectar-em") && i + 1 < argc) opt_reconnect_at = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--efeito") && i + 1 < argc) color_mode = atoi(argv[++i]) % NEFFECTS;
        else if (!strcmp(argv[i], "--trocar-efeito") && i + 1 < argc) opt_cycle = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--corte") && i + 1 < argc) max_depth = atof(argv[++i]);
        else if (!strcmp(argv[i], "--ponto") && i + 1 < argc) point_size = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tela-cheia")) fullscreen = 1;
        else if (!strcmp(argv[i], "--sem-interface")) hud_hidden = 1;
        else if (!strcmp(argv[i], "--exposicao") && i + 1 < argc) exposure_ev = atof(argv[++i]);
        else if (!strcmp(argv[i], "--segundos") && i + 1 < argc) opt_secs = atoi(argv[++i]);
    }
    if (color_mode == FX_NIGHT) ir_mode = 1;
    // tenta algumas vezes: logo apos outro programa fechar, o Kinect pode demorar a liberar
    int ok = 0;
    for (int k = 0; k < 5 && !(ok = kinect_open()); k++) usleep(700000);
    if (!ok) { fprintf(stderr, "Kinect nao encontrado. Confira a fonte e o USB.\n"); return 1; }

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(1280, 960);
    glutCreateWindow("Kinect - nuvem de pontos");
    if (fullscreen) glutFullScreen();
    syphon_ok = syphon_start("Kinect");
    printf(syphon_ok ? "Syphon ativo: no OBS, fonte \"Syphon Client\" > Kinect\n" : "Syphon nao iniciou\n");
    glutDisplayFunc(display); glutReshapeFunc(reshape);
    glutKeyboardFunc(key); glutMouseFunc(mouse); glutMotionFunc(motion);
    glutTimerFunc(16, tick, 0);
    printf("Nuvem de pontos ao vivo. H mostra os atalhos.\n"); fflush(stdout);
    glutMainLoop();
    return 0;
}
