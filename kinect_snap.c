// Captura um quadro RGB e um de profundidade do Kinect v1 e salva como PPM.
// Abre só a câmera (sem microfone/motor), o que evita o firmware de áudio do modelo 1473.
// Compilar: cc kinect_snap.c -I/opt/homebrew/include/libfreenect -L/opt/homebrew/lib -lfreenect -o kinect_snap
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "libfreenect.h"

#define SKIP 30  // quadros descartados enquanto o sensor estabiliza
static uint8_t rgb[640*480*3];
static uint16_t dep[640*480];
static int n_rgb = 0, n_dep = 0;

static void on_video(freenect_device *d, void *buf, uint32_t ts) {
    if (n_rgb++ == SKIP) memcpy(rgb, buf, sizeof rgb);
}
static void on_depth(freenect_device *d, void *buf, uint32_t ts) {
    if (n_dep++ == SKIP) memcpy(dep, buf, sizeof dep);
}
static void hue(float t, uint8_t *p) { // perto vermelho, longe azul
    float r = t < 0.5f ? 1 - 2*t : 0, g = t < 0.5f ? 2*t : 2 - 2*t, b = t < 0.5f ? 0 : 2*t - 1;
    p[0] = r * 255; p[1] = g * 255; p[2] = b * 255;
}

int main(void) {
    freenect_context *ctx; freenect_device *dev;
    if (freenect_init(&ctx, NULL) < 0) { fprintf(stderr, "init falhou\n"); return 1; }
    freenect_set_log_level(ctx, FREENECT_LOG_ERROR);
    freenect_select_subdevices(ctx, FREENECT_DEVICE_CAMERA);
    if (freenect_num_devices(ctx) < 1 || freenect_open_device(ctx, &dev, 0) < 0) {
        fprintf(stderr, "Kinect nao encontrado\n"); return 1;
    }
    freenect_set_video_mode(dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
    freenect_set_depth_mode(dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_MM));
    freenect_set_video_callback(dev, on_video);
    freenect_set_depth_callback(dev, on_depth);
    freenect_start_video(dev); freenect_start_depth(dev);
    while ((n_rgb <= SKIP || n_dep <= SKIP) && freenect_process_events(ctx) >= 0) {}
    freenect_stop_video(dev); freenect_stop_depth(dev);
    freenect_close_device(dev); freenect_shutdown(ctx);

    FILE *f = fopen("kinect_rgb.ppm", "wb");
    fprintf(f, "P6 640 480 255\n"); fwrite(rgb, 1, sizeof rgb, f); fclose(f);

    static uint8_t out[640*480*3];
    int valid = 0, mn = 99999, mx = 0;
    for (int i = 0; i < 640*480; i++) {
        if (!dep[i]) { out[i*3] = out[i*3+1] = out[i*3+2] = 0; continue; }
        valid++; if (dep[i] < mn) mn = dep[i]; if (dep[i] > mx) mx = dep[i];
        float t = (dep[i] - 500) / 4000.0f; if (t < 0) t = 0; if (t > 1) t = 1;
        hue(t, out + i*3);
    }
    f = fopen("kinect_depth.ppm", "wb");
    fprintf(f, "P6 640 480 255\n"); fwrite(out, 1, sizeof out, f); fclose(f);
    printf("OK: %.0f%% pixels com profundidade valida, de %d mm a %d mm\n", 100.0*valid/(640*480), mn, mx);
    return 0;
}
