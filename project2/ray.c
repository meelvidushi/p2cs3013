#include <stdio.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <semaphore.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "ray_ast.h"
#include "ray_math.h"
#include "ray.yacc.generated_h"
#include "ray.lex.generated_h"
#include "ray_render.h"
#include "ray_bmp.h"
#include "ray_physics.h"
#include "ray_console.h"

#define CHECK(x)	do { if (!(x)) { fprintf(stderr, "%s:%d CHECK failed: %s, errno %d %s\n", __FILE__, __LINE__, #x, errno, strerror(errno)); abort(); } } while(0)


// before multithreading (original code):
// real	1m16.115s
// user	1m15.657s
// sys	0m0.441s

// after multithreading (this version):
// real  0m18.775s
// user  0m18.530s
// sys   0m0.188s

void render_scene(struct framebuffer_pt4 *fb, const struct context* ctx);

typedef struct {
    struct framebuffer_pt4 *fb;
    struct context *ctx;
} render_data_t;


//thread 1 velocity calcs
void *thread_calc_vel(void *arg) {
    struct context *ctx = (struct context *)arg;
    calc_velocities(ctx);
    return NULL;
}

// thread 2 rendering
void *thread_render(void *arg) {
    render_data_t *rd = (render_data_t *)arg;
    render_scene(rd->fb, rd->ctx);
    return NULL;
}

int main(int argc, char **argv) {
    // 2 framebuffers (double buffering)
    struct framebuffer_pt4 *fb[2];
    fb[0] = new_framebuffer_pt4(1024, 768);
    fb[1] = new_framebuffer_pt4(1024, 768);

    pthread_t file_write_thread;

    for (int frame = 0; frame < 100; frame++) {
        char prev_filepath[128];
        snprintf(prev_filepath, sizeof(prev_filepath), "%s-%05d.bmp", argv[2], frame - 1);

        if (frame > 0) {
            //  previous frame is written to file in separate thread
            pthread_create(&file_write_thread, NULL, thread_write_bmp, (void*)prev_filepath);
        }

        // the new frame is rendered into current buffer
        pthread_t tid_vel, tid_render;
        render_data_t rd = { fb[frame % 2], ctx };
        pthread_create(&tid_render, NULL, thread_render, &rd);
        pthread_create(&tid_vel, NULL, thread_calc_vel, ctx);

        // waiting for rendering & physics calculations
        pthread_join(tid_vel, NULL);
        pthread_join(tid_render, NULL);
        update_positions(ctx);

        // wait for previous frame file writing to finish before continuing
        if (frame > 0) {
            pthread_join(file_write_thread, NULL);
        }
    }
out:
    yylex_destroy(scanner);
    if (finput) fclose(finput);
    free_context(ctx);

    // Free both framebuffers (double-buffered)
    if (fb[0]) free_framebuffer_pt4(fb[0]);
    if (fb[1]) free_framebuffer_pt4(fb[1]);

    return 0;

}

void render_scene(struct framebuffer_pt4 *fb, const struct context *ctx) {
    double left_right_angle;
    double up_down_angle;
    int xmax = fb->width;
    int ymax = fb->height;
    if (xmax > ymax) {
        left_right_angle = M_PI / 3;
        up_down_angle    = left_right_angle / xmax * ymax;
    } else {
        up_down_angle    = M_PI / 3;
        left_right_angle = up_down_angle / ymax * xmax;
    }

    double left_right_start = - left_right_angle / 2.0;
    double left_right_step  =   left_right_angle  / (xmax - 1);
    double up_down_start    =   up_down_angle     /  2.0;
    double up_down_step     =   up_down_angle     / (ymax - 1);

    for (int x = 0; x < xmax; x++) {
        double xangle = -(left_right_start + left_right_step * x);
        for (int y = 0; y < ymax; y++) {
            double yangle = up_down_start - up_down_step * y;
            pt3 direction = {{sin(xangle), sin(yangle), cos(yangle)*cos(xangle)}};
            pt3_normalize_mut(&direction);
            ray r = {{{0,0,-20}}, direction};
            pt4 px_color = {0};
            raytrace(ctx, &r, &px_color, 3);
            framebuffer_pt4_set(fb, x, y, px_color);
        }
    }
}