/*
 * Copyright © 2020, VideoLAN and dav1d authors
 * Copyright © 2020, Two Orioles, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"
#include "vcs_version.h"
#include "cli_config.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dav1d/dav1d.h"
#include "input/input.h"
#include "input/demuxer.h"
#include "dav1d_cli_parse.h"

#define NUM_RAND_SEEK 3
#define NUM_REL_SEEK  4
#define NUM_END_SEEK  2

/* distinct exit code so CI can tell a hit time limit apart from a
 * clean run; note that seek/decode errors on individual packets are
 * tolerated (exit 0), since randomly seeking into a stream can
 * legitimately produce undecodable sequences */
#define EXIT_TIMEOUT 2

const Demuxer annexb_demuxer = { .name = "" };
const Demuxer section5_demuxer = { .name = "" };

#ifdef _WIN32
#include <windows.h>
static unsigned get_seed(void) {
    return GetTickCount();
}
#else
#include <time.h>
#ifdef __APPLE__
#include <mach/mach_time.h>
#endif
static unsigned get_seed(void) {
#ifdef __APPLE__
    return (unsigned) mach_absolute_time();
#elif HAVE_CLOCK_GETTIME
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (unsigned) (1000000000ULL * ts.tv_sec + ts.tv_nsec);
#endif
}
#endif

static double monotonic_sec(void) {
#ifdef _WIN32
    return GetTickCount64() / 1000.0;
#elif HAVE_CLOCK_GETTIME
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
#else
    return (double) time(NULL);
#endif
}

/* wall-clock time at which the run must stop; 0 means no limit */
static double deadline;

static int timed_out(void) {
    return deadline > 0 && monotonic_sec() >= deadline;
}

static uint32_t xs_state[4];

static void xor128_srand(unsigned seed) {
    xs_state[0] = seed;
    xs_state[1] = ( seed & 0xffff0000) | (~seed & 0x0000ffff);
    xs_state[2] = (~seed & 0xffff0000) | ( seed & 0x0000ffff);
    xs_state[3] = ~seed;
}

// xor128 from Marsaglia, George (July 2003). "Xorshift RNGs".
//             Journal of Statistical Software. 8 (14).
//             doi:10.18637/jss.v008.i14.
static int xor128_rand(void) {
    const uint32_t x = xs_state[0];
    const uint32_t t = x ^ (x << 11);

    xs_state[0] = xs_state[1];
    xs_state[1] = xs_state[2];
    xs_state[2] = xs_state[3];
    uint32_t w = xs_state[3];

    w = (w ^ (w >> 19)) ^ (t ^ (t >> 8));
    xs_state[3] = w;

    return w >> 1;
}

static inline int decode_frame(Dav1dPicture *const p,
                               Dav1dContext *const c, Dav1dData *const data)
{
    int res;
    memset(p, 0, sizeof(*p));
    if ((res = dav1d_send_data(c, data)) < 0) {
        if (res != DAV1D_ERR(EAGAIN)) {
            fprintf(stderr, "Error decoding frame: %s\n",
                    strerror(DAV1D_ERR(res)));
            return res;
        }
    }
    if ((res = dav1d_get_picture(c, p)) < 0) {
        if (res != DAV1D_ERR(EAGAIN)) {
            fprintf(stderr, "Error decoding frame: %s\n",
                    strerror(DAV1D_ERR(res)));
            return res;
        }
    } else dav1d_picture_unref(p);
    return 0;
}

static int decode_rand(DemuxerContext *const in, Dav1dContext *const c,
                       Dav1dData *const data, const double fps)
{
    int res = 0;
    Dav1dPicture p;
    const int num_frames = xor128_rand() % (int)(fps * 5);
    for (int i = 0; i < num_frames && !timed_out(); i++) {
        if ((res = decode_frame(&p, c, data))) break;
        if (!data->sz && (input_read(in, data) || !data->sz)) break;
    }
    return res;
}

static int decode_all(DemuxerContext *const in,
                      Dav1dContext *const c, Dav1dData *const data)
{
    int res = 0;
    Dav1dPicture p;
    do {
        if ((res = decode_frame(&p, c, data))) break;
    } while (!timed_out() &&
             (data->sz > 0 || (!input_read(in, data) && data->sz > 0)));
    return res;
}

static int seek(DemuxerContext *const in, Dav1dContext *const c,
                const uint64_t pts, Dav1dData *const data)
{
    int res;
    if ((res = input_seek(in, pts))) return res;
    Dav1dSequenceHeader seq;
    do {
        dav1d_data_unref(data);
        if ((res = input_read(in, data))) break;
    } while (dav1d_parse_sequence_header(&seq, data->data, data->sz));
    dav1d_flush(c);
    return res;
}

static void usage(const char *const app) {
    fprintf(stderr,
            "Usage: %s [options] -i <input.ivf>\n"
            "In addition to the common dav1d options, seek_stress supports:\n"
            "  -n, --num-rounds <n>   number of seek stress rounds to run;\n"
            "                         0 means unlimited (default: 1)\n"
            "  -t, --timeout <sec>    wall-clock time limit in seconds;\n"
            "                         0 means no limit (default: 0)\n"
            "      --seed <seed>      seed for the random seek pattern;\n"
            "                         defaults to a time-based value\n"
            "Exit status: 0 if the run completed (seek/decode errors on\n"
            "             individual packets are tolerated and only logged),\n"
            "             1 on a usage or initialization error,\n"
            "             %d if the time limit was reached\n",
            app, EXIT_TIMEOUT);
}

static int parse_uint_option(const char *const opt, const char *const arg,
                             unsigned *const out)
{
    char *end;
    errno = 0;
    const unsigned long v = strtoul(arg, &end, 10);
    if (errno || !*arg || *end || v > UINT_MAX) {
        fprintf(stderr, "seek_stress: invalid value '%s' for %s\n", arg, opt);
        return -1;
    }
    *out = (unsigned) v;
    return 0;
}

/* Extract seek_stress-specific options from argv before handing the
 * remaining arguments to the common dav1d option parser. Returns -1
 * (after printing an error) on invalid input. */
static int parse_seek_stress_options(int *const argc, char **const argv,
                                     unsigned *const num_rounds,
                                     unsigned *const timeout,
                                     unsigned *const seed, int *const have_seed)
{
    int dst = 1;
    for (int i = 1; i < *argc; i++) {
        const char *const arg = argv[i];
        const char *val = NULL;
        unsigned *target = NULL;

        if (!strcmp(arg, "-n") || !strcmp(arg, "--num-rounds"))
            target = num_rounds;
        else if (!strncmp(arg, "--num-rounds=", 13))
            target = num_rounds, val = arg + 13;
        else if (arg[0] == '-' && arg[1] == 'n' && arg[2])
            target = num_rounds, val = arg + 2;
        else if (!strcmp(arg, "-t") || !strcmp(arg, "--timeout"))
            target = timeout;
        else if (!strncmp(arg, "--timeout=", 10))
            target = timeout, val = arg + 10;
        else if (arg[0] == '-' && arg[1] == 't' && arg[2])
            target = timeout, val = arg + 2;
        else if (!strcmp(arg, "--seed"))
            target = seed, *have_seed = 1;
        else if (!strncmp(arg, "--seed=", 7))
            target = seed, *have_seed = 1, val = arg + 7;

        if (!target) {
            argv[dst++] = argv[i];
            continue;
        }
        if (!val) {
            if (++i >= *argc) {
                fprintf(stderr, "seek_stress: missing value for %s\n", arg);
                return -1;
            }
            val = argv[i];
        }
        if (parse_uint_option(arg, val, target))
            return -1;
    }
    argv[dst] = NULL;
    *argc = dst;
    return 0;
}

int main(int argc, char **const argv) {
    const char *version = dav1d_version();
    if (strcmp(version, DAV1D_VERSION)) {
        fprintf(stderr, "Version mismatch (library: %s, executable: %s)\n",
                version, DAV1D_VERSION);
        return EXIT_FAILURE;
    }

    CLISettings cli_settings;
    Dav1dSettings lib_settings;
    DemuxerContext *in;
    Dav1dContext *c;
    Dav1dData data = { 0 };
    unsigned total, i_fps[2], i_timebase[2];
    unsigned num_rounds = 1, timeout = 0, seed = 0;
    int have_seed = 0;
    double timebase, spf, fps;
    uint64_t pts;
    int ret = EXIT_SUCCESS;

    if (parse_seek_stress_options(&argc, argv, &num_rounds, &timeout,
                                  &seed, &have_seed))
    {
        usage(argv[0]);
        return EXIT_FAILURE;
    }
    if (!have_seed)
        seed = get_seed();
    xor128_srand(seed);

    parse(argc, argv, &cli_settings, &lib_settings);

    if (!cli_settings.quiet) {
        fprintf(stderr, "seek_stress: seed=%u", seed);
        if (num_rounds) fprintf(stderr, " rounds=%u", num_rounds);
        else            fprintf(stderr, " rounds=unlimited");
        if (timeout)    fprintf(stderr, " timeout=%us", timeout);
        fprintf(stderr, "\n");
    }

    if (input_open(&in, "ivf", cli_settings.inputfile,
                   i_fps, &total, i_timebase) < 0)
    {
        return EXIT_SUCCESS;
    }
    if (!i_timebase[0] || !i_timebase[1] || !i_fps[0] || !i_fps[1]) {
        input_close(in);
        return EXIT_SUCCESS;
    }
    if (dav1d_open(&c, &lib_settings)) {
        input_close(in);
        return EXIT_FAILURE;
    }

    timebase = (double)i_timebase[1] / i_timebase[0];
    spf = (double)i_fps[1] / i_fps[0];
    fps = (double)i_fps[0] / i_fps[1];
    if (fps < 1) goto end;

#define FRAME_OFFSET_TO_PTS(foff) \
    (uint64_t)llround(((foff) * spf) * 1000000000.0)
#define TS_TO_PTS(ts) \
    (uint64_t)llround(((ts) * timebase) * 1000000000.0)

    if (timeout)
        deadline = monotonic_sec() + timeout;

    for (unsigned round = 0; !num_rounds || round < num_rounds; round++) {
        if (timed_out()) goto timeout;

        // seek at random pts
        for (int i = 0; i < NUM_RAND_SEEK; i++) {
            if (timed_out()) goto timeout;
            pts = FRAME_OFFSET_TO_PTS(xor128_rand() % total);
            if (seek(in, c, pts, &data)) continue;
            if (decode_rand(in, c, &data, fps)) goto end;
        }
        pts = TS_TO_PTS(data.m.timestamp);

        // seek left / right randomly with random intervals within 1s
        for (int i = 0, tries = 0;
             i - tries < NUM_REL_SEEK && tries < NUM_REL_SEEK / 2;
             i++)
        {
            if (timed_out()) goto timeout;
            const int sign = xor128_rand() & 1 ? -1 : +1;
            const float diff = (xor128_rand() % 100) / 100.f;
            int64_t new_pts = pts + sign * FRAME_OFFSET_TO_PTS(diff * fps);
            const int64_t new_ts = llround(new_pts / (timebase * 1000000000.0));
            new_pts = TS_TO_PTS(new_ts);
            if (new_pts < 0 || (uint64_t)new_pts >= FRAME_OFFSET_TO_PTS(total)) {
                if (seek(in, c, FRAME_OFFSET_TO_PTS(total / 2), &data)) break;
                pts = TS_TO_PTS(data.m.timestamp);
                tries++;
                continue;
            }
            if (seek(in, c, new_pts, &data))
                if (seek(in, c, 0, &data)) goto end;
            if (decode_rand(in, c, &data, fps)) goto end;
            pts = TS_TO_PTS(data.m.timestamp);
        }

        unsigned shift = 0;
        do {
            shift += 5;
            if (shift > total)
                shift = total;
        } while (!timed_out() &&
                 seek(in, c, FRAME_OFFSET_TO_PTS(total - shift), &data) &&
                 shift < total);

        // simulate seeking after the end of the file
        for (int i = 0; i < NUM_END_SEEK; i++) {
            if (timed_out()) goto timeout;
            if (seek(in, c, FRAME_OFFSET_TO_PTS(total - shift), &data)) goto end;
            if (decode_all(in, c, &data)) goto end;
            int num_flush = 1 + 64 + xor128_rand() % 64;
            while (num_flush--) dav1d_flush(c);
        }
    }
    goto end;

timeout:
    if (!cli_settings.quiet)
        fprintf(stderr, "seek_stress: time limit of %u s reached, "
                "stopping before all rounds completed\n", timeout);
    ret = EXIT_TIMEOUT;
end:
    dav1d_data_unref(&data);
    input_close(in);
    dav1d_close(&c);
    return ret;
}
