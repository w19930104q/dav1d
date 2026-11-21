/*
 * Copyright © 2018, VideoLAN and dav1d authors
 * Copyright © 2018, Two Orioles, LLC
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

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "input/demuxer.h"
#include "input/parse.h"

typedef struct DemuxerPriv {
    FILE *f;
    int broken;
    double timebase;
    uint64_t last_ts;
    uint64_t step;
    /* file offset of the 12-byte header of each packet */
    int64_t *pkt_offsets;
    unsigned num_pkts;
} IvfInputContext;

static const uint8_t probe_data[] = {
    'D', 'K', 'I', 'F',
    0, 0, 0x20, 0,
    'A', 'V', '0', '1',
};

static int ivf_probe(const uint8_t *const data, const int size) {
    return size >= (int) sizeof(probe_data) &&
           !memcmp(data, probe_data, sizeof(probe_data));
}

static unsigned rl32(const uint8_t *const p) {
    return ((uint32_t)p[3] << 24U) | (p[2] << 16U) | (p[1] << 8U) | p[0];
}

static int64_t rl64(const uint8_t *const p) {
    return (((uint64_t) rl32(&p[4])) << 32) | rl32(p);
}

static int ivf_open(IvfInputContext *const c, const char *const file,
                    unsigned fps[2], unsigned *const num_frames, unsigned timebase[2])
{
    uint8_t hdr[32];

    if (!(c->f = fopen(file, "rb"))) {
        fprintf(stderr, "Failed to open %s: %s\n", file, strerror(errno));
        return -1;
    } else if (fread(hdr, 32, 1, c->f) != 1) {
        fprintf(stderr, "Failed to read stream header: %s\n", strerror(errno));
        fclose(c->f);
        return -1;
    } else if (memcmp(hdr, "DKIF", 4)) {
        fprintf(stderr, "%s is not an IVF file [tag=%.4s|0x%02x%02x%02x%02x]\n",
                file, hdr, hdr[0], hdr[1], hdr[2], hdr[3]);
        fclose(c->f);
        return -1;
    } else if (memcmp(&hdr[8], "AV01", 4)) {
        fprintf(stderr, "%s is not an AV1 file [tag=%.4s|0x%02x%02x%02x%02x]\n",
                file, &hdr[8], hdr[8], hdr[9], hdr[10], hdr[11]);
        fclose(c->f);
        return -1;
    }

    timebase[0] = rl32(&hdr[16]);
    timebase[1] = rl32(&hdr[20]);
    const unsigned duration = rl32(&hdr[24]);

    uint8_t data[8];
    c->broken = 0;
    c->pkt_offsets = NULL;
    c->num_pkts = 0;
    unsigned pkt_cap = 0;
    for (*num_frames = 0;; (*num_frames)++) {
        const int64_t off = ftello(c->f);
        if (off < 0) goto fail;
        if (fread(data, 4, 1, c->f) != 1) break; // EOF
        size_t sz = rl32(data);
        if (fread(data, 8, 1, c->f) != 1) break; // EOF
        const uint64_t ts = rl64(data);
        if (*num_frames && ts <= c->last_ts)
            c->broken = 1;
        c->last_ts = ts;
        if (c->num_pkts == pkt_cap) {
            pkt_cap = pkt_cap ? pkt_cap * 2 : 64;
            int64_t *const new_offsets =
                realloc(c->pkt_offsets, pkt_cap * sizeof(*new_offsets));
            if (!new_offsets) goto fail;
            c->pkt_offsets = new_offsets;
        }
        c->pkt_offsets[c->num_pkts++] = off;
        fseeko(c->f, sz, SEEK_CUR);
    }

    if (*num_frames == 0) { /* Reading bailed early */
        fprintf(stderr, "No frames read from %s\n", file);
        free(c->pkt_offsets);
        fclose(c->f);
        return -1;
    }

    uint64_t fps_num = (uint64_t) timebase[0] * *num_frames;
    uint64_t fps_den = (uint64_t) timebase[1] * duration;
    if (fps_num && fps_den) { /* Reduce fraction */
        uint64_t gcd = fps_num;
        for (uint64_t a = fps_den, b; (b = a % gcd); a = gcd, gcd = b);
        fps_num /= gcd;
        fps_den /= gcd;

        while ((fps_num | fps_den) > UINT_MAX) {
            fps_num >>= 1;
            fps_den >>= 1;
        }
    }
    if (fps_num && fps_den) {
        fps[0] = (unsigned) fps_num;
        fps[1] = (unsigned) fps_den;
    } else {
        fps[0] = fps[1] = 0;
    }
    c->timebase = (double)timebase[0] / timebase[1];
    c->step = duration / *num_frames;

    fseeko(c->f, 32, SEEK_SET);
    c->last_ts = 0;

    return 0;
fail:
    fprintf(stderr, "Failed to index %s: %s\n", file, strerror(errno));
    free(c->pkt_offsets);
    fclose(c->f);
    return -1;
}

static inline int ivf_read_header(IvfInputContext *const c, ptrdiff_t *const sz,
                                  int64_t *const off_, uint64_t *const ts)
{
    uint8_t data[8];
    int64_t const off = ftello(c->f);
    if (off_) *off_ = off;
    if (fread(data, 4, 1, c->f) != 1) return -1; // EOF
    *sz = rl32(data);
    if (!c->broken) {
        if (fread(data, 8, 1, c->f) != 1) return -1;
        *ts = rl64(data);
    } else {
        if (fseeko(c->f, 8, SEEK_CUR)) return -1;
        *ts = off > 32 ? c->last_ts + c->step : 0;
    }
    return 0;
}

static unsigned get_bits(const uint8_t *const buf, size_t *const bitpos,
                         const int n)
{
    unsigned v = 0;
    for (int i = 0; i < n; i++) {
        const size_t bp = (*bitpos)++;
        v = (v << 1) | ((buf[bp >> 3] >> (7 - (bp & 7))) & 1);
    }
    return v;
}

/*
 * Minimal classification of an AV1 temporal unit (one IVF packet).
 * Sets *is_keyframe when the packet is a random access point: it carries a
 * sequence header OBU and a shown KEY frame (or a reduced still picture
 * header, which implies one). Returns the number of presented (show_frame or
 * show_existing_frame) frames in the packet; a packet can hold more than one
 * frame OBU (e.g. an invisible frame followed by a show_existing_frame one),
 * and invisible frames such as alt-refs are not counted.
 */
static int classify_packet(const uint8_t *const buf, const size_t buf_sz,
                           int *const is_keyframe)
{
    const uint8_t *ptr = buf;
    int left = (int) buf_sz;
    int reduced_still = 0;
    int has_seq_hdr = 0;
    int has_shown_key = 0;
    int shown = 0;
    *is_keyframe = 0;
    while (left > 0) {
        size_t obu_size;
        enum Dav1dObuType type;
        const int ret = parse_obu_header(ptr, left, &obu_size, &type, 0);
        if (ret < 0 || (size_t) ret > (size_t) left) break;
        const uint8_t *const payload = ptr + ret - obu_size;
        ptr += ret;
        left -= ret;
        if (type == DAV1D_OBU_SEQ_HDR && obu_size >= 1) {
            has_seq_hdr = 1;
            // seq profile (3 bits), still_picture (1), reduced_still... (1)
            size_t bitpos = 0;
            get_bits(payload, &bitpos, 3);
            get_bits(payload, &bitpos, 1);
            reduced_still = !!get_bits(payload, &bitpos, 1);
        } else if ((type == DAV1D_OBU_FRAME_HDR ||
                    type == DAV1D_OBU_FRAME) && obu_size)
        {
            size_t bitpos = 0;
            if (reduced_still) {
                // reduced still picture headers imply a shown key frame and
                // have no show_existing_frame/frame_type/show_frame bits
                shown++;
                has_shown_key = 1;
            } else if (get_bits(payload, &bitpos, 1)) {
                // show_existing_frame always presents a frame, but by itself
                // it does not make the packet a random access point
                shown++;
            } else {
                const unsigned frame_type =
                    get_bits(payload, &bitpos, 2);
                const unsigned show_frame =
                    get_bits(payload, &bitpos, 1);
                if (show_frame) {
                    shown++;
                    if (frame_type == DAV1D_FRAME_TYPE_KEY)
                        has_shown_key = 1;
                }
            }
        }
    }
    *is_keyframe = has_seq_hdr && (has_shown_key || reduced_still);
    return shown;
}

static int ivf_read(IvfInputContext *const c, Dav1dData *const buf) {
    uint8_t *ptr;
    ptrdiff_t sz;
    int64_t off;
    uint64_t ts;
    if (ivf_read_header(c, &sz, &off, &ts)) return -1;
    if (!(ptr = dav1d_data_create(buf, sz))) return -1;
    if (fread(ptr, sz, 1, c->f) != 1) {
        fprintf(stderr, "Failed to read frame data: %s\n", strerror(errno));
        dav1d_data_unref(buf);
        return -1;
    }
    buf->m.offset = off;
    buf->m.timestamp = ts;
    c->last_ts = ts;
    return 0;
}

static int ivf_seek(IvfInputContext *const c, const uint64_t pts) {
    uint64_t cur;
    const uint64_t ts = llround((pts * c->timebase) / 1000000000.0);
    if (ts <= c->last_ts)
        if (fseeko(c->f, 32, SEEK_SET)) goto error;
    while (1) {
        ptrdiff_t sz;
        if (ivf_read_header(c, &sz, NULL, &cur)) goto error;
        if (cur >= ts) break;
        if (fseeko(c->f, sz, SEEK_CUR)) goto error;
        c->last_ts = cur;
    }
    if (fseeko(c->f, -12, SEEK_CUR)) goto error;
    return 0;
error:
    fprintf(stderr, "Failed to seek: %s\n", strerror(errno));
    return -1;
}

static void ivf_close(IvfInputContext *const c) {
    free(c->pkt_offsets);
    fclose(c->f);
}

/* Read the payload of packet pkt into a freshly allocated buffer. */
static int read_packet(IvfInputContext *const c, const unsigned pkt,
                       uint8_t **const data_out, size_t *const size_out)
{
    uint8_t hdr[12];
    if (fseeko(c->f, c->pkt_offsets[pkt], SEEK_SET)) return -1;
    if (fread(hdr, 12, 1, c->f) != 1) return -1;
    const size_t sz = rl32(hdr);
    uint8_t *const buf = malloc(sz ? sz : 1);
    if (!buf) return -1;
    if (fread(buf, sz, 1, c->f) != 1) {
        free(buf);
        return -1;
    }
    *data_out = buf;
    *size_out = sz;
    return 0;
}

static int ivf_seek_frame(IvfInputContext *const c, const unsigned frame_num,
                          unsigned *const seek_pkt,
                          unsigned *const frames_before,
                          unsigned *const total_showable)
{
    /*
     * Frame numbers count presented frames in display order. Scan every
     * packet once to record the number of frames each one presents and which
     * packets are random access points (carry a sequence header OBU). A
     * packet may present zero (invisible alt-ref) or more than one
     * (show_existing_frame) frame.
     */
    uint8_t *const key_flags = malloc(c->num_pkts);
    uint16_t *const shown_per_pkt = malloc(c->num_pkts *
                                           sizeof(*shown_per_pkt));
    if (!key_flags || !shown_per_pkt) {
        free(key_flags);
        free(shown_per_pkt);
        goto error;
    }
    unsigned total_showable_frames = 0;
    for (unsigned pkt = 0; pkt < c->num_pkts; pkt++) {
        uint8_t *buf;
        size_t sz;
        if (read_packet(c, pkt, &buf, &sz)) {
            free(key_flags);
            free(shown_per_pkt);
            goto error;
        }
        int is_keyframe;
        const int shown = classify_packet(buf, sz, &is_keyframe);
        free(buf);
        key_flags[pkt] = (uint8_t) is_keyframe;
        shown_per_pkt[pkt] = (uint16_t) shown;
        total_showable_frames += shown;
    }
    if (frame_num >= total_showable_frames) {
        free(key_flags);
        free(shown_per_pkt);
        fprintf(stderr, "Seek frame %u is beyond the end of the file "
                        "(%u showable frames)\n", frame_num,
                total_showable_frames);
        return -1;
    }

    /* Locate the packet containing the frame_num-th presented frame. */
    unsigned target_pkt = 0, shown = 0;
    for (unsigned pkt = 0; pkt < c->num_pkts; pkt++) {
        if (shown + shown_per_pkt[pkt] > frame_num) {
            target_pkt = pkt;
            break;
        }
        shown += shown_per_pkt[pkt];
    }

    /* Walk back to the closest preceding random access point. */
    unsigned key_pkt = 0;
    for (unsigned pkt = target_pkt + 1; pkt-- > 0;) {
        if (key_flags[pkt]) {
            key_pkt = pkt;
            break;
        }
    }

    /* Number of presented frames preceding the random access point. */
    unsigned before = 0;
    for (unsigned pkt = 0; pkt < key_pkt; pkt++)
        before += shown_per_pkt[pkt];
    free(key_flags);
    free(shown_per_pkt);

    if (fseeko(c->f, c->pkt_offsets[key_pkt], SEEK_SET)) goto error;
    *seek_pkt = key_pkt;
    *frames_before = before;
    *total_showable = total_showable_frames;
    return 0;
error:
    fprintf(stderr, "Failed to seek: %s\n", strerror(errno));
    return -1;
}

const Demuxer ivf_demuxer = {
    .priv_data_size = sizeof(IvfInputContext),
    .name = "ivf",
    .probe = ivf_probe,
    .probe_sz = sizeof(probe_data),
    .open = ivf_open,
    .read = ivf_read,
    .seek = ivf_seek,
    .seek_frame = ivf_seek_frame,
    .close = ivf_close,
};
