#!/usr/bin/env python3
"""Regression test for the dav1d CLI --seek option.

It verifies, on small checked-in IVF fixtures (one with invisible alt-ref
frames, one without), that:

  1. `dav1d --seek N` outputs exactly the frames a full decode outputs from
     visible frame N onward, with identical pixel content (compared through
     per-frame raw YUV files and their md5 hashes).
  2. --verify accepts the md5 of the seek run and rejects a wrong one.
  3. per-frame output files keep their full-decode absolute frame numbers.
  4. --seek + --limit decodes the requested frame window.
  5. out-of-range seeks, non-IVF inputs, --skip, --outputinvisible 1 and
     --decodeframetype key/intra are rejected with a non-zero exit code.
  6. plain decoding without --seek is unaffected (full run is the reference).

The test only needs python3 plus the dav1d binary; no encoder is required.
Run with -I. Usage: seek_test.py /path/to/dav1d [fixture_dir]
"""
import hashlib
import os
import shutil
import struct
import subprocess
import sys
import tempfile


def md5_bytes(data):
    return hashlib.md5(data).hexdigest()


class Dav1d:
    def __init__(self, binary):
        self.binary = binary

    def run(self, args, expect_rc=0):
        proc = subprocess.run([self.binary, "-q"] + args,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if proc.returncode != expect_rc:
            raise AssertionError(
                f"dav1d {' '.join(args)} exited {proc.returncode}, "
                f"expected {expect_rc}\nstderr: {proc.stderr.decode(errors='replace')}")
        return proc

    def run_fail(self, args, needle):
        proc = subprocess.run([self.binary, "-q"] + args,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if proc.returncode == 0:
            raise AssertionError(f"dav1d {' '.join(args)} unexpectedly succeeded")
        err = proc.stderr.decode(errors="replace")
        if needle.lower() not in err.lower():
            raise AssertionError(
                f"dav1d {' '.join(args)} error missing '{needle}': {err}")


def count_ivf_packets(path):
    with open(path, "rb") as f:
        data = f.read()
    assert data[:4] == b"DKIF"
    return struct.unpack_from("<I", data, 24)[0]


def ivf_to_section5(path, out):
    """Raw low-overhead OBU stream: IVF packet payloads concatenated."""
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack_from("<I", data, 24)[0]
    pos = 32
    with open(out, "wb") as o:
        for _ in range(n):
            size = struct.unpack_from("<I", data, pos)[0]
            pos += 12
            o.write(data[pos:pos + size])
            pos += size


def main():
    if len(sys.argv) < 2:
        print("usage: seek_test.py <dav1d-binary> [fixture-dir]", file=sys.stderr)
        return 2
    binary = sys.argv[1]
    here = os.path.dirname(os.path.abspath(__file__))
    fixture_dir = sys.argv[2] if len(sys.argv) > 2 else here
    fixtures = [os.path.join(fixture_dir, n) for n in ("g5.ivf", "g10.ivf")]
    for fx in fixtures:
        if not os.path.isfile(fx):
            print(f"SKIP: missing fixture {fx}", file=sys.stderr)
            return 0

    d = Dav1d(binary)
    work = tempfile.mkdtemp(prefix="dav1d-seek-test-")
    try:
        for ivf in fixtures:
            name = os.path.basename(ivf)
            print(f"-- {name}")
            base = os.path.join(work, name)
            os.makedirs(base)

            # full decode reference: per-frame raw YUV, absolute numbering
            full_dir = os.path.join(base, "full")
            os.makedirs(full_dir)
            d.run(["-i", ivf, "--filmgrain", "0", "--muxer", "frameyuv",
                   "-o", os.path.join(full_dir, "f_%03n.yuv")])
            full_frames = sorted(os.listdir(full_dir))
            nfull = len(full_frames)
            assert nfull >= 4, "fixture too small"

            # whole-stream md5 of a normal (non-seek) run, to prove the
            # default code path is unchanged
            plain = d.run(["-i", ivf, "--filmgrain", "0",
                           "--muxer", "md5", "-o", "-"]).stdout.strip()
            full_cat = b"".join(
                open(os.path.join(full_dir, f), "rb").read()
                for f in full_frames)
            assert plain.decode() == md5_bytes(full_cat), \
                "plain md5 muxer disagrees with raw output"

            points = [0, 1, nfull // 2, nfull - 2, nfull - 1]
            for n in points:
                # reference: md5 of the raw full-decode tail from frame n
                ref = md5_bytes(b"".join(
                    open(os.path.join(full_dir, f), "rb").read()
                    for f in full_frames[n:]))

                got = d.run(["-i", ivf, "--filmgrain", "0", "--seek", str(n),
                             "--muxer", "md5", "-o", "-"]).stdout.strip()
                assert got.decode() == ref, \
                    f"seek {n}: md5 {got.decode()} != full tail {ref}"

                # --verify: matching hash accepted, wrong hash rejected
                d.run(["-i", ivf, "--filmgrain", "0", "--seek", str(n),
                       "--verify", got.decode()])
                d.run_fail(["-i", ivf, "--filmgrain", "0", "--seek", str(n),
                            "--verify", "0" * 32], "")

                # per-frame outputs: correct count and absolute numbers
                sdir = os.path.join(base, f"seek_{n}")
                os.makedirs(sdir)
                d.run(["-i", ivf, "--filmgrain", "0", "--seek", str(n),
                       "--muxer", "frameyuv",
                       "-o", os.path.join(sdir, "s_%03n.yuv")])
                sframes = sorted(os.listdir(sdir))
                assert len(sframes) == nfull - n, \
                    f"seek {n}: {len(sframes)} frames, expected {nfull - n}"
                assert sframes[0] == f"s_{n:03d}.yuv", \
                    f"seek {n}: first file {sframes[0]}"
                for idx, sf in enumerate(sframes):
                    ref_frame = full_frames[n + idx]
                    a = open(os.path.join(sdir, sf), "rb").read()
                    b = open(os.path.join(full_dir, ref_frame), "rb").read()
                    assert a == b, \
                        f"seek {n}: pixel mismatch on {ref_frame}"

            # --seek + --limit window
            n = nfull // 3
            lim = 3
            ldir = os.path.join(base, "limit")
            os.makedirs(ldir)
            d.run(["-i", ivf, "--filmgrain", "0", "--seek", str(n),
                   "--limit", str(lim), "--muxer", "frameyuv",
                   "-o", os.path.join(ldir, "l_%03n.yuv")])
            lframes = sorted(os.listdir(ldir))
            assert len(lframes) == lim, "seek+limit frame count"
            for idx, lf in enumerate(lframes):
                assert lf == f"l_{n + idx:03d}.yuv", "seek+limit numbering"
                a = open(os.path.join(ldir, lf), "rb").read()
                b = open(os.path.join(full_dir, full_frames[n + idx]), "rb").read()
                assert a == b, "seek+limit pixel mismatch"

            # limit beyond EOF just runs to the end
            d.run(["-i", ivf, "--filmgrain", "0", "--seek", str(nfull - 2),
                   "--limit", "1000", "--muxer", "null"])

            # out of range seek fails clearly
            d.run_fail(["-i", ivf, "--seek", str(nfull + 50),
                        "--muxer", "null"], "beyond the end")
            d.run_fail(["-i", ivf, "--seek", "-1", "--muxer", "null"],
                       "non-negative integer")
            d.run_fail(["-i", ivf, "--seek", "abc", "--muxer", "null"],
                       "non-negative integer")

            # option conflicts
            d.run_fail(["-i", ivf, "--seek", "1", "--skip", "1",
                        "--muxer", "null"], "mutually exclusive")
            d.run_fail(["-i", ivf, "--seek", "1", "--outputinvisible", "1",
                        "--muxer", "null"], "outputinvisible")
            d.run_fail(["-i", ivf, "--seek", "1", "--decodeframetype", "key",
                        "--muxer", "null"], "decodeframetype")
            # compatible decode frame type still works
            d.run(["-i", ivf, "--seek", "1", "--decodeframetype",
                   "reference", "--muxer", "null"])

            # non-IVF input is rejected
            s5 = os.path.join(base, "stream.section5")
            ivf_to_section5(ivf, s5)
            d.run(["-i", s5, "--muxer", "null"])  # decodes fine without seek
            d.run_fail(["-i", s5, "--seek", "0", "--muxer", "null"],
                       "only supported for IVF")

        print("ALL SEEK TESTS PASSED")
        return 0
    finally:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
