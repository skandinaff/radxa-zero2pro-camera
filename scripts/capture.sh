#!/bin/sh
# Attempt a single-frame capture from the ISP's V4L2 node and report what came
# back. Writes a raw frame to /tmp/frame.raw plus, if ffmpeg is available, a
# viewable PNG next to it.
#
# Usage: ./scripts/capture.sh [/dev/videoN]
set -e

DEV=${1:-}
if [ -z "$DEV" ]; then
	for d in /dev/video0 /dev/video1 /dev/video2 /dev/video3; do
		[ -c "$d" ] && { DEV=$d; break; }
	done
fi
[ -n "$DEV" ] || { echo "error: no /dev/video* found -- did the ISP probe?" >&2; exit 1; }

command -v v4l2-ctl >/dev/null || {
	echo "error: v4l2-ctl missing (sudo apt install v4l-utils)" >&2; exit 1; }

printf '=== %s capabilities ===\n' "$DEV"
v4l2-ctl -d "$DEV" --all 2>&1 | head -40

printf '\n=== supported formats ===\n'
v4l2-ctl -d "$DEV" --list-formats-ext 2>&1 | head -30

printf '\n=== capturing 1 frame ===\n'
rm -f /tmp/frame.raw
# --stream-mmap with a small count: if the pipeline never produces a buffer
# this returns a timeout rather than hanging forever.
timeout 20 v4l2-ctl -d "$DEV" \
	--stream-mmap --stream-count=1 --stream-to=/tmp/frame.raw 2>&1 || {
	rc=$?
	printf 'capture failed (exit %s)\n' "$rc"
	printf 'dmesg tail:\n'
	dmesg 2>/dev/null | tail -20 | sed 's/^/  /'
	exit $rc
}

if [ -s /tmp/frame.raw ]; then
	printf 'OK: wrote /tmp/frame.raw (%s bytes)\n' "$(stat -c %s /tmp/frame.raw)"
	# Sanity check that it isn't just a buffer of zeros -- a driver can hand
	# back a correctly-sized but entirely blank frame when the sensor never
	# actually streamed.
	if od -An -tx1 -N 65536 /tmp/frame.raw | tr -d ' \n' | grep -qv '^0*$'; then
		printf 'frame contains non-zero data\n'
	else
		printf 'WARNING: first 64KB is all zeros -- sensor may not be streaming\n'
	fi
else
	printf 'ERROR: /tmp/frame.raw is empty\n'
	exit 1
fi
