#!/bin/sh
# Restore the exact boot artifacts and module tree saved by
# install-vim3-kernel-replace.sh.  Run on the VIM3's root filesystem (or from
# recovery after mounting it at / and chrooting into it).
set -eu

RELEASE=6.18.44-current-meson64

die() { echo "error: $*" >&2; exit 1; }

[ "$(id -u)" -eq 0 ] || die "run as root"
if [ "${1:-}" != "--restore" ] || [ "$#" -ne 2 ]; then
	echo "Usage: sudo $0 --restore /boot/birdcher-backup-$RELEASE-<timestamp>" >&2
	exit 2
fi

backup=$2
[ -d "$backup/boot" ] || die "missing backup boot directory"
[ -d "$backup/modules/$RELEASE" ] || die "missing backup module tree"

echo "Restoring module tree"
rm -rf "/lib/modules/$RELEASE"
cp -a "$backup/modules/$RELEASE" /lib/modules/
depmod -a "$RELEASE"

echo "Restoring boot artifacts"
for f in "vmlinuz-$RELEASE" "initrd.img-$RELEASE" "uInitrd-$RELEASE" \
	 "System.map-$RELEASE" "config-$RELEASE"; do
	[ -e "$backup/boot/$f" ] || die "backup lacks $f"
	cp -a "$backup/boot/$f" "/boot/$f"
done
for f in Image uInitrd; do
	[ -e "$backup/boot/$f" ] || [ -L "$backup/boot/$f" ] || die "backup lacks $f"
	rm -f "/boot/$f"
	cp -a "$backup/boot/$f" "/boot/$f"
done
sync

echo "Restore complete. Reboot when ready."
