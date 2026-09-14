#!/bin/sh
# Install a matching VIM3 test kernel bundle that deliberately reuses the
# current kernel release.  This is a replacement deployment, not a dual-boot
# install: the caller must retain UART/recovery access.
set -eu

RELEASE=6.18.44-current-meson64
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)

die() { echo "error: $*" >&2; exit 1; }

[ "$(id -u)" -eq 0 ] || die "run as root"

if [ "${1:-}" != "--install" ] || [ "$#" -ne 1 ]; then
	cat <<EOF
This replaces the active $RELEASE kernel and its complete module tree.
It first saves /boot artifacts and /lib/modules/$RELEASE under /boot.

Usage: sudo $0 --install
EOF
	exit 2
fi

[ "$(uname -r)" = "$RELEASE" ] || die "running kernel is $(uname -r), expected $RELEASE"
for f in "$ROOT/kernel/Image" "$ROOT/kernel/System.map" "$ROOT/kernel/config" \
	 "$ROOT/modules.tar.gz"; do
	[ -f "$f" ] || die "bundle is incomplete: $f"
done

timestamp=$(date -u +%Y%m%dT%H%M%SZ)
backup="/boot/birdcher-backup-$RELEASE-$timestamp"
mkdir -p "$backup/boot" "$backup/modules"

echo "Backing up current boot artifacts to $backup"
for f in Image uInitrd "vmlinuz-$RELEASE" "initrd.img-$RELEASE" \
	 "uInitrd-$RELEASE" "System.map-$RELEASE" "config-$RELEASE"; do
	[ -e "/boot/$f" ] || [ -L "/boot/$f" ] || die "missing /boot/$f"
	cp -a "/boot/$f" "$backup/boot/"
done
[ -d "/lib/modules/$RELEASE" ] || die "missing /lib/modules/$RELEASE"
cp -a "/lib/modules/$RELEASE" "$backup/modules/"
sync

echo "Installing replacement module tree"
rm -rf "/lib/modules/$RELEASE"
tar -xzf "$ROOT/modules.tar.gz" -C /lib/modules
rm -f "/lib/modules/$RELEASE/build" "/lib/modules/$RELEASE/source"
depmod -a "$RELEASE"

echo "Installing kernel image and regenerating initramfs"
install -m 0644 "$ROOT/kernel/Image" "/boot/vmlinuz-$RELEASE"
install -m 0644 "$ROOT/kernel/System.map" "/boot/System.map-$RELEASE"
install -m 0644 "$ROOT/kernel/config" "/boot/config-$RELEASE"
update-initramfs -u -k "$RELEASE"
mkimage -A arm64 -O linux -T ramdisk -C none \
	-d "/boot/initrd.img-$RELEASE" "/boot/uInitrd-$RELEASE"
ln -sfn "vmlinuz-$RELEASE" /boot/Image
ln -sfn "uInitrd-$RELEASE" /boot/uInitrd
sync

echo "Installed. Backup: $backup"
echo "Reboot only with UART/recovery available."
