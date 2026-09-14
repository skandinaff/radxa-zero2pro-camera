# VIM3 replacement-kernel bundle

This bundle is for the Khadas VIM3 currently running
`6.18.44-current-meson64`.  It contains Linux `v6.18.44` (source commit
`1efe5d048a391de3ead2804b2e7f86376c356cc5`) built with the VIM3's exported
configuration and `CONFIG_VIDEO_IMX415=m`.

The release string deliberately remains `6.18.44-current-meson64`.  The VIM3
boot script always loads `/boot/Image` and `/boot/uInitrd`, so this is a
**replacement deployment**, not a boot-menu entry.  It backs up the current
kernel artifacts and the complete module tree before replacing them.  Keep
UART/recovery access available.

The bundle does not replace `/boot/dtb-*` or `armbianEnv.txt`.  VIM3 camera
nodes are supplied later as runtime overlays from `camera-stack/`.

## Install

Copy the whole unpacked directory to the VIM3 and inspect it first:

```sh
cd vim3-6.18.44-current-meson64-replacement
sha256sum -c SHA256SUMS
sudo ./install-vim3-kernel-replace.sh
```

The last command is intentionally a dry run.  With UART/recovery available,
perform the replacement:

```sh
sudo ./install-vim3-kernel-replace.sh --install
sudo reboot
```

The installer prints the exact `/boot/birdcher-backup-*` directory.  From a
booted recovery system (mount and chroot into the VIM3 root filesystem if
needed), restore it with:

```sh
sudo ./restore-vim3-kernel-replace.sh --restore /boot/birdcher-backup-6.18.44-current-meson64-<timestamp>
```

## Post-boot checks

Confirm the expected release and IMX415 module are present before camera
bring-up:

```sh
uname -r
modinfo imx415 | head
find /lib/modules/$(uname -r) -name imx415.ko
```

`camera-stack/` contains the separately-built VIM3 runtime modules and DT
overlays.  Do not run its `load.sh full` until the IMX415's actual I2C address
and the board's PWDN/reset sequencing have been verified; the kernel
replacement itself neither probes nor powers the camera.
