cat arch/arm/boot/zImage arch/arm/boot/dts/exynos4210-i9100.dtb > zimage
mkimage -A arm -O linux -T kernel -C none -a 0x40008000 -e 0x40008000 -n Linux-android-sucks -d zimage boot_safe.img
