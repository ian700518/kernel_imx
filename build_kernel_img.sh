export PATH=~/Lingshi/myandroid_3d_learn/bootable/bootloader/uboot-imx/tools:$PATH
export ARCH=arm
export CROSS_COMPILE=~/Lingshi/myandroid_3d_learn/prebuilts/gcc/linux-x86/arm/arm-eabi-4.6/bin/arm-eabi-
make imx_v7_android_defconfig
make uImage LOADADDR=0x10008000
