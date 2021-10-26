#!/bin/bash

# Bash Color
green='\033[01;32m'
red='\033[01;31m'
blink_red='\033[05;31m'
restore='\033[0m'

clear

# Resources
THREAD="-j$(grep -c ^processor /proc/cpuinfo)"
KERNEL="Image.gz"
DTBOIMG="dtbo.img"

# Defconfigs
UMIDEFCONFIG="vendor/yarpiin_umi_defconfig"
CMIDEFCONFIG="vendor/yarpiin_cmi_defconfig"
CASDEFCONFIG="vendor/yarpiin_cas_defconfig"

# Build dirs
KERNEL_DIR="/home/yarpiin/Android/Kernel/Xiaomi/White-Wolf-UMI-UNI"
RESOURCE_DIR="$KERNEL_DIR/.."
KERNELFLASHER_DIR="/home/yarpiin/Android/Kernel/Xiaomi/Kernel_Flasher"
MODULES_DIR="/home/yarpiin/Android/Kernel/Xiaomi/Kernel_Flasher/modules/system/lib/modules"

# Toolchain paths
TOOLCHAIN_DIR="/home/yarpiin/Android/Toolchains"
CLANG_DIR="/home/yarpiin/Android/Toolchains/google-clang/bin"
GCC_DIR="/home/yarpiin/Android/Toolchains/google-gcc/bin"

# Kernel Details
YARPIIN_VER="WHITE WOLF KERNEL MI10 5G / PRO"
BASE_YARPIIN_VER="WHITE.WOLF.UNI.R."
UMI_VER="UMI"
CMI_VER="CMI"
CAS_VER="CAS"
VER=".016"
YARPIIN_UMI_VER="$BASE_YARPIIN_VER$UMI_VER$VER"
YARPIIN_CMI_VER="$BASE_YARPIIN_VER$CMI_VER$VER"
YARPIIN_CAS_VER="$BASE_YARPIIN_VER$CAS_VER$VER"

# Vars
export ARCH=arm64
export SUBARCH=arm64
export KBUILD_BUILD_USER=yarpiin
export KBUILD_BUILD_HOST=kernel

# Image dirs
ZIMAGE_DIR="$KERNEL_DIR/out/arch/arm64/boot"

# Output dir
ZIP_MOVE="/home/yarpiin/Android/Kernel/Zip"

# Functions
function clean_all {

		if [ -f "$MODULES_DIR/*.ko" ]; then
			rm `echo $MODULES_DIR"/*.ko"`
		fi
		if [ -f "$KERNELFLASHER_DIR/$KERNEL" ]; then
		    rm `echo $KERNELFLASHER_DIR/$KERNEL`
        fi
		if [ -f "$KERNELFLASHER_DIR/$DTBOIMG" ]; then
		    rm `echo $KERNELFLASHER_DIR/$DTBOIMG`
        fi
		cd $KERNEL_DIR
		echo
		make clean && make mrproper
        rm -rf out/
}

function make_umi_kernel {
		echo
        export LOCALVERSION=-`echo $YARPIIN_UMI_VER`
        export TARGET_PRODUCT=umi
        make O=out ARCH=arm64 $UMIDEFCONFIG

        PATH="$CLANG_DIR:$GCC_DIR:${PATH}" \
        make -j$(nproc --all) O=out \
                      ARCH=arm64 \
                      CC=clang \
                      CLANG_TRIPLE=aarch64-linux-gnu- \
                      CROSS_COMPILE=aarch64-linux-android-

		cp -vr $ZIMAGE_DIR/$KERNEL $KERNELFLASHER_DIR
		cp -vr $ZIMAGE_DIR/$DTBOIMG $KERNELFLASHER_DIR
        find ${KERNEL_DIR} -name '*.ko' -exec cp -v {} ${MODULES_DIR} \;
}

function make_cmi_kernel {
		echo
        export LOCALVERSION=-`echo $YARPIIN_CMI_VER`
        export TARGET_PRODUCT=cmi
        make O=out ARCH=arm64 $CMIDEFCONFIG

        PATH="$CLANG_DIR:$GCC_DIR:${PATH}" \
        make -j$(nproc --all) O=out \
                      ARCH=arm64 \
                      CC=clang \
                      CLANG_TRIPLE=aarch64-linux-gnu- \
                      CROSS_COMPILE=aarch64-linux-android-

		cp -vr $ZIMAGE_DIR/$KERNEL $KERNELFLASHER_DIR
		cp -vr $ZIMAGE_DIR/$DTBOIMG $KERNELFLASHER_DIR
        find ${KERNEL_DIR} -name '*.ko' -exec cp -v {} ${MODULES_DIR} \;
}

function make_cas_kernel {
		echo
        export LOCALVERSION=-`echo $YARPIIN_CAS_VER`
        export TARGET_PRODUCT=cas
        make O=out ARCH=arm64 $CASDEFCONFIG

        PATH="$CLANG_DIR:$GCC_DIR:${PATH}" \
        make -j$(nproc --all) O=out \
                      ARCH=arm64 \
                      CC=clang \
                      CLANG_TRIPLE=aarch64-linux-gnu- \
                      CROSS_COMPILE=aarch64-linux-android-

		cp -vr $ZIMAGE_DIR/$KERNEL $KERNELFLASHER_DIR
		cp -vr $ZIMAGE_DIR/$DTBOIMG $KERNELFLASHER_DIR
        find ${KERNEL_DIR} -name '*.ko' -exec cp -v {} ${MODULES_DIR} \;
}

function make_umi_zip {
		cd $KERNELFLASHER_DIR
		zip -r9 `echo $YARPIIN_UMI_VER`.zip *
		mv  `echo $YARPIIN_UMI_VER`.zip $ZIP_MOVE
		cd $KERNEL_DIR
}

function make_cmi_zip {
		cd $KERNELFLASHER_DIR
		zip -r9 `echo $YARPIIN_CMI_VER`.zip *
		mv  `echo $YARPIIN_CMI_VER`.zip $ZIP_MOVE
		cd $KERNEL_DIR
}

function make_cas_zip {
		cd $KERNELFLASHER_DIR
		zip -r9 `echo $YARPIIN_CAS_VER`.zip *
		mv  `echo $YARPIIN_CAS_VER`.zip $ZIP_MOVE
		cd $KERNEL_DIR
}

DATE_START=$(date +"%s")

echo -e "${green}"
echo "YARPIIN Kernel Creation Script:"
echo

echo "---------------------------"
echo "Kernel Version:"
echo "---------------------------"

echo -e "${red}"; echo -e "${blink_red}"; echo "$YARPIIN_VER"; echo -e "${restore}";

echo -e "${green}"
echo "---------------------------"
echo "Building White Wolf Kernel:"
echo "---------------------------"
echo -e "${restore}"

while read -p "Do you want to clean stuffs (y/n)? " cchoice
do
case "$cchoice" in
	y|Y )
		clean_all
		echo
		echo "All Cleaned now."
		break
		;;
	n|N )
		break
		;;
	* )
		echo
		echo "Invalid try again!"
		echo
		;;
esac
done

echo

while read -p "Do you want to build MI 10 kernel (y/n)? " dchoice
do
case "$dchoice" in
	y|Y)
		make_umi_kernel
		break
		;;
	n|N )
		break
		;;
	* )
		echo
		echo "Invalid try again!"
		echo
		;;
esac
done

echo

while read -p "Do you want to zip Mi10 5G kernel (y/n)? " dchoice
do
case "$dchoice" in
	y|Y)
		make_umi_zip
		break
		;;
	n|N )
		break
		;;
	* )
		echo
		echo "Invalid try again!"
		echo
		;;
esac
done

echo

while read -p "Do you want to clean stuffs (y/n)? " cchoice
do
case "$cchoice" in
	y|Y )
		clean_all
		echo
		echo "All Cleaned now."
		break
		;;
	n|N )
		break
		;;
	* )
		echo
		echo "Invalid try again!"
		echo
		;;
esac
done

echo

while read -p "Do you want to build MI 10 Pro kernel (y/n)? " dchoice
do
case "$dchoice" in
	y|Y)
		make_cmi_kernel
		break
		;;
	n|N )
		break
		;;
	* )
		echo
		echo "Invalid try again!"
		echo
		;;
esac
done

echo

while read -p "Do you want to zip Mi10 Pro kernel (y/n)? " dchoice
do
case "$dchoice" in
	y|Y)
		make_cmi_zip
		break
		;;
	n|N )
		break
		;;
	* )
		echo
		echo "Invalid try again!"
		echo
		;;
esac
done

echo

while read -p "Do you want to clean stuffs (y/n)? " cchoice
do
case "$cchoice" in
	y|Y )
		clean_all
		echo
		echo "All Cleaned now."
		break
		;;
	n|N )
		break
		;;
	* )
		echo
		echo "Invalid try again!"
		echo
		;;
esac
done

echo

while read -p "Do you want to build MI 10 Ultra kernel (y/n)? " dchoice
do
case "$dchoice" in
	y|Y)
		make_cas_kernel
		break
		;;
	n|N )
		break
		;;
	* )
		echo
		echo "Invalid try again!"
		echo
		;;
esac
done

echo

while read -p "Do you want to zip Mi10 Ultra kernel (y/n)? " dchoice
do
case "$dchoice" in
	y|Y)
		make_cas_zip
		break
		;;
	n|N )
		break
		;;
	* )
		echo
		echo "Invalid try again!"
		echo
		;;
esac
done

echo

echo
echo
echo -e "${green}"
echo "-------------------"
echo "Build Completed in:"
echo "-------------------"
echo -e "${restore}"

DATE_END=$(date +"%s")
DIFF=$(($DATE_END - $DATE_START))
echo "Time: $(($DIFF / 60)) minute(s) and $(($DIFF % 60)) seconds."
echo

