#!/bin/sh


echo "-------------------------------------------------"
echo "    POST script @template@ install configuration"
echo "-------------------------------------------------"

TARGET_DIR=$1
PACKAGE_@TEMPLATE@_DIR=$O/../package/@template@

echo ""
echo "install network @template@ interfaces"
cp -v $PACKAGE_@TEMPLATE@_DIR/interfaces $TARGET_DIR/etc/network/

echo ""
echo "install Volta init file"
cp -v $PACKAGE_@TEMPLATE@_DIR/S30@template@ $TARGET_DIR/etc/init.d/.

echo ""
echo "install wifi for Volta init file"
cp -v $PACKAGE_@TEMPLATE@_DIR/S45wifi $TARGET_DIR/etc/init.d/.

echo ""
echo "install inittab special @template@"
cp -v $PACKAGE_@TEMPLATE@_DIR/inittab $TARGET_DIR/etc/inittab