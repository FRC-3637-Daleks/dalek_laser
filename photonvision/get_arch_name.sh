#!/bin/sh

ARCH=$(uname -m)
ARCH_NAME=""
if [ "$ARCH" = "aarch64" ]; then
  ARCH_NAME="linuxarm64"
elif [ "$ARCH" = "armv7l" ]; then
  exit -1
elif [ "$ARCH" = "x86_64" ]; then
  ARCH_NAME="linuxx64"
else
  exit -1
fi

echo $ARCH_NAME