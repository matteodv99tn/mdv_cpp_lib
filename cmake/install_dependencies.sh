#!/bin/bash

os=`cat /etc/os-release | grep -Ev "(VERSION|URL|BUILD)" | sed "s/ID=\(.*\)/\1/" | grep -v "="`
echo "FOUND os: $os"


case "$os" in
    arch)
        pacman -Sy --noconfirm git cmake base-devel clang eigen fmt spdlog cgal gtest;
        ;;

    ubuntu)
        apt-get update -y;
        apt-get install -y git cmake  build-essential libeigen3-dev libfmt-dev libspdlog-dev libcgal-dev libgtest-dev;
        ;;
    *)
        echo "UNRECOGNISED OS!"
        return 1
        ;;
esac
