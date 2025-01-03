#!/bin/bash

os=`cat /etc/os-release | grep -Ev "(VERSION|URL|BUILD)" | sed "s/ID=\(.*\)/\1/" | grep -v "="`
echo "FOUND os: $os"


case "$os" in
    arch)

        ;;

    ubuntu)
        apt-get update -y;
        apt-get install -y git cmake libeigen3-dev libfmt-dev libspdlog-dev libcgal-dev catch2;
        ;;
    *)
        echo "UNRECOGNISED OS!"
        return 1
        ;;
esac
