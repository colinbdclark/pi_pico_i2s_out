#!/bin/sh

export PICO_SDK_PATH=`pwd`/lib/pico-sdk
cmake -B build-docker "$@" && (cd build-docker && make -j4)
