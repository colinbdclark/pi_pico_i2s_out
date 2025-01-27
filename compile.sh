#!/bin/sh

cmake -B build "$@" && (cd build && make -j4)
