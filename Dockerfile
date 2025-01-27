FROM ubuntu:latest

RUN apt-get update && \
    apt-get clean &&  \
    apt-get install -y \
        wget \
        curl \
        git  \
        python3 \
        cmake \
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
        build-essential \
        libstdc++-arm-none-eabi-newlib

# Install Pi Pico SDK for building picotool with
WORKDIR /
RUN git clone https://github.com/raspberrypi/pico-sdk.git --branch 2.1.0
WORKDIR /pico-sdk
RUN git submodule update --init
ENV PICO_SDK_PATH=/pico-sdk

# Build and install picotool
WORKDIR /
RUN git clone https://github.com/raspberrypi/picotool.git --branch 2.1.0
WORKDIR /picotool
RUN git submodule update --init
RUN mkdir build
WORKDIR /picotool/build
RUN cmake ../
RUN make
RUN cmake --install .

WORKDIR /project
