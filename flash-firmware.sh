#!/bin/sh

if [ -z "$1" ]; then
    echo "Error: No firmware file specified."
    echo "Usage: $0 <firmware-file>.elf"
    exit 1
fi

$PICO_OPENOCD_PATH/openocd -s $PICO_OPENOCD_PATH/scripts -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" -c "program $1 verify reset exit"
