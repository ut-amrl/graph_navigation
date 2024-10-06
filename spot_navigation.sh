#!/bin/bash
set -x

if [ -z "$VERBOSE_LOGGING" ]; then
    VERBOSE_LOGGING=0
fi

echo VERBOSE_LOGGING=$VERBOSE_LOGGING

VERBOSE_LOGGING=$VERBOSE_LOGGING ./bin/navigation -robot_config /root/ut-amrl/spot_autonomy/config/navigation.lua
