#!/bin/bash

if [ -d build ]; then
    rm -rf build
fi

./tools/build.sh $1