#!/bin/bash
find . -name "*.h" -o -name "*.cxx" | xargs clang-format -i
