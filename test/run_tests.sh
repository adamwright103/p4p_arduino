#!/usr/bin/env bash
# Compiles and runs the host-side tests. No Arduino toolchain needed: shim/
# provides just enough of Arduino.h for the files under test.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
g++ -std=c++17 -Wall -Wextra -I"$HERE/shim" -I"$HERE/.." \
    -o "$HERE/speedmap_test" "$HERE/speedmap_test.cpp" "$HERE/../speedmap.cpp"
"$HERE/speedmap_test"
