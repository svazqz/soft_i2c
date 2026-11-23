#!/bin/sh
set -eu
export LC_ALL=C

driver=${1:?Pass the driver name}
workdir=$(mktemp -d)
trap 'rm -rf "$workdir"' EXIT
cp -R /src/Makefile /src/include /src/src /src/tests "$workdir"
cd "$workdir"

make BUILD_DIR=build/sanitized CC=gcc \
  CFLAGS='-O1 -g -Werror -fsanitize=address,undefined -fno-omit-frame-pointer -fno-pie' \
  LDFLAGS='-fsanitize=address,undefined -no-pie' test
make BUILD_DIR=build/coverage CC=gcc CFLAGS='-O0 -g --coverage' test

report=$(gcov -b -o "build/coverage/test_$driver-$driver.gcno" "src/$driver.c")
printf '%s\n' "$report"

# The first gcov section describes the production driver, not the test files.
line_percent=$(printf '%s\n' "$report" | awk -F '[:%]' '/^Lines executed:/{print $2; exit}')
if ! awk -v coverage="$line_percent" 'BEGIN { exit !(coverage ~ /^[0-9]+([.][0-9]+)?$/ && coverage >= 95) }'; then
  printf 'Expected at least 95%% production line coverage, got %s%%.\n' "$line_percent" >&2
  exit 1
fi
