#!/bin/sh
# Rehearse the first car session against the simulated car: every
# read-only kdiag step for engine, ABS, traction control and airbag, over
# the ECU process (ecusim serve ... must be running, ignition on).
# The adapter self-test needs real hardware and is not part of this.
#
# Usage: tools/car_day_rehearsal.sh [SOCKET]   (default: ecusim's socket)
set -u
sock=${1:-auto}
k=./target/release/kdiag
pass=0
fail=0
step() {
    name=$1
    shift
    if out=$("$@" 2>&1); then
        pass=$((pass + 1))
        printf 'PASS  %s\n' "$name"
    else
        fail=$((fail + 1))
        printf 'FAIL  %s\n' "$name"
    fi
    printf '%s\n' "$out" | sed 's/^/      /' | head -6
}
step "engine: identification"   $k ident --ecusim "$sock"
step "engine: fault codes"      $k dtc   --ecusim "$sock"
step "engine: live data (3 s)"  $k log   --ecusim "$sock" --duration-s 3
step "engine: via J2534 (mock)" $k dtc   --ecusim "$sock" --j2534-mock
step "ABS: fault codes"         $k dtc   --ecusim "$sock" --system abs
step "ABS: live data (2 s)"     $k log   --ecusim "$sock" --system abs --duration-s 2
step "TCS: fault codes"         $k dtc   --ecusim "$sock" --system tcs
step "TCS: live data (2 s)"     $k log   --ecusim "$sock" --system tcs --duration-s 2
step "SRS: identification"      $k ident --ecusim "$sock" --system srs
step "SRS: fault codes"         $k dtc   --ecusim "$sock" --system srs
step "SRS: crash records"       $k crash --ecusim "$sock" --system srs
printf '\n%d passed, %d failed\n' "$pass" "$fail"
[ "$fail" -eq 0 ]
