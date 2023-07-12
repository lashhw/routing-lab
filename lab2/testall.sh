#!/bin/bash
set -x
for testcase in adaptec1 adaptec2 adaptec3 adaptec4 adaptec5 bigblue1 bigblue2 bigblue3 newblue1 newblue2 newblue5 newblue6 
do
    timeout --signal=KILL 30m ./router $testcase.gr $testcase.sol 1800 | tee $testcase.log
    echo "return value is ${PIPESTATUS[0]}|${PIPESTATUS[1]}"
done
