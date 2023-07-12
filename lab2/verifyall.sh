#!/bin/bash
set -x
for testcase in adaptec1 adaptec2 adaptec3 adaptec4 adaptec5 bigblue1 bigblue2 bigblue3 newblue1 newblue2 newblue5 newblue6 
do
    perl eval2008.pl $testcase.gr $testcase.sol
    echo "return value is $?"
done
