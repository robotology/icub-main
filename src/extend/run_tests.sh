#!/bin/bash

mkdir -p output
for f in `cd tests; ls test*$1*.txt`; do
    echo tests/$f
    ./extend < tests/$f > output/result_$f
done
