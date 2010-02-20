// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/Lib0.h> //dependency
#include <iCub/Lib1.h>

#include <stdio.h>

void Lib1::lib1() {
    Lib0 lib0;
    printf("Warning using lib1\n");
    printf("We need lib0\n");
    lib0.lib0();
}
