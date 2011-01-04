// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeWorldManager.h"

#include <stdio.h>

void FakeWorldManager::apply(const WorldOp& op, WorldResult& result) {
    printf("*** fake world ***\n");
    result.msg = "not implemented";
}

