
/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "gtest/gtest.h"

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);

	//::testing::GTEST_FLAG(filter) = "General.toEomn_positive_001";

	return RUN_ALL_TESTS();

}