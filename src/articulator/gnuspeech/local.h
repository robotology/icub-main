// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifdef rint
#undef rint
#endif
#define rint(x) ((long int)(x))


#ifdef isnan
#undef isnan
#endif
#define isnan(x) (0)
#ifdef isinf
#undef isinf
#endif
#define isinf(x) (0)
