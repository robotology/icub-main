// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium
 * Author: Giorgio Metta
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * based on code from Fabio Berton
 */

#include <iostream>
#include <string>
#include <iCub/RC_DIST_FB_logpolar_mapper.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>

using namespace yarp::sig;
using namespace iCub::logpolar;
using namespace std;

/*
 * main. usage: prog_name image_filename[.ppm]
 *
 */
int main (int argc, char *argv[])
{
    if (argc < 2) {
        cout << argv[0] << " : usage: " << argv[0] << " image_filename" << endl;
        return -1;
    }

    logpolarTransform trsf;
    const int nEcc = 152;
    const int nAng = 252;
    const double overlap = 2.00;

    cout << "||| reading input image" << endl;
    ImageOf<PixelRgb> inputImg;
    if (!yarp::sig::file::read(inputImg, argv[1])) {
        cerr << argv[0] << " : can't load image " << argv[1] << endl;
        return -1;
    }

    if (!yarp::sig::file::write(inputImg, "ex_out_orig.ppm")) {
        cerr << argv[0] << " : can't write original image" << endl;
    }

    cout << "||| got image of size (w x h): " << inputImg.width() << " " << inputImg.height() << endl;

    ImageOf<PixelRgb> lp;
    lp.resize(nAng, nEcc);

    //const int cartSize = (inputImg.width() < inputImg.height()) ? inputImg.width() : inputImg.height();
    trsf.allocLookupTables(nEcc, nAng, inputImg.width(), inputImg.height(), overlap);

    // assumes in & out are allocated and correctly sized.
    trsf.cartToLogpolar(lp, inputImg);

    cout << "||| saving logpolar image" << endl;
    if (!yarp::sig::file::write(lp, "ex_out_lp.ppm")) {
        cerr << argv[0] << " : can't write logpolar output image" << endl;
    }
    
    inputImg.zero();    // the mapping doesn't fill the borders.
    trsf.logpolarToCart(inputImg, lp);

    cout << "||| saving reconstructed image" << endl;
    if (!yarp::sig::file::write(inputImg, "ex_out_cart.ppm")) {
        cerr << argv[0] << " : can't write cartesian output image" << endl;
    }

    cout << "||| freeing memory" << endl;
    trsf.freeLookupTables();
    return 0;
}
