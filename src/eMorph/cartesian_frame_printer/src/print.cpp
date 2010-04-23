#include "print.h"

C_print::C_print()
{
    cvNamedWindow( "Events_originals", 1 );
}
C_print::~C_print()
{
    cvDestroyWindow("Events_originals");
}

void C_print::print_events(ImageOf<PixelMono16> i_cFrame)
{
    IplImage* _tmp_cart_img = cvCreateImage(cvSize(128, 128), IPL_DEPTH_8U, 1);//cvCloneImage(reinterpret_cast<IplImage*>(i_cFrame.getIplImage()));
//    if(_tmp_cart_img == NULL)
//    {
#ifdef _DEBUG
        cout << "Ugly copy ..." << endl;
#endif
        for(int i=0; i<128; i++)
            for(int j=0; j<128; j++)
                ((uchar *)(_tmp_cart_img->imageData + (unsigned int)(127-i)*_tmp_cart_img->widthStep))[(unsigned int)(127-j)] = (uchar)i_cFrame(i,j);
//    }
    cvShowImage("Events_originals", _tmp_cart_img);

    cvWaitKey(10);

    cvReleaseImage(&_tmp_cart_img);
}
