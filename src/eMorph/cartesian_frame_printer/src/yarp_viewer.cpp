#include "yarp_viewer.h"

C_cFramePrinter::C_cFramePrinter()
{
}
C_cFramePrinter::~C_cFramePrinter()
{
}
void C_cFramePrinter::onRead(ImageOf<PixelMono16>& i_img)
{
#ifdef _DEBUG
	cout << "C_yarpViewer::onRead(C_unmaskedbuffer& i_ub)" << endl;
    start_p = clock();
#endif
    print_events.print_events(i_img);
#ifdef _DEBUG
    stop = clock();
    cout << "Printing task : " << (stop/CLOCKS_PER_SEC) - (start_p/CLOCKS_PER_SEC) << endl;
#endif
}
