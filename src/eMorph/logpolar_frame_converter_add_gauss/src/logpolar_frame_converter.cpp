#include "logpolar_frame_converter.h"

C_lFrameConverter::C_lFrameConverter():convert_events(128,128)
{
}
C_lFrameConverter::~C_lFrameConverter()
{
}
void C_lFrameConverter::onRead(C_sendingBuffer& i_ub)
{
#ifdef _DEBUG
	cout << "C_yarpViewer::onRead(C_unmaskedbuffer& i_ub)" << endl;
    start_u = clock();
#endif
    list<AER_struct> datas = unmask_events.unmaskData(i_ub.get_packet(), i_ub.get_sizeOfPacket());
#ifdef _DEBUG
    start_p = clock();
#endif
    ImageOf<PixelMono16> logFrame = convert_events.create_frame(datas);
    convert_events.send_frame(logFrame);
#ifdef _DEBUG
    stop = clock();
    cout << "Unmask task : " << (stop/CLOCKS_PER_SEC) - (start_u/CLOCKS_PER_SEC) << endl
        << "Printing task : " << (stop/CLOCKS_PER_SEC) - (start_p/CLOCKS_PER_SEC) << endl;
#endif
}
