#include "cartesian_frame_converter.h"

C_cframeConverter::C_cframeConverter():convert_events(128,128)
{
}
C_cframeConverter::~C_cframeConverter()
{
}
void C_cframeConverter::onRead(C_sendingBuffer& i_ub)
{
#ifdef _DEBUG
	cout << "C_yarpViewer::onRead(C_unmaskedbuffer& i_ub)" << endl;
    start_u = clock();
#endif
    list<AER_struct> datas = unmask_events.unmaskData(i_ub.get_packet(), i_ub.get_sizeOfPacket());
#ifdef _DEBUG
    start_p = clock();
#endif
    ImageOf<PixelMono16> frame = convert_events.create_frame(datas);
    convert_events.send_frame(frame);
#ifdef _DEBUG
    stop = clock();
    cout << "Unmask task : " << (stop/CLOCKS_PER_SEC) - (start_u/CLOCKS_PER_SEC) << endl
        << "Printing task : " << (stop/CLOCKS_PER_SEC) - (start_p/CLOCKS_PER_SEC) << endl;
#endif
}
