#include "convert.h"

C_converter::C_converter(int i_width, int i_height)
{
#ifdef _DEBUG
	cout << "C_printingThread::C_printingThread(...)" << endl;
#endif
    base_img.resize(i_width, i_height);
    cart_pol_acc = new int*[i_width];
    for(int i=0; i<i_width; i++)
    {
        cart_pol_acc[i] = new int[i_height];
        for(int j=0; j<i_height; j++)
        {
            cart_pol_acc[i][j] = 0;
            base_img(i,j) = 125;
        }
    }
    height = i_height;
    width = i_width;

    port.open("/image/cartesian");
}
C_converter::~C_converter()
{
    for(int i=0; i<width; i++)
        delete[] cart_pol_acc[i];
    delete[] cart_pol_acc;

    port.close();
}

ImageOf<PixelMono16> C_converter::create_frame(list<AER_struct> i_lAER)
{
    int size_stack;
//Variables for the construction of the cartesian image
    int _x;
    int _y;
    int _pol;
    double _ts;

#ifdef _DEBUG
    cout << "Initialisation of the table of accumulation of the events" << endl;
#endif
    start = clock();
    while(( ((double)start/CLOCKS_PER_SEC)*1e3+20.0) > (((double)clock()/CLOCKS_PER_SEC)*1e3))
    {
    // #ifdef _DEBUG
    //             cout << ((double)start/CLOCKS_PER_SEC)*1e3+200.0 << "\t" << ((double)clock()/CLOCKS_PER_SEC)*1e3 << endl;
    // #endif
    }
#ifdef _DEBUG
    cout << "Size of the list (number of event packed) : " << i_lAER.size() << endl;
#endif
    size_stack = (int)i_lAER.size();

    //Init the base-frame in gray value
    ImageOf<PixelMono16> _tmp_cart_img = base_img;

    if(size_stack > 0)
    {
        for(int i=0; i<width; i++)
        {
            for(int j=0; j<height; j++)
                cart_pol_acc[i][j] = 0;
        }
        for(int index=0;index<size_stack;index++)
        {
            AER_struct event = i_lAER.front();
            _x = event.y;//event.x;
            _y = event.x;//event.y;
            _pol = event.pol;
            _ts = event.ts;
            i_lAER.pop_front();
//            cout << '\t' << _x << " " << _y << " " << _pol << " " << _ts << endl;

            if(_x!=-1 && _y!=-1)
            {
                //**Cartesian image computation
                cart_pol_acc[_x][_y]+=_pol;
                if(cart_pol_acc[_x][_y]< 0)
                    _tmp_cart_img(_x,_y)=0;
                else
                    if(cart_pol_acc[_x][_y]> 0)
                        _tmp_cart_img(_x,_y)=255;
                    else
                        _tmp_cart_img(_x,_y)=125;
                //******************************
            }
        }
    }
    return _tmp_cart_img;
}

void C_converter::send_frame(ImageOf<PixelMono16> i_img)
{
    ImageOf<PixelMono16>& tmp = port.prepare();
    tmp = i_img;
    port.write();
}

int C_converter::sign(int i_val)
{
    if(i_val > 0)
        return 1;
    else if(i_val < 0)
        return -1;
    else
        return 0;

}

float C_converter::mean_event(int i_nE)
{
    static float number_of_data = 0.0;
    static float mean_time_loop = 0.0;
    number_of_data++;
    if(number_of_data == 1)
        mean_time_loop = (float)i_nE;
    else
        mean_time_loop = mean_time_loop + (1/number_of_data)*((float)i_nE-mean_time_loop);
    return mean_time_loop;
}
