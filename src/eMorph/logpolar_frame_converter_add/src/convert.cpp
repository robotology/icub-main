#include "convert.h"

C_converter::C_converter(int i_width, int i_height):lp_tools(i_width,i_height)
{
#ifdef _DEBUG
	cout << "C_printingThread::C_printingThread(...)" << endl;
#endif
    base_clImg.resize(i_width, i_height);
    for(int i=0; i<i_width; i++)
    {
        for(int j=0; j<i_height; j++)
            base_clImg(i, j) = 125;
    }
    m = lp_tools.get_m();
    n = lp_tools.get_n();

    base_lImg.resize(m, n);
    acc = new int*[m];
    postS = new double*[m];
    t_0 = new double*[m];
    spiked = new bool*[m]; //Test

    int* _pixels_size = new int[m];
    int _total_pixels = 0;
    int _middle = (int)(floor(((float)n+1)/2));


    for(int i=0; i<m; i++)
    {
        acc[i] = new int[n];
        postS[i] = new double[n];
        t_0[i] = new double[n];
        spiked[i] = new bool[n]; //Test
        _pixels_size[i] = lp_tools.get_x(i, _middle).size();
        _total_pixels += _pixels_size[i];
        for(int j=0; j<n; j++)
        {
            acc[i][j] = 0;
            postS[i][j] = 0.0;
            t_0[i][j] = 0.0;
            spiked[i][j] = false;
            base_lImg(i, j) = 125;
        }
    }
    height = i_height;
    width = i_width;

    mu = (float)_total_pixels/(float)m;
    theta = 0;
    for(int i=0; i<m; i++)
        theta += ((float)_pixels_size[i]-mu)*((float)_pixels_size[i]-mu);
    theta = sqrt((1/(float)m)*theta);
    cout << "mean : " << mu << endl
        << "Standard_deviation : " << theta << endl;
    port.open("/image/logpolar_add");
}
C_converter::~C_converter()
{
    for(int i=0; i<m; i++)
    {
        delete[] acc[i];
        delete[] postS[i];
        delete[] t_0[i];
        delete[] spiked[i]; //Test
    }
    delete[] acc;
    delete[] postS;
    delete[] t_0;
    delete[] spiked; //Test

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

//Variables for the construction of the logpolar image
    int _rho;
    int _phi;
    int t_diff;
    AER_struct _AER;

//Variables for the reconstruction of the cartesian image from the logpolar image
    vector<int> tmp_list_x;
    vector<int> tmp_list_y;

#ifdef _DEBUG
    cout << "Initialisation of the image in grey value" << endl;
#endif
    ImageOf<PixelMono16> tmp_lImg = base_lImg;
    ImageOf<PixelMono16> tmp_clImg = base_clImg;

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
    if(size_stack > 0)
    {
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
                //**Logpolar image computation
                _rho = lp_tools.get_rho(_x, _y);
                _phi = lp_tools.get_phi(_x, _y);
                if(_rho >= m || _phi >= n)
                {
#ifdef _DEBUG
                    cout << "Problem of index ..." << endl;
#endif
                }
                else if((_rho == -1) || (_phi == -1))
                {
#ifdef _DEBUG
                     cout << "Indexing error" << endl;
#endif
                }
                else
                {
//                    switch(sign(_pol))
//                    {
//                        case -1: tmp_lImg(_rho, _phi)=tmp_lImg(_rho, _phi)>=25?tmp_lImg(_rho, _phi)+_pol*10:0/*0*/;
//                            break;
//                        case 1: tmp_lImg(_rho, _phi)=tmp_lImg(_rho, _phi)<=225?tmp_lImg(_rho, _phi)+_pol*10:255/*255*/;
//                            break;
//                        case 0: tmp_lImg(_rho, _phi)=125; break;
//                    }
                    acc[_rho][_phi] += _pol;
                    switch(sign(acc[_rho][_phi]))
                    {
                        case -1: tmp_lImg(_rho, _phi)=125+acc[_rho][_phi]*10;
                            if(tmp_lImg(_rho, _phi)<0)
                                tmp_lImg(_rho, _phi) = 0;
                            break;
                        case 1: tmp_lImg(_rho, _phi)=125+acc[_rho][_phi]*10;
                            if(tmp_lImg(_rho, _phi)>255)
                                tmp_lImg(_rho, _phi) = 255;
                            break;
                        case 0: tmp_lImg(_rho, _phi)=125; break;
                    }
                }
            }
            else
            {
                AER_struct tmp_struct;
                tmp_struct.pol = 0;
                tmp_struct.ts = 0;
                for(int i=0; i<m;i++)
                    for(int j=0; j<n; j++)
                    {
                        tmp_list_x = lp_tools.get_x(i, j);
                        tmp_list_y = lp_tools.get_y(i, j);
                        if(!tmp_list_x.empty())
                            for(int k=0; k<tmp_list_x.size(); k++)
                            {
                                tmp_clImg((unsigned int)tmp_list_x[k], (unsigned int)tmp_list_y[k])=
                                    tmp_lImg((unsigned int)i,(unsigned int)j);
                            }
                    }

#ifdef _DEBUG
                cout << "Initialisation of the table of accumulation of the events" << endl;
#endif
                //Reinit the variable of the logpolar processing
                for(int i=0; i<m; i++)
                {
                    for(int j=0; j<n; j++)
                    {
                        acc[i][j] = 0;
                        postS[i][j] = 0.0;
                        spiked[i][j] = false; //test
                    }
                }
            }
        }
    }
    return tmp_clImg;
}

void C_converter::send_frame(ImageOf<PixelMono16> i_frame)
{
    ImageOf<PixelMono16>& tmp = port.prepare();
    tmp = i_frame;
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
