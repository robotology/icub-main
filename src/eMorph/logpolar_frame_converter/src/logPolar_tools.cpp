#include "logPolar_tools.h"

C_logpolarTools::C_logpolarTools(int i_height, int i_width):cart_height(i_height), cart_width(i_width)
{
#ifdef _DEBUG
	cout << "C_logpolarTools::C_logpolarTools(...)" << endl;
#endif
    this->initData();
    this->createTables();
}

C_logpolarTools::~C_logpolarTools()
{
    for(int i=0; i<cart_width; i++)
    {
        delete[] rho_xy[i];
        delete[] phi_xy[i];
    }
    delete[] rho_xy;
    delete[] phi_xy;

    for(int i=0; i<m; i++)
    {
        delete[] x_rhophi[i];
        delete[] y_rhophi[i];
        delete[] logCart_x[i];
        delete[] logCart_y[i];
    }
    delete[] x_rhophi;
    delete[] y_rhophi;
    delete[] logCart_x;
    delete[] logCart_y;
}

void C_logpolarTools::initData()
{
#ifdef _DEBUG
	cout << "C_logpolarTools::initData()" << endl;
#endif
    mu = 1.0;
    rho_0 = 2;
    r_0 = 2;
    x_0 = (int) (floor/*ceil*/(((float)cart_width+1)/2));
    y_0 = (int) (floor/*ceil*/(((float)cart_height+1)/2));
    int r_tmp_1 = (int)sqrt(double((cart_width-x_0)*(cart_width-x_0)+(cart_height-y_0)*(cart_height-y_0)));
    r_max = r_tmp_1;
    alpha = ((float)r_max - mu * (float)r_0 * ((float)r_0+0.5))/(mu * (float)r_0 - 1.0);
    beta = ((float)r_0 + alpha)*log((float)r_0 + alpha) - (float)r_0 - 0.5;
    M = (float)r_0+alpha;
    rho_max = (int)floor(M * log((float)r_max + alpha) - beta + 0.5);
    m = rho_max-rho_0; //Number of ring in the retinal image
    n = 64;
    rho_xy = new int*[cart_width];
    phi_xy = new int*[cart_width];
    for(int i=0; i<cart_width; i++)
    {
        rho_xy[i] = new int[cart_height];
        phi_xy[i] = new int[cart_height];
        for(int j=0; j<cart_height; j++)
        {
            rho_xy[i][j] = -1;
            phi_xy[i][j] = -1;
        }
    }
    x_rhophi = new vector<int>*[rho_max];
    y_rhophi = new vector<int>*[rho_max];
    logCart_x = new int*[rho_max];
    logCart_y = new int*[rho_max];
    for(int i=0; i<rho_max; i++)
    {
        x_rhophi[i] = new vector<int>[n];
        y_rhophi[i] = new vector<int>[n];
        logCart_x[i] = new int[n];
        logCart_y[i] = new int[n];
        for(int j=0; j<n; j++)
        {
            logCart_x[i][j]=-1;
            logCart_y[i][j]=-1;
        }
    }
}

void C_logpolarTools::createTables()
{
#ifdef _DEBUG
	cout << "C_logpolarTools::createTables()" << endl;
#endif
    float r = 0.0;
    float _atan = 0.0;
    for(int i=0; i<cart_width;i++)
        for(int j=0; j<cart_height; j++)
        {
            r=sqrt(double((i-x_0)*(i-x_0)+(j-y_0)*(j-y_0)));
            if(floor(r+0.5)>r_0)
            {
                rho_xy[i][j]=floor(M*log(r+alpha)-beta/*+0.5*/);
                _atan = atan2(double(j-y_0), double(i-x_0));
                if(_atan < 0)
                    _atan = _atan+2*M_PI;
                phi_xy[i][j]=floor((n/(2*M_PI)) * _atan/*+0.5*/);
                if((rho_xy[i][j]>=rho_max) || (phi_xy[i][j]>=n))
                {
                    if(rho_xy[i][j]>rho_max)
                        cout << "Error : rho_xy[" <<i<<"]["<<j<<"]>m " << rho_xy[i][j] << " - " << rho_max << endl;
                    if (phi_xy[i][j]>n)
                        cout << "Error : phi_xy["<<i<<"]["<<j<<"]>n " << phi_xy[i][j] << " - " << n << endl;
                }
                else
                {
                    int tmp_rho = rho_xy[i][j]!=0?rho_xy[i][j]-1:rho_xy[i][j];
                    int tmp_phi = phi_xy[i][j]!=0?phi_xy[i][j]-1:phi_xy[i][j];
                	x_rhophi[tmp_rho][tmp_phi].push_back(i);
                	y_rhophi[tmp_rho][tmp_phi].push_back(j);
                	if(x_rhophi[tmp_rho][tmp_phi].size()==1)
                	{
#ifdef _DEBUG
                	    cout << "Init the center computation" << endl;
#endif
                	    logCart_x[tmp_rho][tmp_phi] = i;
                        logCart_y[tmp_rho][tmp_phi] = j;
                	}
                	else
                	{
                	    logCart_x[tmp_rho][tmp_phi] = logCart_x[tmp_rho][tmp_phi] + (1/x_rhophi[tmp_rho][tmp_phi].size())*(i-logCart_x[tmp_rho][tmp_phi]);
                	    logCart_y[tmp_rho][tmp_phi] = logCart_y[tmp_rho][tmp_phi] + (1/y_rhophi[tmp_rho][tmp_phi].size())*(j-logCart_y[tmp_rho][tmp_phi]);
#ifdef _DEBUG
                	    cout << "Computed center : " << logCart_x[tmp_rho][tmp_phi] << " / " << logCart_y[tmp_rho][tmp_phi] << endl;
#endif
                	}
                }
            }
        }
}

int C_logpolarTools::get_phi(int i_x, int i_y)
{
    int res = -1;
    if( (i_x < cart_width) && (i_y < cart_height) )
        res = phi_xy[i_x][i_y];
    return res;
}

int C_logpolarTools::get_rho(int i_x, int i_y)
{
    int res = -1;
    if( (i_x < cart_width) && (i_y < cart_height) )
        res = rho_xy[i_x][i_y];
    return res;
}

vector<int> C_logpolarTools::get_x(int i_rho, int i_phi)
{
    vector<int> res;
    if( (i_rho < rho_max) && (i_phi < n) )
        res = x_rhophi[i_rho][i_phi];
    return res;

}

vector<int> C_logpolarTools::get_y(int i_rho, int i_phi)
{
    vector<int> res;
    if( (i_rho < rho_max) && (i_phi < n) )
        res = y_rhophi[i_rho][i_phi];
    return res;
}

int C_logpolarTools::get_logCart_x(int i_rho, int i_phi)
{
    int res = -1;
    if( (i_rho < rho_max) && (i_phi < n) )
        res = logCart_x[i_rho][i_phi];
    return res;
}


int C_logpolarTools::get_logCart_y(int i_rho, int i_phi)
{
    int res = -1;
    if( (i_rho < rho_max) && (i_phi < n) )
        res = logCart_y[i_rho][i_phi];
    return res;
}
