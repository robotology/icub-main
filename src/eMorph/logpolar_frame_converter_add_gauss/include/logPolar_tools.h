#ifndef C_LOGPOLAR_TOOLS
#define C_LOGPOLAR_TOOLS

#include <cmath>
#include <iostream>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846 
#endif

//#define _DEBUG

using namespace std;
class C_logpolarTools
{
public:
    C_logpolarTools(int, int);
    ~C_logpolarTools();

    int get_rho(int, int);
    int get_phi(int, int);

    vector<int> get_x(int, int);
    vector<int> get_y(int, int);

    int get_logCart_x(int, int);
    int get_logCart_y(int, int);

    int get_m(){return rho_max;};
    int get_n(){return n;};
private:
    void initData();
    void createTables();

    int** rho_xy;
    int** phi_xy;

    vector<int>** x_rhophi;
    vector<int>** y_rhophi;

    int** logCart_x;
    int** logCart_y;

    int cart_height;
    int cart_width;

    int x_0;
    int y_0;
    int rho_0;
    int rho_max;
    int r_0;
    int r_max;
    int m;
    int n;
    float mu;
    float M;
    float alpha;
    float beta;
};
#endif //C_LOGPOLAR_TOOLS
