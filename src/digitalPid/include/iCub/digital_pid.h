// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

#ifndef ICUB_DIGITAL_PID
#define ICUB_DIGITAL_PID

/* 
Implements a digital PID controller with integrator windup:

u(kh) = p(kh) + i(kh) + d(kh)

p(kh) = K*(b*ref(kh)-y(kh))

i(kh) = i(kh-h) + K*h/Ti*e(kh-h) + h/Tt*(sat(u(kh-h))-u(kh-h))

d(kh) = Td/(Td+N*h)*d(kh-h) - K*Td*N/(Td+N*h)*(y(kh-y))

*/

class DigitalPID
{
private:
    double h;       //sampling period
    double K;       //proportional gain
    double Ti;      //integral time
    double Td;      //derivative time
    double Tt;      //anti-windup reset time (from 0.1 to 1 the value of Ti) 
    double b;       //fraction of the reference influencing the proportional term
    double N;       //derivative low-pass filter bandwidth (3 to 20, typ. 10)
    double uhigh;   //upper saturation limit
    double ulow;    //lower saturation limit

    double y_before;
    double u_before;
    double i_before;
    double d_before;
    double e_before;

public;

    DigitalPID()
    {
        //sets defaults
        h = 1;
        K = 1;
        Ti = 0;
        Td = 0;
        Tt = 0;
        b = 1;
        N = 10;
        uhigh = 0;
        ulow = 0;
        Reset();
    };
    ~DigitalPID() {};

    void Reset() 
    {
        y_before = 0;
        u_before = 0;
        i_before = 0 ;
        d_before = 0;
        e_before = 0;
    };

    void SetSamplingPeriod(double val){h=val;};
    void SetProportionalGain(double val){K=val;};
    void SetDerivativeTime(double val){Td=val;};
    void SetIntegralTime(double val){Ti=val;};
    void SetIntegralGain(double val){(val==0 ? Ti = 0; Ti = 1/val);};
    void SetResetTime(double val){Tt = val;};
    void SetCommandFraction(double val){b=val;};
    void SetDerivativeBandwidth(double val){N=val;};
    void SetUpperLimit(double val){uhigh=val;};
    void SetLowerLimit(double val){ulow=val;};

    double GetSamplingPeriod(){return h;};
    double GetProportionalGain(){return K;};
    double GetDerivativeTime(){return Td;};
    double GetIntegralTime(){return Ti;};
    double GetIntegralGain(){if(Ti==0) return 0; else return 1/Ti;};
    double GetResetTime(){return Tt;};
    double GetCommandFraction(){return b;};
    double GetDerivativeBandwidth(){return N;};
    double GetUpperLimit(){return uhigh;};
    double GetLowerLimit(){return ulow;};

    double Compute(double y, double ref)
    {
        double p, e, i, s, d, u;


        if(u_before > uhigh)
            s = uhigh - u_before;
        else if (u_before < ulow)
            s = ulow - u_before;
        else 
            s = 0;
                    
        p = K*(b*ref-y);
        e = ref - y;
        if(Ti != 0)
        {
            i = i_before + K*h/Ti*e_before;
            if(Tt != 0)
                i += s*h/Tt;
        }
        else
            i = 0;

        e_before = e;

        if(Td != 0)
        {
            d = Td/(Td+N*h)-K*Td*N/(Td+N*h)*(y-y_before);
        }
        else 
            d = 0;

        y_before = y;

        u = p + i + d;
        u_before = u;

        if( u  > uhigh )
            u = uhigh;
        else if (u < ulow)
            u = ulow;

        return u;

    };
};


#endif /* ICUB_DIGITAL_PID */

