#pragma once

#include <ace/Auto_Event.h>

class SuperPort : public BufferedPort<Bottle>
{
protected:
    ACE_Auto_Event ev;
    Bottle sol;

public:
    SuperPort()
    {
        useCallback();        
        ev.reset();
    }

    virtual void onRead(Bottle& b)
    {
        sol=b;
        ev.signal();
    }

    void wait(Bottle& b)
    {
        ev.wait();
        b=sol;
        ev.reset();
    }
};

