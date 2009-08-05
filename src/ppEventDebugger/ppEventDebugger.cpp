// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include "ppEventDebugger.h"

#ifdef __LINUX__
#include <unistd.h>
#include <sys/io.h>
#include <stdio.h>
#endif
#ifdef __WIN32__
#include <windows.h>
#include "winio.c"
#endif 

ppEventDebugger::ppEventDebugger()
{
	initialized=false;
}

ppEventDebugger::ppEventDebugger(unsigned int address)
{ 
	if (open(address))
	{
	   initialized=true;
	}
}

ppEventDebugger::~ppEventDebugger()
{
	close();	
}

bool ppEventDebugger::open(unsigned int address)
{
    if (initialized)
        return true;

    base=address;
#ifdef __LINUX__
    if (ioperm(base, 1, 1))
        {
            fprintf(stderr, "Could not get the port at %x\n", base);
            return false;
        }
    initialized=true;
#endif
#ifdef __WIN32__
    if (OpenPortTalk()==-1)
        initialized=false;
    else
        initialized=true;
#endif

    return initialized;
}

void ppEventDebugger::close()
{
    if (!initialized)
	return;

    initialized=false;
#ifdef __LINUX__
    ioperm(base, 1, 0);
#endif

#ifdef __WIN32__
    ClosePortTalk();
#endif
}

void ppEventDebugger::set()
{
    if (!initialized)
	return;

    #ifdef __LINUX__
    outb(0xff, base);
    #endif
    #ifdef __WIN32__
    outp(base,0xff);
    #endif
}

void ppEventDebugger::reset()
{
   if (!initialized)
	return;
#ifdef __LINUX__
    outb(0x00, base);
#endif
#ifdef __WIN32__
    outp(base,0x00);
#endif
}
