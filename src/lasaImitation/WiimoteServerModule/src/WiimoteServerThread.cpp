// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Eric Sauser
 * email:   eric.sauser@a3.epfl.ch
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

 
#include "WiimoteServerThread.h"

#include <string.h>
#include <iostream>
using namespace std;

wiimote**            WiimoteServerThread::sWiimotes     = NULL;
WiimoteServerThread::WiimoteState         WiimoteServerThread::sWState;
WiimoteServerThread::WiimoteState         WiimoteServerThread::sWPrevState;
WiimoteServerThread::WiimoteState         WiimoteServerThread::sWEventState;
bool                 WiimoteServerThread::sDisconnected = false;
bool                 WiimoteServerThread::sGotEvent     = false;

WiimoteServerThread::WiimoteServerThread(int period, const char* baseName)
:RateThread(period)
{
    mPeriod = period;
    strncpy(mBaseName,baseName,256);
    bEventMode = true;
}

WiimoteServerThread::~WiimoteServerThread()
{}

bool WiimoteServerThread::threadInit()
{
    char portName[256];
    snprintf(portName,256,"/%s/output",mBaseName);
    mOutputPort.open(portName);

    memset(&sWState,' ',sizeof(WiimoteState));
    sWState.bNULL = 0;
    memset(&sWPrevState,' ',sizeof(WiimoteState));
    sWPrevState.bNULL = 0;
    memset(&sWEventState,' ',sizeof(WiimoteState));
    sWEventState.bNULL = 0;

    
    
    return true;
}

void WiimoteServerThread::threadRelease()
{
    mOutputPort.close();
}

void WiimoteServerThread::run()
{
    mMutex.wait();
    
    memset(&sWEventState,' ',sizeof(WiimoteState));
    sWEventState.bNULL = 0;
    sGotEvent = false;
    while (wiiuse_poll(sWiimotes, MAX_WIIMOTES)) {
        int i = 0;
        for (; i < MAX_WIIMOTES; ++i) {
            switch (sWiimotes[i]->event) {
                case WIIUSE_EVENT:
                    /* a generic event occured */
                    handle_event(sWiimotes[i]);
                    break;

                case WIIUSE_STATUS:
                    /* a status event occured */
                    handle_ctrl_status(sWiimotes[i]);
                    break;

                case WIIUSE_DISCONNECT:
                case WIIUSE_UNEXPECTED_DISCONNECT:
                    /* the wiimote disconnected */
                    handle_disconnect(sWiimotes[i]);
                    break;

                default:
                    break;
            }
        }
    }

    if(bEventMode?sGotEvent:true){
        //cout << "<" << ((char*)&sWEventState) <<">"<<endl;
        //cout << "<" << ((char*)&sWState) <<">"<<endl;
        //cout << "<---------------------------->"<<endl;

        // Write data to output port
        Vector &outputVec = mOutputPort.prepare();
        outputVec.resize(11);
        for(int i=0;i<11;i++){
            char c;
            if(bEventMode)  c = ((char*)&sWEventState)[i]; 
            else            c = ((char*)&sWState)[i]; 
            if(c=='x')
                outputVec[i] =  1.0;
            else if (c=='.')
                outputVec[i] = -1.0;
            else 
                outputVec[i] =  0.0;
        }
        if(bEventMode)
            mOutputPort.writeStrict();
        else 
            mOutputPort.write();
    }

    mMutex.post();
}

void WiimoteServerThread::SetEventMode(bool event){
    mMutex.wait();
    bEventMode = event;
    mMutex.post();
}

#define ON_CHAR 'x'

void WiimoteServerThread::handle_event(struct wiimote_t* wm) {

    memset(&sWState,' ',sizeof(WiimoteState));
    sWState.bNULL = 0;
    
    if (wm->btns) {    
        if (IS_PRESSED(wm, WIIMOTE_BUTTON_A))     sWState.bA = ON_CHAR; 
        if (IS_PRESSED(wm, WIIMOTE_BUTTON_B))     sWState.bB = ON_CHAR; 
        if (IS_PRESSED(wm, WIIMOTE_BUTTON_UP))    sWState.bUP = ON_CHAR; 
        if (IS_PRESSED(wm, WIIMOTE_BUTTON_DOWN))  sWState.bDOWN = ON_CHAR; 
        if (IS_PRESSED(wm, WIIMOTE_BUTTON_LEFT))  sWState.bLEFT = ON_CHAR; 
        if (IS_PRESSED(wm, WIIMOTE_BUTTON_RIGHT)) sWState.bRIGHT = ON_CHAR; 
        if (IS_PRESSED(wm, WIIMOTE_BUTTON_MINUS)) sWState.bMINUS = ON_CHAR; 
        if (IS_PRESSED(wm, WIIMOTE_BUTTON_PLUS))  sWState.bPLUS = ON_CHAR; 
        if (IS_PRESSED(wm, WIIMOTE_BUTTON_ONE))   sWState.bONE = ON_CHAR; 
        if (IS_PRESSED(wm, WIIMOTE_BUTTON_TWO))   sWState.bTWO = ON_CHAR; 
        if (IS_PRESSED(wm, WIIMOTE_BUTTON_HOME))  sWState.bHOME = ON_CHAR; 
    }
    for(int i=0;i<11;i++){
        if(((char*)&sWState)[i] != ((char*)&sWPrevState)[i]){
            if(((char*)&sWState)[i]==ON_CHAR)
                ((char*)&sWEventState)[i] = 'x';
            else
                ((char*)&sWEventState)[i] = '.';
            sGotEvent = true;
        }
    }
    memcpy(&sWPrevState,&sWState,sizeof(WiimoteState));
    

}

void WiimoteServerThread::handle_ctrl_status(struct wiimote_t* wm) {
    printf("\n\n--- CONTROLLER STATUS [wiimote id %i] ---\n", wm->unid);

    printf("attachment:      %i\n", wm->exp.type);
    printf("speaker:         %i\n", WIIUSE_USING_SPEAKER(wm));
    printf("ir:              %i\n", WIIUSE_USING_IR(wm));
    printf("leds:            %i %i %i %i\n", WIIUSE_IS_LED_SET(wm, 1), WIIUSE_IS_LED_SET(wm, 2), WIIUSE_IS_LED_SET(wm, 3), WIIUSE_IS_LED_SET(wm, 4));
    printf("battery:         %f %%\n", wm->battery_level);
    printf("\n");
}

void WiimoteServerThread::handle_disconnect(struct wiimote_t* wm) {
    printf("Wiimote disconected: exiting...\n");  
    sDisconnected = true;
}
