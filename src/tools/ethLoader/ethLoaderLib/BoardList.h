/*
 * Copyright (C) 2012 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ETHUPDATER_BOARDLIST_H__
#define __ETHUPDATER_BOARDLIST_H__

#include "BoardInfo.h"

#include <vector>
using namespace std;

class BoardList
{
public:
    BoardList()
    {
        mnBoards=0;

        for (int i=0; i<256; ++i)
        {
            mpBoards[i]=NULL;
        }
    }

    virtual ~BoardList()
    {
        empty();
    }

    int size(){ return mnBoards; }

    int numSelected()
    {
        int n=0;
        for (int i=0; i<mnBoards; ++i)
        {
            if (mpBoards[i])
            {
                if (mpBoards[i]->mSelected) ++n;
            }
        }

        return n;
    }

    void empty()
    {
        for (int i=0; i<mnBoards; ++i)
        {
            if (mpBoards[i])
            {
                delete mpBoards[i];
                mpBoards[i]=NULL;
            }
        }

        mnBoards=0;
    }

    bool addBoard(BoardInfo *pBoard)
    {
        if (mnBoards>=256) return false;

        mpBoards[mnBoards++]=pBoard;

        return true;
    }

    vector<BoardInfo *> getBoards(ACE_UINT32 address)
    {
        vector<BoardInfo *> ret;
        ret.resize(0);

        for(int i=0; i<mnBoards; i++)
        {
            bool found = false;
            if(0 == address)
            {   // searching for the selected boards
                if(true == mpBoards[i]->mSelected)
                {
                    found = true;
                }
            }
            else if(mpBoards[i]->mAddress == address)
            {   // searching for a given ip address
                found = true;
            }

            if(true == found)
            {
                ret.push_back(mpBoards[i]);
            }
        }

        return ret;
    }


    bool replaceBoard(BoardInfo *pBoard)
    {
        // search for the board with the same mac

        for(int i=0; i<mnBoards; i++)
        {
            if(mpBoards[i]->mMac == pBoard->mMac)
            {
                // found ...
                delete mpBoards[i];
                mpBoards[i] = pBoard;
                return true;
            }
        }

        delete pBoard;

        return true;
    }

    void selectAll(bool sel)
    {
        for (int i=0; i<mnBoards; ++i)
        {
            if (mpBoards[i])
            {
                mpBoards[i]->mSelected=sel;
            }
        }
    }

    BoardInfo& operator[](int i){ return *mpBoards[i]; }

protected:
    int mnBoards;
    BoardInfo *mpBoards[256];
};

#endif
