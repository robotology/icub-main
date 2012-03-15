
/* @file       test-eocontainers.c
	@brief      This file implements a test for embobj
	@author     marco.accame@iit.it
    @date       06/21/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"

// embobj
#include "EoCommon.h"
#include "EOdeque.h"
#include "EOlist.h"
#include "EOfifo.h"
#include "EOfifoByte.h"
#include "EOfifoWord.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------


 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "test-eocontainers.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------

typedef struct
{
    uint32_t field32;
    uint8_t  field8;
    uint16_t field16;
} myTy0_t;



typedef struct
{
    uint32_t field32;
    uint8_t  field8;
    uint16_t field16;
    uint64_t field64;
} myTy1_t;

typedef struct
{
    uint8_t f0;
    uint8_t f1;
    uint8_t f2;
    uint8_t f3;
} myTy4B_t;

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

eOresult_t myInit0(void *p, uint32_t a);
eOresult_t myCopy0(void *des, void *ori);
eOresult_t myClear0(void *p);

eOresult_t myInit4(void *p, uint32_t a);
eOresult_t myCopy4(void *des, void *ori);
eOresult_t myClear4(void *p);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


static EOdeque *dek0a = NULL;
static EOdeque *dek0b = NULL;
static EOdeque *dek4b = NULL;


static EOlist *lis0a = NULL;
static EOlist *lis0b = NULL;
static EOlist *lis4b = NULL;


static EOfifo *fifo0a = NULL;
static EOfifo *fifo0b = NULL;
static EOfifo *fifo4b = NULL;


static EOfifoByte *fifobyte = NULL;

static EOfifoWord *fifoword = NULL;

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

void test_eocont_EOdeque(void)
{
    myTy0_t myt0 = {0, 1, 2};
    myTy0_t *pm0 = NULL;
//    myTy0_t mmt;
    eOsizecntnr_t nitems = 0;

    myTy4B_t myt4b = {0, 1, 2, 3};
    myTy4B_t *pm4 = NULL;


    dek0a = eo_deque_New(sizeof(myTy0_t), 10, NULL, 0, NULL, NULL);
    eo_deque_PushBack(dek0a, &myt0);
    eo_deque_PushBack(dek0a, &myt0);
    pm0 = (myTy0_t*) eo_deque_Front(dek0a);
    pm0 = pm0;
    eo_deque_PopFront(dek0a);
    nitems = eo_deque_Size(dek0a);
    nitems = nitems;


    dek0b = eo_deque_New(sizeof(myTy0_t), 10, myInit0, 55, myCopy0, myClear0);
    eo_deque_PushBack(dek0b, &myt0);
    eo_deque_PushBack(dek0b, &myt0);
    pm0 = (myTy0_t*) eo_deque_Front(dek0b);
    eo_deque_PopFront(dek0b);
    nitems = eo_deque_Size(dek0b);


    dek4b = eo_deque_New(sizeof(myTy4B_t), 10, myInit4, 66, myCopy4, myClear4);
    eo_deque_PushBack(dek4b, &myt4b);
    eo_deque_PushBack(dek4b, &myt4b);
    pm4 = (myTy4B_t*) eo_deque_Front(dek4b);
    pm4 = pm4;
    eo_deque_PopFront(dek4b);
    nitems = eo_deque_Size(dek4b);
}

eOresult_t s_itembigger_than(void *item, void *param)
{
    myTy0_t *p = (myTy0_t*)item;
    uint32_t v = *((uint32_t*)param);

    if(p->field32 > v)
    {
        return(eores_OK);
    }
    else
    {
        return(eores_NOK_generic);
    }
}

eOresult_t s_itemequal_to(void *item, void *param)
{
    myTy0_t *p = (myTy0_t*)item;
    uint32_t v = *((uint32_t*)param);

    if(p->field32 == v)
    {
        return(eores_OK);
    }
    else
    {
        return(eores_NOK_generic);
    }
}


void test_print_list(EOlist *list)
{
    EOlistIter *li = eo_list_Begin(list);
    uint8_t i = 0;
    myTy0_t *pm0 = NULL;

    printf("\n value of list[%d] is: [", eo_list_Size(list));
    for(i=0; i<eo_list_Size(list); i++)
    {
        pm0 = (myTy0_t*) eo_list_At(list, li);
        printf("%d ", pm0->field32);
        li = eo_list_Next(list, li);
    }
    printf("]\n");
}

void test_eocont_EOlist(void)
{
    EOlistIter *li = NULL;
    uint32_t val = 0;
    myTy0_t myt0 = {0, 0, 0};
    myTy0_t myt1 = {1, 1, 1};
    myTy0_t myt2 = {2, 2, 2};
    myTy0_t myt3 = {3, 3, 3};
    myTy0_t myt4 = {4, 4, 4};
    myTy0_t myt5 = {5, 5, 5};
    myTy0_t *pm0 = NULL;
//    myTy0_t mmt;
    eOsizecntnr_t nitems = 0;

    myTy4B_t myt4b = {0, 1, 2, 3};
    myTy4B_t *pm4 = NULL;


    lis0a = eo_list_New(sizeof(myTy0_t), 10, NULL, 0, NULL, NULL);


    // add 3, 2, 5, 1, 4 so that they are in ascending order
    {
        li = eo_list_Find(lis0a, s_itembigger_than, &myt3.field32);
        if(0)
        {
            eo_list_PushBack(lis0a, &myt3);
        }
        else 
        if(NULL == li)
        {
            eo_list_PushBack(lis0a, &myt3);
        }
        else
        {
            eo_list_Insert(lis0a, li, &myt3);
        }

        test_print_list(lis0a);

        li = eo_list_Find(lis0a, s_itembigger_than, &myt2.field32);
        
        if(0)
        {
            eo_list_PushBack(lis0a, &myt2);
        }
        else 
        if(NULL == li)
        {
            eo_list_PushBack(lis0a, &myt2);
        }
        else
        {
            eo_list_Insert(lis0a, li, &myt2);
        }

        test_print_list(lis0a);

        li = eo_list_Find(lis0a, s_itembigger_than, &myt5.field32);
        if(0)
        {
            eo_list_PushBack(lis0a, &myt5);
        }        
        else 
        if(NULL == li)
        {
            eo_list_PushBack(lis0a, &myt5);
        }
        else
        {
            eo_list_Insert(lis0a, li, &myt5);
        }

        test_print_list(lis0a);

        li = eo_list_Find(lis0a, s_itembigger_than, &myt1.field32);
        if(0)
        {
            eo_list_PushBack(lis0a, &myt1);
        }        
        else 
        if(NULL == li)
        {
            eo_list_PushBack(lis0a, &myt1);
        }
        else
        {
            eo_list_Insert(lis0a, li, &myt1);
        }

        test_print_list(lis0a);

        li = eo_list_Find(lis0a, s_itembigger_than, &myt4.field32);
        if(0)
        {
            eo_list_PushBack(lis0a, &myt4);
        }
        else
        if(NULL == li)
        {
            eo_list_PushBack(lis0a, &myt4);
        }
        else
        {
            eo_list_Insert(lis0a, li, &myt4);
        }

        test_print_list(lis0a);


        li = eo_list_Find(lis0a, s_itemequal_to, &myt3.field32);
        if(NULL != li)
        {                     
            eo_list_Erase(lis0a, li);
        }
        test_print_list(lis0a);

        li = eo_list_Find(lis0a, s_itemequal_to, &myt1.field32);
        if(NULL != li)
        {                     
            eo_list_Erase(lis0a, li);
        }
        test_print_list(lis0a);

        li = eo_list_Find(lis0a, s_itemequal_to, &myt5.field32);
        if(NULL != li)
        {                     
            eo_list_Erase(lis0a, li);
        }
        test_print_list(lis0a);

        li = eo_list_Find(lis0a, s_itemequal_to, &myt4.field32);
        if(NULL != li)
        {                     
            eo_list_Erase(lis0a, li);
        }
        test_print_list(lis0a);

        li = eo_list_Find(lis0a, s_itemequal_to, &myt2.field32);
        if(NULL != li)
        {                     
            eo_list_Erase(lis0a, li);
        }
        test_print_list(lis0a);


        li = eo_list_Find(lis0a, s_itemequal_to, &myt2.field32);
        if(NULL != li)
        {                     
            eo_list_Erase(lis0a, li);
        }
        test_print_list(lis0a);

    }
    // remove 2, 5, 1 and check the ascending order
    {

    }










    val = 1;
    li = eo_list_Find(lis0a, s_itembigger_than, &val);

    eo_list_PushFront(lis0a, &myt0);
    li = eo_list_Find(lis0a, s_itembigger_than, &val);

    eo_list_PushFront(lis0a, &myt1);
    li = eo_list_Find(lis0a, s_itembigger_than, &val);

    eo_list_PushFront(lis0a, &myt2);
    li = eo_list_Find(lis0a, s_itembigger_than, &val);

    eo_list_PushFront(lis0a, &myt1);
    li = eo_list_Find(lis0a, s_itembigger_than, &val);

    pm0 = (myTy0_t*) eo_list_Front(lis0a);
    pm0 = pm0;
    eo_list_PopFront(lis0a);
    nitems = eo_list_Size(lis0a);
    nitems = nitems;


    lis0b = eo_list_New(sizeof(myTy0_t), 10, myInit0, 55, myCopy0, myClear0);
    eo_list_PushFront(lis0b, &myt0);
    eo_list_PushFront(lis0b, &myt0);
    pm0 = (myTy0_t*) eo_list_Front(lis0b);
    eo_list_PopFront(lis0b);
    nitems = eo_list_Size(lis0b);


    lis4b = eo_list_New(sizeof(myTy4B_t), 10, myInit4, 66, myCopy4, myClear4);
    eo_list_PushFront(lis4b, &myt4b);
    eo_list_PushFront(lis4b, &myt4b);
    pm4 = (myTy4B_t*) eo_list_Front(lis4b);
    pm4 = pm4;
    eo_list_PopFront(lis4b);
    nitems = eo_list_Size(lis4b);
}



void test_eocont_EOfifo(void)
{
    myTy0_t myt0 = {0, 1, 2};
    myTy0_t *pm0 = NULL;
//    myTy0_t mmt;
    eOsizecntnr_t nitems = 0;

    myTy4B_t myt4b = {0, 1, 2, 3};
    myTy4B_t *pm4 = NULL;

    pm4 = pm4;


    fifo0a = eo_fifo_New(sizeof(myTy0_t), 10, NULL, 0, NULL, NULL, NULL);
    eo_fifo_Put(fifo0a, &myt0, eok_reltimeINFINITE);
    eo_fifo_Put(fifo0a, &myt0, eok_reltimeINFINITE);
    eo_fifo_Get(fifo0a, (const void**)&pm0, eok_reltimeINFINITE);
    eo_fifo_Rem(fifo0a, eok_reltimeINFINITE);
    eo_fifo_Size(fifo0a, &nitems, eok_reltimeINFINITE);
    nitems = nitems;


    fifo0b = eo_fifo_New(sizeof(myTy0_t), 10, myInit0, 55, myCopy0, myClear0, NULL);
    eo_fifo_Put(fifo0b, &myt0, eok_reltimeINFINITE);
    eo_fifo_Put(fifo0b, &myt0, eok_reltimeINFINITE);
    eo_fifo_Get(fifo0b, (const void**)&pm0, eok_reltimeINFINITE);
    eo_fifo_Rem(fifo0b, eok_reltimeINFINITE);
    eo_fifo_Size(fifo0b, &nitems, eok_reltimeINFINITE);


    fifo4b = eo_fifo_New(sizeof(myTy4B_t), 10, myInit4, 66, myCopy4, myClear4, NULL);
    eo_fifo_Put(fifo4b, &myt4b, eok_reltimeINFINITE);
    eo_fifo_Put(fifo4b, &myt4b, eok_reltimeINFINITE);
    eo_fifo_Get(fifo4b, (const void**)&pm0, eok_reltimeINFINITE);
    eo_fifo_Rem(fifo4b, eok_reltimeINFINITE);
    eo_fifo_Size(fifo4b, &nitems, eok_reltimeINFINITE);
}

void test_eocont_EOfifoByte(void)
{

    eOsizecntnr_t nitems = 0;

    uint8_t byte0 = 1;
    uint8_t byte = 0;


    fifobyte = eo_fifobyte_New(10, NULL);
    eo_fifobyte_Put(fifobyte, byte0, eok_reltimeINFINITE);
    eo_fifobyte_Put(fifobyte, byte0, eok_reltimeINFINITE);
    eo_fifobyte_Get(fifobyte, &byte, eok_reltimeINFINITE);
    eo_fifobyte_Rem(fifobyte, eok_reltimeINFINITE);
    eo_fifobyte_Size(fifobyte, &nitems, eok_reltimeINFINITE);

}


void test_eocont_EOfifoWord(void)
{

    eOsizecntnr_t nitems = 0;

    uint32_t word0 = 1;
    uint32_t word = 0;


    fifoword = eo_fifoword_New(10, NULL);
    eo_fifoword_Put(fifoword, word0, eok_reltimeINFINITE);
    eo_fifoword_Put(fifoword, word0, eok_reltimeINFINITE);
    eo_fifoword_Get(fifoword, &word, eok_reltimeINFINITE);
    eo_fifoword_Rem(fifoword, eok_reltimeINFINITE);
    eo_fifoword_Size(fifoword, &nitems, eok_reltimeINFINITE);

}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

  


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

eOresult_t myInit0(void *p, uint32_t a)
{
    myTy0_t *t0 = (myTy0_t*)p;
    
    t0->field32 = a;

    return(eores_OK);
}

eOresult_t myCopy0(void *des, void *ori)
{
    myTy0_t *t0d = (myTy0_t*)des;
    myTy0_t *t0o = (myTy0_t*)ori;

    memcpy(t0d, t0o, sizeof(myTy0_t));
    t0d->field8 = 7;

    return(eores_OK);
}

eOresult_t myClear0(void *p)
{
    myTy0_t *t0 = (myTy0_t*)p;

    memset(t0, 0, sizeof(myTy0_t));

    return(eores_OK);
}



eOresult_t myInit4(void *p, uint32_t a)
{
    myTy4B_t *t4 = (myTy4B_t*)p;
    
    t4->f0 = a;

    return(eores_OK);
}

eOresult_t myCopy4(void *des, void *ori)
{
    myTy4B_t *t4d = (myTy4B_t*)des;
    myTy4B_t *t4o = (myTy4B_t*)ori;

    memcpy(t4d, t4o, sizeof(myTy4B_t));
    t4d->f1 = 7;

    return(eores_OK);
}

eOresult_t myClear4(void *p)
{
    myTy4B_t *t4 = (myTy4B_t*)p;

    memset(t4, 0, sizeof(myTy4B_t));

    return(eores_OK);
}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



