
/* @file       EOlist.c
	@brief      This file implements internal implementation of a list object.
	@author     marco.accame@iit.it
    @date       08/03/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOlist.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOlist_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

#if 0
EO_static_inline void s_eo_list_default_clear(void *p, uint16_t size)
{
     memset(p, 0, size);
}
#else
EO_static_inline void s_eo_list_default_clear(void *p, uint16_t size)
{
}    
#endif

EO_static_inline void* s_eo_list_get_data(EOlist *list, EOlistIter *li)
{
    if(list->item_size > sizeof(void*))
    {
        return(li->data);
    }
    else
    {
        return(&(li->data));
    }
}

static EOlistIter * s_eo_list_push_front(EOlistIter *head, EOlistIter *li);
static EOlistIter * s_eo_list_push_back(EOlistIter *tail, EOlistIter *li);
static EOlistIter * s_eo_list_insert_before(EOlistIter *head, EOlistIter *iter, EOlistIter *li);
static EOlistIter * s_eo_list_rem_front(EOlistIter *head);
static EOlistIter * s_eo_list_rem_back(EOlistIter *tail);
//static EOlistIter * s_eo_list_rem_iter(EOlistIter *head, EOlistIter *li);
static void s_eo_list_rem_any(EOlist *list, EOlistIter *li);
static EOlistIter * s_eo_list_front(EOlist *list);
static EOlistIter * s_eo_list_back(EOlist *list);
static EOlistIter * s_eo_list_iter_next(EOlistIter *li);
static EOlistIter * s_eo_list_iter_prev(EOlistIter *li);

static void s_eo_list_copy_item_into_iterator(EOlist *list, EOlistIter *li, void *p);
static void s_eo_list_clean_iterator(EOlist *list, EOlistIter *li);




// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOlist";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern EOlist* eo_list_New(eOsizeitem_t item_size, eOsizecntnr_t capacity, 
                           eOres_fp_voidp_uint32_t item_init, uint32_t init_par,
                           eOres_fp_voidp_voidp_t item_copy, eOres_fp_voidp_t item_clear)
{
    EOlist *retptr = NULL;
    eOsizecntnr_t i = 0;
    EOlistIter *li = NULL;    


    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOlist), 1);

    
    // now the obj has valid memory. i need to initialise it with user-defined data,
    retptr->head            = NULL;
    retptr->tail            = NULL;
    retptr->size            = 0;

    eo_errman_Assert(eo_errman_GetHandle(), (0 != item_size), s_eobj_ownname, "item_size is zero");
    eo_errman_Assert(eo_errman_GetHandle(), (0 != capacity), s_eobj_ownname, "capacity is zero");

    retptr->max_items       = capacity;
    retptr->item_size       = item_size;
    retptr->item_copy       = item_copy;
    retptr->item_clear      = item_clear;
    
     
    for(i=0; i<capacity; i++) 
    {
        li = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOlist), 1);
        // now we allocate memory for storing the items with size item_size. 
        // however, if item_size is smaller/equal to teh size of a void* (<=4 in 32-bit arch), then we use the value of data to 
        // store the item directly instead of allocating extra memory .... 
        if(item_size > sizeof(void*))
        {   // normal mode: the .data field contains a pointer to the actual data
            li->data = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, item_size, 1);
            
            if(NULL != item_init)
            {
                item_init(li->data, init_par);
            }
        }
        else
        {   // compact mode: the bytes of the .data field contain the data itself. 
            if(NULL != item_init)
            {   // thus, item_init() accepts the pointer to .data
                item_init(&(li->data), init_par);
            }
        }
        
        retptr->freeiters = s_eo_list_push_front(retptr->freeiters, li);
    }    

    return(retptr);
}



extern eOsizecntnr_t eo_list_Size(EOlist *list) 
{
    if(NULL == list) 
    {
        return(0);    
    }
    
    return(list->size);        
}


extern eOsizecntnr_t eo_list_Capacity(EOlist *list) 
{
    if(NULL == list)
    {
        return(0);    
    }
    
    return(list->max_items);    
}


extern eObool_t eo_list_Empty(EOlist *list) 
{
    if(NULL == list) 
    {
        return(eobool_true);    
    }
    
    return((0 == list->size) ? (eobool_true) : (eobool_false));        
}


extern eObool_t eo_list_Full(EOlist *list) 
{
    if(NULL == list) 
    {
        return(eobool_true);    
    }
    
    return((list->max_items == list->size) ? (eobool_true) : (eobool_false));        
}


extern void eo_list_PushFront(EOlist *list, void *p) 
{
    EOlistIter *tmpiter = NULL;
    
    if((NULL == list) || (NULL == p)) 
    {
        return;    
    }
    
    if(list->max_items == list->size) 
    { 
        // list is full
        return;
    }
    
    // get the first free iter
    tmpiter = list->freeiters;

    if(NULL != tmpiter) 
    {
        // i remove it from front of the free list
        list->freeiters = s_eo_list_rem_front(list->freeiters);

        // copy the passed obj inside the iter or store it directly if size is small
        s_eo_list_copy_item_into_iterator(list, tmpiter, p);
        
        // if it is the first element in the list, set the tail.
        if(0 == list->size) 
        {
            list->tail = tmpiter;
        }

        // insert the iter in front of the head
        list->head = s_eo_list_push_front(list->head, tmpiter);

        // increment size of the list    
        list->size ++;
    }
    
    return; 
}


extern void * eo_list_Front(EOlist *list) 
{
    EOlistIter *li = NULL;
    void *ret = NULL;
    
    if(NULL == list) 
    {
        return(NULL);
    }
    
    li = s_eo_list_front(list);
    
    if(NULL == li) 
    {
        return(NULL);    
    }

    ret = s_eo_list_get_data(list, li);
   
    return(ret);         
}


extern void eo_list_PushBack(EOlist *list, void *p) 
{
    EOlistIter *tmpiter = NULL;
    
    if((NULL == list) || (NULL == p)) 
    {
        return;    
    }
    
    if(list->max_items == list->size) 
    { 
        // list is full
        return;
    }
    
    // get the first free iter
    tmpiter = list->freeiters;

    if(NULL != tmpiter) 
    {
        // i remove it from front of the free list
        list->freeiters = s_eo_list_rem_front(list->freeiters);

        // copy the passed obj inside the iter or store it directly if size is small
        s_eo_list_copy_item_into_iterator(list, tmpiter, p);
        
        // if it is the first element in the list, set the head.
        if(0 == list->size) 
        {
             list->head = tmpiter;
        }

        // insert the iter after the tail
        list->tail = s_eo_list_push_back(list->tail, tmpiter);

        // increment size of the list    
        list->size ++;
    }
    
    return; 
}


extern void * eo_list_Back(EOlist *list) 
{
    EOlistIter *tmpiter = NULL;
    void *ret = NULL;
    
    if(NULL == list) 
    {
        return(NULL);
    }
    
    tmpiter = s_eo_list_back(list);
    
    if(NULL == tmpiter) 
    {
        return(NULL);    
    }

    ret = s_eo_list_get_data(list, tmpiter);
    
    return(ret);         
}


extern void eo_list_Insert(EOlist *list, EOlistIter *li, void *p)
{
    EOlistIter *tmpiter = NULL;
    
    if((NULL == list) || (NULL == li)) 
    {
        return;    
    }

    if(list->max_items == list->size) 
    { 
        // list is full
        return;
    }

    // get the first free iter so that we can place p in that
    tmpiter = list->freeiters;

    if(NULL != tmpiter) 
    {
        // i remove it from front of the free list
        list->freeiters = s_eo_list_rem_front(list->freeiters);
        // copy the passed obj inside the iter tmpiter or store it directly if size is small
        s_eo_list_copy_item_into_iterator(list, tmpiter, p);
        
        // if it is the first element in the list, set the tail.
        if(0 == list->size) 
        {
             list->tail = tmpiter;
        }
        // insert the element tmpiter in front of the iter li
        list->head = s_eo_list_insert_before(list->head, li, tmpiter);
        // increment size of the list    
        list->size ++;
    }
   
    return;
}


extern void * eo_list_At(EOlist *list, EOlistIter *li) 
{
    void *ret = NULL;
    
    if((NULL == list) || (NULL == li)) 
    {
        return(NULL);    
    }

    ret = s_eo_list_get_data(list, li);

    return(ret);
}


extern EOlistIter* eo_list_Begin(EOlist *list) 
{
    return((NULL == list) ? (NULL) :(s_eo_list_front(list)));         
}
 

extern EOlistIter* eo_list_Next(EOlist *list, EOlistIter *li) 
{
    // next in list is simple. if li belongs to list, it is enough to get next of li.
    // we dont do check that li belong to list, because it can be heavy to do.
    if(NULL == list)
    {
        return(NULL);
    }
    return(s_eo_list_iter_next(li));         
    
}


extern EOlistIter* eo_list_Prev(EOlist *list, EOlistIter *li) 
{
    // previous in list can be dangerous also if li belongs to list.
    // if li is head, .... what is prev ?? NULL.
    // we dont do check that li belong to list, because it can be heavy to do. 
    if(NULL == list)
    {
        return(NULL);
    }
    return(s_eo_list_iter_prev(li));         
}


extern EOlistIter* eo_list_FindItem(EOlist *list, void *p) 
{
    EOlistIter *tmpiter = NULL;
    const void* target = (const void*)p;
    void* data = NULL;

    if((NULL == list) || (NULL == p)) 
    {
         return(NULL);
    }
    
    // i navigate from beginning to end until i find a NULL pointer or i break
    for(tmpiter = s_eo_list_front(list); NULL != tmpiter; tmpiter = s_eo_list_iter_next(tmpiter)) 
    {
        data = s_eo_list_get_data(list, tmpiter);
        // data is a pointer to what is contained inside the list.

        if(0 == memcmp(data, target, list->item_size))
        {
            break;
        }

    }
    
    return(tmpiter);    
}


/* @fn         extern EOlistIter* eo_list_FindInside(EOlist *list, uint32_t target, uint32_t (get_value_from_item)(void *item))
    @brief      Finds the iterator which contains the object which matches the target by means of function get_target(). 
    @param      list            Pointer to the EOlist object.
    @param      target          The target.
    @param      get_value_from_item   The function which gets the value to be compared with @e target
    @return     The iterator (or NULL if list is NULL or empty / @e p is not in the list / @e p 
                is NULL).
 **/
//extern EOlistIter* eo_list_FindInside(EOlist *list, uint32_t target, uint32_t (get_value_from_item)(void *item));

//extern EOlistIter* eo_list_FindInside(EOlist *list, uint32_t target, uint32_t (get_value_from_item)(void *item)) 
//{
//    EOlistIter *tmpiter = NULL;
//    void* data = NULL;
//
//    if((NULL == list) || (NULL == get_value_from_item)) 
//    {
//         return(NULL);
//    }
//    
//    
//    // i navigate from beginning to end until i find a NULL pointer or i break
//    for(tmpiter = s_eo_list_front(list); NULL != tmpiter; tmpiter = s_eo_list_iter_next(tmpiter)) 
//    {
//        data = s_eo_list_get_data(list, tmpiter);
//
//        if(target == get_value_from_item(data)) 
//        {
//            break;
//        }
//
//    }
//    
//    return(tmpiter);    
////}


extern EOlistIter* eo_list_Find(EOlist *list, eOresult_t (matching_rule)(void *item, void *param), void *param)
{
    EOlistIter *tmpiter = NULL;
    void* data = NULL;

    if((NULL == list) || (NULL == matching_rule)) 
    {
         return(NULL);
    }
    
    // i navigate from beginning to end until i find a NULL pointer or i break
    for(tmpiter = s_eo_list_front(list); NULL != tmpiter; tmpiter = s_eo_list_iter_next(tmpiter)) 
    {
        data = s_eo_list_get_data(list, tmpiter);

        if(eores_OK == matching_rule(data, param)) 
        {
            break;
        }

    }
    
    return(tmpiter);   

}

extern void eo_list_ForEach(EOlist *list, void (execute)(void *item, void *param), void *param)
{
    EOlistIter *tmpiter = NULL;
    void* data = NULL;

    if((NULL == list) || (NULL == execute)) 
    {
         return;
    }
    
    // i navigate from beginning to end until i find a NULL pointer
    for(tmpiter = s_eo_list_front(list); NULL != tmpiter; tmpiter = s_eo_list_iter_next(tmpiter)) 
    {
        data = s_eo_list_get_data(list, tmpiter);
        execute(data, param);
    }
    
    return; 
}


extern void eo_list_FromIterForEach(EOlist *list, EOlistIter *li, void (execute)(void *item, void *param), void *param)
{
    EOlistIter *tmpiter = NULL;
    void* data = NULL;

    if((NULL == list) || (NULL == li) || (NULL == execute)) 
    {
         return;
    }
    
    // i navigate from li to end until i find a NULL pointer
    for(tmpiter = li; NULL != tmpiter; tmpiter = s_eo_list_iter_next(tmpiter)) 
    {
        data = s_eo_list_get_data(list, tmpiter);
        execute(data, param);
    }
    
    return; 
}


extern eObool_t eo_list_IsIterInside(EOlist *list, EOlistIter *li)
{
    EOlistIter *tmpiter = NULL;

    if((NULL == list) || (NULL == li)) 
    {
         return(eobool_false);
    }
    
    // i navigate from beginning to end until we find li pointer or we return
    for(tmpiter = s_eo_list_front(list); NULL != tmpiter; tmpiter = s_eo_list_iter_next(tmpiter)) 
    {
        if(li == tmpiter) 
        {
            return(eobool_true);
        }
    }
    
    return(eobool_false);    
}


extern void eo_list_PopFront(EOlist *list) 
{
    EOlistIter *tmpiter = NULL;
            
    if(NULL == list) 
    {
        return;    
    }
    
    // get the first iter of list
    tmpiter = s_eo_list_front(list);
    
    if(NULL != tmpiter) 
    {
        // i remove it from front of the list
        list->head = s_eo_list_rem_front(list->head);

        // i clean it 
        s_eo_list_clean_iterator(list, tmpiter);

        // and i put tmpiter back into the free iters
        list->freeiters = s_eo_list_push_front(list->freeiters, tmpiter);

        // finally, i decrement size of list
        list->size --;

        // if there are no more elements in the list, set the tail to NULL
        if(0 == list->size) 
        {
             list->tail = NULL;
        }
    }
    
    return; 
}


extern void eo_list_PopBack(EOlist *list) 
{
    EOlistIter *tmpiter = NULL;

            
    if(NULL == list) 
    {
        return;    
    }
    
    // get the last iter of list
    tmpiter = s_eo_list_back(list);
    
    if(NULL != tmpiter) 
    {
        // i remove it from end of the list
        list->tail = s_eo_list_rem_back(list->tail);

        // i clean it 
        s_eo_list_clean_iterator(list, tmpiter);

        // and i put tmpiter back into the free iters
        list->freeiters = s_eo_list_push_front(list->freeiters, tmpiter);

        // finally, i decrement size of list
        list->size --;

        // if there are no more elements in the list, set the head to NULL
        if(0 == list->size) 
        {
             list->head = NULL;
        }
    }
    
    return; 
}


extern void eo_list_Erase(EOlist *list, EOlistIter *li) 
{
    EOlistIter *tmp = NULL;
    
    if((NULL == list) || (NULL == li)) 
    {
        return;    
    }

    // extra safety
    if(eobool_false == eo_list_IsIterInside(list, li)) 
    {
         return;
    }
    
    // ok, the iter li exists in the list, thus i can safely remove it.

    // get the first iter of list, the head
    tmp = s_eo_list_front(list);

    if(NULL != tmp) 
    {
        // i have a valid head and a valid list iter li, thus i can i remove it
        //list->head = s_eo_list_rem_iter(list->head, li);
        s_eo_list_rem_any(list, li);

        // i clean it
        s_eo_list_clean_iterator(list, li);

        // and i put li back into the free iters
        list->freeiters = s_eo_list_push_front(list->freeiters, li);

        // finally, i decrement size of list
        list->size --;

        // if there are no more elements in the list, set the tail to NULL
        if(0 == list->size) 
        {
             list->tail = NULL;
        }
    }
    
    return;
}


extern void eo_list_Clear(EOlist *list)
{
    if(NULL == list) 
    {
        return;    
    }

    while(NULL != s_eo_list_front(list))
    {
        eo_list_PopFront(list);
    }
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


// returns the head
static EOlistIter * s_eo_list_push_front(EOlistIter *head, EOlistIter *li) 
{
    EOlistIter *oldhead = head;
    
    li->prev = NULL;
    li->next = head;
    
    head = li;
    
    if(NULL != oldhead) 
    {
        oldhead->prev = head;
    }
    
    return(head);
}


// returns teh tail
static EOlistIter * s_eo_list_push_back(EOlistIter *tail, EOlistIter *li) 
{
    EOlistIter *oldtail = tail;
    
    li->prev = tail;
    li->next = NULL;
    
    tail = li;
    
    if(NULL != oldtail) 
    {
        oldtail->next = tail;
    }
    
    return(tail);
}


// returns the head
static EOlistIter * s_eo_list_insert_before(EOlistIter *head, EOlistIter *iter, EOlistIter *li) 
{
    
    if(iter == head)
    {
        return(s_eo_list_push_front(head, li));
    }
    else
    {
        // iter is not null, if iter is not the head, then ... also iter->prev is not NULL
        // pre is teh node before iter
        EOlistIter *pre = iter->prev;

        // fix the links of li
        li->prev = pre;
        li->next = iter;

        // fix the link of pre
        pre->next = li;

        // fix the link of iter
        iter->prev = li;

        // the head remains the same
        return(head);
    }

}

 
// returns the head
static EOlistIter * s_eo_list_rem_front(EOlistIter *head) 
{
    EOlistIter *oldhead = head;
    
    if(NULL == oldhead) 
    {
        return(NULL);    
    }
    
    if(NULL == oldhead->next) 
    {
        // only one element in current list
        return(NULL);
    } 
    else 
    {
        // at least two elements
        head = oldhead->next;
        head->prev = NULL;
    }
    

    return(head);
}


// returns the tail
static EOlistIter * s_eo_list_rem_back(EOlistIter *tail) 
{
    EOlistIter *oldtail = tail;
    
    if(NULL == oldtail) 
    {
        return(NULL);    
    }
    
    if(NULL == oldtail->prev) 
    {
        // only one element in current list
        return(NULL);
    } 
    else 
    {
        // at least two elements
        tail = oldtail->prev;
        tail->next = NULL;
    }
    

    return(tail);
}



// returns the head
//static EOlistIter * s_eo_list_rem_iter(EOlistIter *head, EOlistIter *li) 
//{
//    EOlistIter *newhead = head;
//    
//    if(NULL == head) 
//    {
//        return(NULL);    
//    }
//
//    if(NULL == li) 
//    {
//         return(newhead);
//    }
//
//    // head and li are not NULL
//    if(li == head) 
//    {
//         // li is the head, thus remove li from the front.
//        newhead = s_eo_list_rem_front(head);
//    }
//    else if(NULL == li->next) 
//    {
//        // li is the tail, thus ...
//        li->prev->next = NULL;
//    }
//    else 
//    {
//         // li is in the middle
//        li->prev->next = li->next;
//        li->next->prev = li->prev;
//    }
//
//
//    return(newhead);
//}


static void s_eo_list_rem_any(EOlist *list, EOlistIter *li) 
{
    EOlistIter *head = list->head;
//    EOlistIter *newhead = list->head;
    
    if(NULL == list->head) 
    {
        return;    
    }

    if(NULL == li) 
    {
         return;
    }

    // head and li are not NULL
    if(li == list->head) 
    {
         // li is the head, thus remove li from the front.
        list->head = s_eo_list_rem_front(head);
    }
    else if(li == list->tail) 
    {
        // li is the tail, thus ...
        list->tail = s_eo_list_rem_back(list->tail);
    }
    else 
    {
         // li is in the middle
        li->prev->next = li->next;
        li->next->prev = li->prev;
    }

}


static EOlistIter * s_eo_list_front(EOlist *list) 
{
    return(list->head);
}


static EOlistIter * s_eo_list_back(EOlist *list) 
{
    return(list->tail);
}


static EOlistIter * s_eo_list_iter_next(EOlistIter *li) 
{
    if(NULL == li) 
    {
         return(NULL);
    }

    return(li->next);
}  


static EOlistIter * s_eo_list_iter_prev(EOlistIter *li) 
{
    if(NULL == li) 
    {
         return(NULL);
    }

    return(li->prev);
}  


static void s_eo_list_copy_item_into_iterator(EOlist *list, EOlistIter *li, void *p)
{
    void* data = s_eo_list_get_data(list, li);

    if(NULL != list->item_copy)
    {
        list->item_copy(data, p);
    }
    else
    {
        memcpy(data, p, list->item_size);
    }

}


static void s_eo_list_clean_iterator(EOlist *list, EOlistIter *li)
{
    void* data = s_eo_list_get_data(list, li);

    // call its dtor
    if(NULL != list->item_clear)
    {
        list->item_clear(data);
    }
    else
    {
        s_eo_list_default_clear(data, list->item_size);
    }

}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



