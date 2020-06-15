/******************************************************************************/
/*                                                                            */
/*    Copyright (c) 2013-2015, Kyu-Young Whang, KAIST                         */
/*    All rights reserved.                                                    */
/*                                                                            */
/*    Redistribution and use in source and binary forms, with or without      */
/*    modification, are permitted provided that the following conditions      */
/*    are met:                                                                */
/*                                                                            */
/*    1. Redistributions of source code must retain the above copyright       */
/*       notice, this list of conditions and the following disclaimer.        */
/*                                                                            */
/*    2. Redistributions in binary form must reproduce the above copyright    */
/*       notice, this list of conditions and the following disclaimer in      */
/*       the documentation and/or other materials provided with the           */
/*       distribution.                                                        */
/*                                                                            */
/*    3. Neither the name of the copyright holder nor the names of its        */
/*       contributors may be used to endorse or promote products derived      */
/*       from this software without specific prior written permission.        */
/*                                                                            */
/*    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     */
/*    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       */
/*    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS       */
/*    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE          */
/*    COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,    */
/*    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,    */
/*    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;        */
/*    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER        */
/*    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT      */
/*    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN       */
/*    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE         */
/*    POSSIBILITY OF SUCH DAMAGE.                                             */
/*                                                                            */
/******************************************************************************/
/******************************************************************************/
/*                                                                            */
/*    ODYSSEUS/EduCOSMOS Educational Purpose Object Storage System            */
/*    (Version 1.0)                                                           */
/*                                                                            */
/*    Developed by Professor Kyu-Young Whang et al.                           */
/*                                                                            */
/*    Advanced Information Technology Research Center (AITrc)                 */
/*    Korea Advanced Institute of Science and Technology (KAIST)              */
/*                                                                            */
/*    e-mail: odysseus.educosmos@gmail.com                                    */
/*                                                                            */
/******************************************************************************/
/*
 * Module: edubtm_Split.c
 *
 * Description : 
 *  This file has three functions about 'split'.
 *  'edubtm_SplitInternal(...) and edubtm_SplitLeaf(...) insert the given item
 *  after spliting, and return 'ritem' which should be inserted into the
 *  parent page.
 *
 * Exports:
 *  Four edubtm_SplitInternal(ObjectID*, BtreeInternal*, Two, InternalItem*, InternalItem*)
 *  Four edubtm_SplitLeaf(ObjectID*, PageID*, BtreeLeaf*, Two, LeafItem*, InternalItem*)
 */


#include <string.h>
#include "EduBtM_common.h"
#include "BfM.h"
#include "EduBtM_Internal.h"



/*@================================
 * edubtm_SplitInternal()
 *================================*/
/*
 * Function: Four edubtm_SplitInternal(ObjectID*, BtreeInternal*,Two, InternalItem*, InternalItem*)
 *
 * Description:
 * (Following description is for original ODYSSEUS/COSMOS BtM.
 *  For ODYSSEUS/EduCOSMOS EduBtM, refer to the EduBtM project manual.)
 *
 *  At first, the function edubtm_SplitInternal(...) allocates a new internal page
 *  and initialize it.  Secondly, all items in the given page and the given
 *  'item' are divided by halves and stored to the two pages.  By spliting,
 *  the new internal item should be inserted into their parent and the item will
 *  be returned by 'ritem'.
 *
 *  A temporary page is used because it is difficult to use the given page
 *  directly and the temporary page will be copied to the given page later.
 *
 * Returns:
 *  error code
 *    some errors caused by function calls
 *
 * Note:
 *  The caller should call BfM_SetDirty() for 'fpage'.
 */
Four edubtm_SplitInternal(
    ObjectID                    *catObjForFile,         /* IN catalog object of B+ tree file */
    BtreeInternal               *fpage,                 /* INOUT the page which will be splitted */
    Two                         high,                   /* IN slot No. for the given 'item' */
    InternalItem                *item,                  /* IN the item which will be inserted */
    InternalItem                *ritem)                 /* OUT the item which will be returned by spliting */
{
	/* These local variables are used in the solution code. However, you don��t have to use all these variables in your code, and you may also declare and use additional local variables if needed. */
    Four                        e;                      /* error number */
    Two                         i;                      /* slot No. in the given page, fpage */
    Two                         j;                      /* slot No. in the splitted pages */
    Two                         k;                      /* slot No. in the new page */
    Two                         maxLoop;                /* # of max loops; # of slots in fpage + 1 */
    Four                        sum;                    /* the size of a filled area */
    Boolean                     flag=FALSE;             /* TRUE if 'item' become a member of fpage */
    PageID                      newPid;                 /* for a New Allocated Page */
    BtreeInternal               *npage;                 /* a page pointer for the new allocated page */
    Two                         fEntryOffset;           /* starting offset of an entry in fpage */
    Two                         nEntryOffset;           /* starting offset of an entry in npage */
    Two                         entryLen;               /* length of an entry */
    btm_InternalEntry           *fEntry;                /* internal entry in the given page, fpage */
    btm_InternalEntry           *nEntry;                /* internal entry in the new page, npage*/
    Boolean                     isTmp;

    e = btm_AllocPage(catObjForFile, (PageID*)&fpage->hdr.pid, &newPid);
    if(e < 0) ERR(e);

    e = btm_IsTemporary(catObjForFile, &isTmp);
    if(e < 0) ERR(e);

    e = edubtm_InitInternal(&newPid, FALSE, isTmp);
    if(e < 0) ERR(e);

    e = BfM_GetNewTrain(&newPid, (char **)&npage, PAGE_BUF);
    if(e < 0) ERR(e);

    maxLoop = fpage->hdr.nSlots + 1;
    for(sum = 0, i = 0, j = 0; j < maxLoop && sum < BI_HALF; j++){
        if(j == high + 1){
            entryLen = sizeof(ShortPageID) + ALIGNED_LENGTH(sizeof(Two) + item->klen);
            flag = TRUE;
        }else{
            fEntryOffset = fpage->slot[-i];
            fEntry = (btm_InternalEntry*)&(fpage->data[fEntryOffset]);
            entryLen = sizeof(ShortPageID) + ALIGNED_LENGTH(sizeof(Two) + fEntry->klen);
            i++;
        }
        sum += entryLen + sizeof(Two);
    }

    fpage->hdr.nSlots = i;

    for(k = -1; j < maxLoop; j++, k++){
        if(k == -1){
            nEntry = (btm_InternalEntry*)ritem;
        }else{
            nEntryOffset = npage->slot[-k] = npage->hdr.free;
            nEntry = (btm_InternalEntry*)&(npage -> data[nEntryOffset]);
        }

        if(j == high + 1){
            nEntry->spid = item->spid;
            nEntry->klen = item->klen;
            memcpy(&(nEntry->kval[0]), &(item->kval[0]), nEntry->klen);
            entryLen = sizeof(ShortPageID) + ALIGNED_LENGTH(sizeof(Two) + nEntry->klen);
        }else{
            fEntryOffset = fpage->slot[-i];
            fEntry = (btm_InternalEntry*)&(fpage->data[fEntryOffset]);
            entryLen = sizeof(ShortPageID) + ALIGNED_LENGTH(sizeof(Two) + fEntry->klen);
            memcpy((char*)nEntry, (char*)fEntry, entryLen);

            if(fEntryOffset + entryLen == fpage->hdr.free)
                fpage->hdr.free -= entryLen;
            else
                fpage->hdr.unused += entryLen;

            i++;
        }
        if(k == -1){
            npage->hdr.p0 = nEntry->spid;
            ritem->spid = newPid.pageNo;
        }else{
            npage->hdr.free += entryLen;
        }
    }

    npage->hdr.nSlots = k;

    if(flag){
        entryLen = sizeof(ShortPageID) + ALIGNED_LENGTH(sizeof(Two) + item->klen);

        if(BI_CFREE(fpage) < entryLen + sizeof(Two))
            edubtm_CompactInternalPage(fpage, NIL);
        
        for(i = fpage->hdr.nSlots - 1; i >= high + 1; i--)
            fpage->slot[-(i + 1)] = fpage->slot[-i];
        
        fEntryOffset = fpage->slot[-(high + 1)] = fpage->hdr.free;
        fEntry = (btm_InternalEntry*)&(fpage->data[fEntryOffset]);

        fEntry->spid = item->spid;
        fEntry->klen = item->klen;
        memcpy(&(fEntry->kval[0]), &(item->kval[0]), fEntry->klen);

        fpage->hdr.free += entryLen;
        fpage->hdr.nSlots++;
    }

    if(fpage->hdr.type & ROOT) fpage->hdr.type = INTERNAL;

    e = BfM_SetDirty(&newPid, PAGE_BUF);
    if(e < 0) ERR(e);

    e = BfM_FreeTrain(&newPid, PAGE_BUF);
    if(e < 0) ERR(e);

    
    return(eNOERROR);
    
} /* edubtm_SplitInternal() */



/*@================================
 * edubtm_SplitLeaf()
 *================================*/
/*
 * Function: Four edubtm_SplitLeaf(ObjectID*, PageID*, BtreeLeaf*, Two, LeafItem*, InternalItem*)
 *
 * Description: 
 * (Following description is for original ODYSSEUS/COSMOS BtM.
 *  For ODYSSEUS/EduCOSMOS EduBtM, refer to the EduBtM project manual.)
 *
 *  The function edubtm_SplitLeaf(...) is similar to edubtm_SplitInternal(...) except
 *  that the entry of a leaf differs from the entry of an internal and the first
 *  key value of a new page is used to make an internal item of their parent.
 *  Internal pages do not maintain the linked list, but leaves do it, so links
 *  are properly updated.
 *
 * Returns:
 *  Error code
 *  eDUPLICATEDOBJECTID_BTM
 *    some errors caused by function calls
 *
 * Note:
 *  The caller should call BfM_SetDirty() for 'fpage'.
 */
Four edubtm_SplitLeaf(
    ObjectID                    *catObjForFile, /* IN catalog object of B+ tree file */
    PageID                      *root,          /* IN PageID for the given page, 'fpage' */
    BtreeLeaf                   *fpage,         /* INOUT the page which will be splitted */
    Two                         high,           /* IN slotNo for the given 'item' */
    LeafItem                    *item,          /* IN the item which will be inserted */
    InternalItem                *ritem)         /* OUT the item which will be returned by spliting */
{
	/* These local variables are used in the solution code. However, you don��t have to use all these variables in your code, and you may also declare and use additional local variables if needed. */
    Four                        e;              /* error number */
    Two                         i;              /* slot No. in the given page, fpage */
    Two                         j;              /* slot No. in the splitted pages */
    Two                         k;              /* slot No. in the new page */
    Two                         maxLoop;        /* # of max loops; # of slots in fpage + 1 */
    Four                        sum;            /* the size of a filled area */
    PageID                      newPid;         /* for a New Allocated Page */
    PageID                      nextPid;        /* for maintaining doubly linked list */
    BtreeLeaf                   tpage;          /* a temporary page for the given page */
    BtreeLeaf                   *npage;         /* a page pointer for the new page */
    BtreeLeaf                   *mpage;         /* for doubly linked list */
    btm_LeafEntry               *itemEntry;     /* entry for the given 'item' */
    btm_LeafEntry               *fEntry;        /* an entry in the given page, 'fpage' */
    btm_LeafEntry               *nEntry;        /* an entry in the new page, 'npage' */
    ObjectID                    *iOidArray;     /* ObjectID array of 'itemEntry' */
    ObjectID                    *fOidArray;     /* ObjectID array of 'fEntry' */
    Two                         fEntryOffset;   /* starting offset of 'fEntry' */
    Two                         nEntryOffset;   /* starting offset of 'nEntry' */
    Two                         oidArrayNo;     /* element No in an ObjectID array */
    Two                         alignedKlen;    /* aligned length of the key length */
    Two                         itemEntryLen;   /* length of entry for item */
    Two                         entryLen;       /* entry length */
    Boolean                     flag;
    Boolean                     isTmp;
 
    itemEntry = (btm_LeafEntry*)&(tpage.data[0]);
    if(item->nObjects == 0){
        alignedKlen = ALIGNED_LENGTH(item->klen);
        itemEntry->nObjects = 1;
        itemEntry->klen = item->klen;
        memcpy(&(itemEntry->kval[0]), &(item->kval[0]), itemEntry->klen);
        memcpy(&(itemEntry->kval[alignedKlen]), (char*)&(item->oid), OBJECTID_SIZE);

        itemEntryLen = BTM_LEAFENTRY_FIXED + alignedKlen + OBJECTID_SIZE;
    }else if(item->nObjects > 0){
        fEntryOffset = fpage->slot[-(high + 1)];
        fEntry = (btm_LeafEntry*)&(fpage->data[fEntryOffset]);
        alignedKlen = ALIGNED_LENGTH(fEntry->klen);
        fOidArray = (ObjectID*)&(fEntry->kval[alignedKlen]);

        e = btm_BinarySearchOidArray(fOidArray, &(item->oid), fEntry -> nObjects, &oidArrayNo);
        
        itemEntry->nObjects = fEntry->nObjects + 1;
        itemEntry->klen = fEntry->klen;
        memcpy(&(itemEntry->kval[0]), &(fEntry->kval[0]), itemEntry->klen);
        iOidArray = (ObjectID*)&(itemEntry->kval[alignedKlen]);
        memcpy((char*)&(iOidArray[0]), (char*)&(fOidArray[0]), OBJECTID_SIZE*(oidArrayNo + 1));
        iOidArray[oidArrayNo + 1] = item->oid;
        memcpy((char*)&(iOidArray[oidArrayNo + 2]), (char*)&(fOidArray[oidArrayNo + 1]), OBJECTID_SIZE*(fEntry->nObjects - (oidArrayNo + 1)));
        itemEntryLen = BTM_LEAFENTRY_FIXED + alignedKlen + OBJECTID_SIZE*(itemEntry->nObjects);

        for(i = high + 1; i < fpage->hdr.nSlots; i++)
            fpage->slot[-i] = fpage->slot[-(i + 1)];
        fpage->hdr.nSlots--;
        if(fEntryOffset + itemEntryLen - sizeof(ObjectID) == fpage->hdr.free)
            fpage->hdr.free -= itemEntryLen - sizeof(ObjectID);
        else
            fpage->hdr.unused += itemEntryLen - sizeof(ObjectID);
    }else{

    }
    
    e = btm_AllocPage(catObjForFile, (PageID*)&fpage->hdr.pid, &newPid);
    if(e < 0) ERR(e);

    e = btm_IsTemporary(catObjForFile, &isTmp);
    if(e < 0) ERR(e);

    e = edubtm_InitLeaf(&newPid, FALSE, isTmp);
    if(e < 0) ERR(e);

    e = BfM_GetNewTrain(&newPid, (char **)&npage, PAGE_BUF);
    if(e < 0) ERR(e);

    maxLoop = fpage->hdr.nSlots + 1;
    flag = FALSE;
    for(sum = 0, i = 0, j = 0; j < maxLoop && sum < BL_HALF; j++){
        if(j == high + 1){
            sum += itemEntryLen;
            flag = TRUE;
        }else{
            fEntryOffset = fpage->slot[-i];
            fEntry = (btm_LeafEntry*)&(fpage->data[fEntryOffset]);
            sum += BTM_LEAFENTRY_FIXED + ALIGNED_LENGTH(fEntry->klen) + ((fEntry->nObjects > 0) ? OBJECTID_SIZE*fEntry->nObjects : sizeof(ShortPageID));

            i++;
        }
        sum += sizeof(Two);
    }

    fpage->hdr.nSlots = i;

    for(k = 0; j < maxLoop; j++, k++){
        nEntryOffset = npage->slot[-k] = npage->hdr.free;
        nEntry = (btm_LeafEntry*)&(npage->data[nEntryOffset]);

        if(j == high + 1){
            entryLen = itemEntryLen;
            memcpy((char * )nEntry, (char*)itemEntry, entryLen);
        }else{
            fEntryOffset = fpage->slot[-i];
            fEntry = (btm_LeafEntry*)&(fpage->data[fEntryOffset]);

            if(fEntry->nObjects < 0){
                entryLen = BTM_LEAFENTRY_FIXED + ALIGNED_LENGTH(fEntry->klen) + sizeof(ShortPageID);
            }else{
                entryLen = BTM_LEAFENTRY_FIXED + ALIGNED_LENGTH(fEntry->klen) + fEntry->nObjects*OBJECTID_SIZE;
            }

            memcpy((char*)nEntry, (char*)fEntry, entryLen);

            if(fEntryOffset + entryLen == fpage->hdr.free)
                fpage->hdr.free -= entryLen;
            else
                fpage->hdr.unused += entryLen;

            i++;
        }

        npage->hdr.free += entryLen;
    }

    npage->hdr.nSlots = k;

    if(flag){
        if(BL_CFREE(fpage) < itemEntryLen + sizeof(Two))
            edubtm_CompactLeafPage(fpage, NIL);

        for(i = fpage->hdr.nSlots - 1; i >= high + 1; i--)
            fpage->slot[-(i + 1)] = fpage->slot[-i];
        
        fEntryOffset = fpage->slot[-(high + 1)] = fpage->hdr.free;
        fEntry = (btm_LeafEntry*)&(fpage->data[fEntryOffset]);

        memcpy((char*)fEntry, (char*)itemEntry, itemEntryLen);

        fpage->hdr.free += itemEntryLen;
        fpage->hdr.nSlots++;
    }

    nEntryOffset = npage->slot[0];
    nEntry = (btm_LeafEntry*)&(npage->data[nEntryOffset]);
    ritem->spid = newPid.pageNo;
    ritem->klen = nEntry->klen;
    memcpy(&(ritem->kval[0]), &(nEntry->kval[0]), ritem->klen);

    if(fpage->hdr.type & ROOT) fpage->hdr.type = LEAF;

    MAKE_PAGEID(nextPid, root->volNo, fpage->hdr.nextPage);
    npage->hdr.nextPage = nextPid.pageNo;
    npage->hdr.prevPage = root->pageNo;
    fpage->hdr.nextPage = newPid.pageNo;
    if(nextPid.pageNo != NIL){
        e = BfM_GetTrain(&nextPid, (char**)&mpage, PAGE_BUF);
        if(e < 0) ERR(e);
        mpage->hdr.prevPage = newPid.pageNo;

        e = BfM_SetDirty(&nextPid, PAGE_BUF);
        if(e < 0) ERR(e);

        e = BfM_FreeTrain(&nextPid, PAGE_BUF);
        if(e < 0) ERR(e);
    }

    e = BfM_SetDirty(&newPid, PAGE_BUF);
    if(e < 0) ERR(e);

    e = BfM_FreeTrain(&newPid, PAGE_BUF);
    if(e < 0) ERR(e);

    return(eNOERROR);
    
} /* edubtm_SplitLeaf() */
