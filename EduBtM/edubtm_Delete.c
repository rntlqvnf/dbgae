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
 * Module: edubtm_Delete.c
 *
 * Description : 
 *  This function edubtm_Delete(...) recursively calls itself until the type
 *  of root page becomes LEAF.  If the root page is an internal page, it
 *  may get the proper child page using the binary search routine and then
 *  recursively calls itself using the child as a root page. If the filled
 *  area of the child page is less than half of the page, it should merge
 *  or redistribute using the given root, and set the flag 'f' according to
 *  the result status of the given root page.
 *
 *  If the root page is a leaf page , it find out the correct node (entry)
 *  using the binary search routine.  If the entry is normal,  it simply
 *  delete the ObjectID or the entry when the # of ObjectIDs becomes zero.
 *  The entry, however, is not normal, that is, if the overflow page is used,
 *  the special routine btm_DeleteOverflow(...) should be called. The # of
 *  ObjectIDs will be returned by the result of the btm_DeleteOverflow(...),
 *  if the total # of ObjectIDs is less than 1/4 of the page and the ObjectIDs
 *  in the overflow page should be moved to the leaf page. (This process may
 *  has a complicate problem which the leaf page may be splitted in spite of
 *  deleteing not inserting an ObjectID.)
 *
 *  Deleting an ObjectID may cause redistribute pages and by this reason, the
 *  page may be splitted.
 *
 * Exports:
 *  Four edubtm_Delete(ObjectID*, PageID*, KeyDesc*, KeyValue*, ObjectID*,
 *                  Boolean*, Boolean*, InternalItem*, Pool*, DeallocListElem*)
 *
 */


#include <string.h>
#include "EduBtM_common.h"
#include "Util.h"
#include "BfM.h"
#include "OM_Internal.h"	/* for "SlottedPage" including catalog object */
#include "EduBtM_Internal.h"


/*@ Internal Function Prototypes */
Four edubtm_DeleteLeaf(PhysicalFileID*, PageID*, BtreeLeaf*, KeyDesc*, KeyValue*, ObjectID*,
		    Boolean*, Boolean*, InternalItem*, Pool*, DeallocListElem*);



/*@================================
 * edubtm_Delete()
 *================================*/
/*
 * Function: Four edubtm_Delete(ObjectID*, PageID*, KeyDesc*, KeyValue*,
 *                           ObjectID*, Boolean*, Boolean*, InternalItem*)
 *
 * Description:
 * (Following description is for original ODYSSEUS/COSMOS BtM.
 *  For ODYSSEUS/EduCOSMOS EduBtM, refer to the EduBtM project manual.)
 *
 *
 * Returns:
 *  error code
 *    eBADBTREEPAGE_BTM
 *    some errors caused by function calls
 *
 * Side effects:
 *  f    : TRUE if the given root page is not half full.
 *  h    : TRUE if the given page is splitted.
 *  item : The internal item to be inserted into the parent if 'h' is TRUE.
 */
Four edubtm_Delete(
    ObjectID                    *catObjForFile, /* IN catalog object of B+ tree file */
    PageID                      *root,          /* IN root page */
    KeyDesc                     *kdesc,         /* IN a key descriptor */
    KeyValue                    *kval,          /* IN key value */
    ObjectID                    *oid,           /* IN Object IDentifier which will be deleted */
    Boolean                     *f,             /* OUT whether the root page is half full */
    Boolean                     *h,             /* OUT TRUE if it is spiltted. */
    InternalItem                *item,          /* OUT The internal item to be returned */
    Pool                        *dlPool,        /* INOUT pool of dealloc list elements */
    DeallocListElem             *dlHead)        /* INOUT head of the dealloc list */
{
	/* These local variables are used in the solution code. However, you don��t have to use all these variables in your code, and you may also declare and use additional local variables if needed. */
    Four                        e;              /* error number */
    Boolean                     lf;             /* TRUE if a page is not half full */
    Boolean                     lh;             /* TRUE if a page is splitted */
    Two                         idx;            /* the index by the binary search */
    PageID                      child;          /* a child page when the root is an internal page */
    KeyValue                    tKey;           /* a temporary key */
    BtreePage                   *rpage;         /* for a root page */
    InternalItem                litem;          /* local internal item */
    btm_InternalEntry           *iEntry;        /* an internal entry */
    SlottedPage                 *catPage;       /* buffer page containing the catalog object */
    sm_CatOverlayForBtree       *catEntry;      /* pointer to Btree file catalog information */
    PhysicalFileID              pFid;           /* B+-tree file's FileID */
  

    /* Error check whether using not supported functionality by EduBtM */
	int i;
    for(i=0; i<kdesc->nparts; i++)
    {
        if(kdesc->kpart[i].type!=SM_INT && kdesc->kpart[i].type!=SM_VARSTRING)
            ERR(eNOTSUPPORTED_EDUBTM);
    }

        
    *h = *f = FALSE;
    
    e = BfM_GetTrain((TrainID*)catObjForFile, (char**)&catPage, PAGE_BUF);

    GET_PTR_TO_CATENTRY_FOR_BTREE(catObjForFile, catPage, catEntry);

    MAKE_PHYSICALFILEID(pFid, catEntry->fid.volNo, catEntry->firstPage);

    e = BfM_FreeTrain((TrainID*)catObjForFile, PAGE_BUF);

    e = BfM_GetTrain(root, (char**)&rpage, PAGE_BUF);

    if(rpage->any.hdr.type & INTERNAL){
        (Boolean) edubtm_BinarySearchInternal((&rpage->bi), kdesc, kval &idx);

        if(idx >= 0) {
            iEntry = (btm_InternalEntry*)&(rpage->bi.data[rpage->bi.slot[-idx]]);
            MAKE_PAGEID(child, root->volNo, iEntry->spid);
        } else
        {
            MAKE_PAGEID(child, root->volNo, rpage->bi.hdr.p0);
        }

        e = edubtm_Delete(catObjForFile, &child, kdesc, kval, oid, &lf, &lh, &litem, dlPool, dlHead);

        if(lh){
            tKey.len = litem.klen;
            memcpy(&(tKey.val[0]), &(litem.kval[0]), tKey.len);
            (Boolean) edubtm_BinarySearchInternal(&(rpage->bi), kdesc, &tKey, &idx);

            e = edubtm_InsertInternal(catObjForFile, &(rpage->bi), &litem, idx, h, item);

            e = BfM_SetDirty(root, PAGE_BUF);
        }else if(lf){
            e = btm_Underflow(&pFid, rpage, &child, idx, f, &lh, &litem, dlPool, dlHead);

            if(lh){
                tKey.len = litem.klen;
                memcpy(&(tKey.val[0]), &(litem.kval[0]), tKey.len);
                (Boolean)edubtm_BinarySearchInternal((&rpage->bi), kdesc, &tKey, &idx);

                e = edubtm_InsertInternal(catObjForFile, &(rpage->bi), &litem, idx, h, item);

            }

            e = BfM_SetDirty(root, PAGE_BUF);

        }
        
    }else if(rpage->any.hdr.type & LEAF){
        e = edubtm_DeleteLeaf(&pFid, root, &(rpage->bl), kdesc, kval, oid, f, h, item, dlPool, dlHead);
    }

    e = BfM_FreeTrain(root, PAGE_BUF);


    return(eNOERROR);
    
}   /* edubtm_Delete() */



/*@================================
 * edubtm_DeleteLeaf()
 *================================*/
/*
 * Function: Four edubtm_DeleteLeaf(PhysicalFileID*, PageID*, BtreeLeaf*, KeyDesc*,
 *                               KeyValue*, ObjectID*, Boolean*, Boolean*,
 *                               InternalItem*, Pool*, DeallocListElem*)
 *
 * Description:
 * (Following description is for original ODYSSEUS/COSMOS BtM.
 *  For ODYSSEUS/EduCOSMOS EduBtM, refer to the EduBtM project manual.)
 *
 *
 * Returns:
 *  Error code
 *    eNOTFOUND_BTM
 *    some errors caused by function calls
 *
 * Side effects:
 *  f    : TRUE if the given root page is not half full.
 *  h    : TRUE if the given page is splitted.
 *  item : The internal item to be inserted into the parent if 'h' is TRUE.
 */ 
Four edubtm_DeleteLeaf(
    PhysicalFileID              *pFid,          /* IN FileID of the Btree file */
    PageID                      *pid,           /* IN PageID of the leaf page */
    BtreeLeaf                   *apage,         /* INOUT buffer for the Leaf Page */
    KeyDesc                     *kdesc,         /* IN a key descriptor */
    KeyValue                    *kval,          /* IN key value */
    ObjectID                    *oid,           /* IN ObjectID which will be deleted */
    Boolean                     *f,             /* OUT whether the root page is half full */
    Boolean                     *h,             /* OUT TRUE if it is spiltted. */
    InternalItem                *item,          /* OUT The internal item to be returned */
    Pool                        *dlPool,        /* INOUT pool of dealloc list elements */
    DeallocListElem             *dlHead)        /* INOUT head of a dealloc list */
{
	/* These local variables are used in the solution code. However, you don��t have to use all these variables in your code, and you may also declare and use additional local variables if needed. */
    Four                        e;              /* error number */
    Two                         i;              /* index */
    Two                         of;             /* # of ObjectIDs of an overflow page when less than 1/4 */
    Two                         idx;            /* the index by the binary search */
    ObjectID                    tOid;           /* a Object IDentifier */
    BtreeOverflow               *opage;         /* for a overflow page */
    Boolean                     found;          /* Search Result */
    Two                         lEntryOffset;   /* starting offset of a leaf entry */
    btm_LeafEntry               *lEntry;        /* an entry in leaf page */
    ObjectID                    *oidArray;      /* start position of the ObjectID array */
    Two                         oidArrayElemNo; /* element number in the ObjectIDs array */
    Two                         entryLen;       /* length of the old leaf entry */
    Two                         newLen;         /* length of the new leaf entry */
    Two                         alignedKlen;    /* aligned length of the key length */
    PageID                      ovPid;          /* overflow page's PageID */
    DeallocListElem             *dlElem;        /* an element of the dealloc list */


    /* Error check whether using not supported functionality by EduBtM */
    for(i=0; i<kdesc->nparts; i++)
    {
        if(kdesc->kpart[i].type!=SM_INT && kdesc->kpart[i].type!=SM_VARSTRING)
            ERR(eNOTSUPPORTED_EDUBTM);
    }

    found = edubtm_BinarySearchLeaf(apage, kdesc, kval, &idx);

    if(!found) return(eNOTFOUND_BTM);
    lEntryOffset = apage->slot[-idx];
    lEntry - (btm_LeafEntry*)&(apage->data[lEntryOffset]);

    alignedKlen = ALIGNED_LENGTH(lEntry->klen);

    if(lEntry->nObjects < 0){
        MAKE_PAGEID(ovPid, pid->volNo, *((ShortPageID*)&(lEntry->kval[alignedKlen])));

        e = btm_DeleteOverflow(pFid, &ovPid, oid, &of, dlPool, dlHead);

        if(of){
            if(of*OBJECTID_SIZE <= (BL_FREE(apage) +sizeof(ShortPageID))){
                e = BfM_GetTrain(&ovPid, (char**)&opage, PAGE_BUF);

                entryLen = BTM_LEAFENTRY_FIXED + alignedKlen + sizeof(ShortPageID);

                newLen = BTM_LEAFENTRY_FIXED + alignedKlen + of*OBJECTID_SIZE;

                if(lEntryOffset + entryLen == apage->hdr.free && of*OBJECTID_SIZE <= BL_CFREE(apage) + sizeof(ShortPageID)){
                    apage->hdr.free += of*OBJECTID_SIZE - sizeof(ShortPageID);
                }else if(newLen <= BL_CFREE(apage)){
                    apage->slot[-idx] = apage->hdr.free;

                    memcpy(&(apage->data[apage->hdr.free]), (char*)lEntry, entryLen-sizeof(ShortPageID));
                    lEntry = (btm_LeafEntry*)&(apage->data[apage->hdr.free]);

                    apage->hdr.free += newLen;
                    apage->hdr.unused += entryLen;
                }else{
                    edubtm_CompactLeafPage(apage, idx);

                    lEntryOffset = apage->slot[-idx];
                    lEntry = (btm_LeafEntry*)&(apage->data[lEntryOffset]);

                    apage->hdr.free += of*OBJECTID_SIZE - sizeof(ShortPageID);
                }
                oidArray = (ObjectID*)&(lEntry->kval[alignedKlen]);
                memcpy((char*)&(oidArray[0]), (char*)&(opage->oid[0]), of*OBJECTID_SIZE);

                lEntry->nObjects = of;
                opage->hdr.type = FREEPAGE;
                e = BfM_SetDirty(&ovPid, PAGE_BUF);

                e = BfM_FreeTrain(&ovPid, PAGE_BUF);

                e = Util_getElementFromPool(dlPool, &dlElem);

                dlElem->type = DL_PAGE;
                dlElem->elem.pid = ovPid;
                dlElem->next = dlHead->next;
                dlHead->next = dlElem;
            }
        }
    }else if(lEntry->nObjects > 1){
        oidArray = (ObjectID*)&(lEntry->kval[alignedKlen]);
        found = btm_BinarySearchOidArray(oidArray, oid, lEntry->nObjects, &oidArrayElemNo);

        if(found){
            entryLen = BTM_LEAFENTRY_FIXED + alignedKlen + lEntry->nObjects*OBJECTID_SIZE;

            memmove((char *)&(oidArray[oidArrayElemNo]), (char*)&(oidArray[oidArrayElemNo + 1]), (lEntry->nObjects - oidArrayElemNo - 1) * OBJECTID_SIZE);

            lEntry->nObjects --;

            if(lEntryOffset + entryLen == apage->hdr.free)
                apage->hdr.free -= OBJECTID_SIZE;
            else
                apage->hdr.unused += OBJECTID_SIZE;
        }else
            return(eNOTFOUND_BTM);
    }else{
        tOid = *((ObjectID*)&(lEntry->kval[alignedKlen]));
        if(btm_ObjectIdComp(oid, &tOid) == EQUAL){
            for(i = idx + 1; i < apage->hdr.nSlots; i++)
                apage->slot[-(i - 1)] = apage->slot[-i];

            entryLen = BTM_LEAFENTRY_FIXED + alignedKlen + OBJECTID_SIZE;

            if(lEntryOffset + entryLen == apage->hdr.free)
                apage->hdr.free -= entryLen;
            else
                apage->hdr.unused += entryLen;
            
            apage->hdr.nSlots--;
        }else
            return(eNOTFOUND_BTM);
    }

    if(BL_FREE(apage) > BL_HALF)
        *f = TRUE;
    
    e = BfM_SetDirty(pid, PAGE_BUF);
    if(e < 0) ERR(e);
	      
    return(eNOERROR);
    
} /* edubtm_DeleteLeaf() */
