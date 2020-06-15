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
 * Module: EduBtM_FetchNext.c
 *
 * Description:
 *  Find the next ObjectID satisfying the given condition. The current ObjectID
 *  is specified by the 'current'.
 *
 * Exports:
 *  Four EduBtM_FetchNext(PageID*, KeyDesc*, KeyValue*, Four, BtreeCursor*, BtreeCursor*)
 */


#include <string.h>
#include "EduBtM_common.h"
#include "BfM.h"
#include "EduBtM_Internal.h"


/*@ Internal Function Prototypes */
Four edubtm_FetchNext(KeyDesc*, KeyValue*, Four, BtreeCursor*, BtreeCursor*);



/*@================================
 * EduBtM_FetchNext()
 *================================*/
/*
 * Function: Four EduBtM_FetchNext(PageID*, KeyDesc*, KeyValue*,
 *                              Four, BtreeCursor*, BtreeCursor*)
 *
 * Description:
 * (Following description is for original ODYSSEUS/COSMOS BtM.
 *  For ODYSSEUS/EduCOSMOS EduBtM, refer to the EduBtM project manual.)
 *
 *  Fetch the next ObjectID satisfying the given condition.
 * By the B+ tree structure modification resulted from the splitting or merging
 * the current cursor may point to the invalid position. So we should adjust
 * the B+ tree cursor before using the cursor.
 *
 * Returns:
 *  error code
 *    eBADPARAMETER_BTM
 *    eBADCURSOR
 *    some errors caused by function calls
 */
Four EduBtM_FetchNext(
    PageID                      *root,          /* IN root page's PageID */
    KeyDesc                     *kdesc,         /* IN key descriptor */
    KeyValue                    *kval,          /* IN key value of stop condition */
    Four                        compOp,         /* IN comparison operator of stop condition */
    BtreeCursor                 *current,       /* IN current B+ tree cursor */
    BtreeCursor                 *next)          /* OUT next B+ tree cursor */
{
	/* These local variables are used in the solution code. However, you don��t have to use all these variables in your code, and you may also declare and use additional local variables if needed. */
    int							i;
    Four                        e;              /* error number */
    Four                        cmp;            /* comparison result */
    Two                         slotNo;         /* slot no. of a leaf page */
    Two                         oidArrayElemNo; /* element no. of the array of ObjectIDs */
    Two                         alignedKlen;    /* aligned length of key length */
    PageID                      overflow;       /* temporary PageID of an overflow page */
    Boolean                     found;          /* search result */
    ObjectID                    *oidArray;      /* array of ObjectIDs */
    BtreeLeaf                   *apage;         /* pointer to a buffer holding a leaf page */
    BtreeOverflow               *opage;         /* pointer to a buffer holding an overflow page */
    btm_LeafEntry               *entry;         /* pointer to a leaf entry */
    BtreeCursor                 tCursor;        /* a temporary Btree cursor */
  
    
    /*@ check parameter */
    if (root == NULL || kdesc == NULL || kval == NULL || current == NULL || next == NULL)
	ERR(eBADPARAMETER_BTM);
    
    /* Is the current cursor valid? */
    if (current->flag != CURSOR_ON && current->flag != CURSOR_EOS)
		ERR(eBADCURSOR);
    
    if (current->flag == CURSOR_EOS) return(eNOERROR);
    
    /* Error check whether using not supported functionality by EduBtM */
    for(i=0; i<kdesc->nparts; i++)
    {
        if(kdesc->kpart[i].type!=SM_INT && kdesc->kpart[i].type!=SM_VARSTRING)
            ERR(eNOTSUPPORTED_EDUBTM);
    }

    *next = *current
    e = BfM_GetTrain(&next->leaf, (char**)&apage, PAGE_BUF);
    if(e < 0) ERR(e);

    if(!(apage->hdr.type & LEAF)) goto retraverse;

    found = FALSE;
    if(next->slotNo >= 0 && next->slotNo < apage->hdr.nSlots){
        entry = (btm_LeafEntry*)&apage->data[apage->slot[-next->slotNo]];

        if(edubtm_KeyCompare(kdesc, (KeyValue*)&entry->klen, &current->key) == EQUAL)
            found = TRUE;
    }

    if(!found){
        found = edubtm_BinarySearchLeaf(apage, kdesc, &current->key, &slotNo);
        if(!found && (slotNo < 0 || slotNo >= apage->hdr.nSlots-1)) goto retraverse;
        next->slotNo = slotNo;
    }

    if(!found){
        switch(compOp){
            case SM_EQ:
                next->flag = CURSOR_EOS;
                break;
            case SM_LT:
            case SM_LE:
            case SM_EOF:
                next->slotNo++;
                entry = (btm_LeafEntry*)&(apage->data[apage->slot[-next->slotNo]]);
                alignedKlen = ALIGNED_LENGTH(entry->klen);

                if(compOp != SM_EOF){
                    cmp = edubtm_KeyCompare(kdesc, (KeyValue*)&entry->klen, kval);
                    if(cmp == GREAT || (cmp == EQUAL && compOp == SM_LT)){
                        next->flag = CURSOR_EOS;
                        break;
                    }
                }

                if(entry->nObjects < 0){
                    MAKE_PAGEID(next->overflow, next->leaf.volNo, *((ShortPageID*)&entry->kval]alignedKlen]));

                    e = BfM_GetTrain(&next-overflow, (char**)&opage, PAGE_BUF);

                    next->oidArrayElemNo = 0;
                    next->oid = opage->oid[next->oidArrayElemNo];
                    
                    e = BfM_FreeTrain(&next->overflow, PAGE_BUF);
                }else{
                    oidArray = (ObjectID*)&(entry->kval[alignedKlen]);

                    next->oidArrayElemNo = 0;
                    next->oid = oidArray[0];
                    MAKE_PAGEID(next->overflow, next->leaf.volNo, NIL);
                }

                next->flag = CURSOR_ON;
                break;
            
            case SM_GT:
            case SM_GE:
            case SM_BOF:
                entry = (btm_LeafEntry*)&(apage->data[apage -> slot[-next->slotNo]]);
                alignedKlen = ALIGNED_LENGTH(entry->klen);

                if(compOp != SM_BOF){
                    cmp = edubtm_KeyCompare(kdesc, (KeyValue*)&entry->klen, kval);
                    if(cmp == LESS || (cmp == EQUAL && compOp == SM_GT)){
                        next->flag = CURSOR_EOS;
                        break;
                    }
                }

                if(entry->nObjects < 0){

                    MAKE_PAGEID(next->overflow, next->leaf.volNo, *((ShortPageID*)&entry->kval[alignedKlen]));

                    e = BfM_GetTrain(&next->overflow, (char**)&opage, PAGE_BUF);
                    while(opage->hdr.nextPage != nil){
                        MAKE_PAGEID(overflow, next->overflow.volNo, opage->hdr.nextPage);

                        e = BfM_FreeTrain(&next->overflow, PAGE_BUF);

                        next -> overflow = overflow;

                        e = BfM_GetTrain(&next->overflow, (char**)&opage, PAGE_BUF);
                    }
                    next->oidArrayElemNo = opage->hdr.nObjects - 1;
                    next->oid = opage->oid[next->oidArrayElemNo];

                    e = BfM_FreeTrain(&next->overflow, PAGE_BUF);
                }else{
                    oidArray = (ObjectID*)&(entry->kval[alignedKlen]);

                    next->oidArrayElemNo = entry->nObjects - 1;
                    next->oid = oidArray[next->oidArrayElemNo];
                    MAKE_PAGEID(next->overflow, next->leaf.volNo, NIL);
                }

                next->flag = CURSOR_ON;
                break;
        }

        if(next->flag == CURSOR_ON)
            memcpy((char*)&next->key, (char*) &entry->klen, entry->klen + sizeof(Two));
        
        e = BfM_FreeTrain(&next->leaf, PAGE_BUF);

        return (eNOERROR);
    }

    entry = (btm_LeafEntry*)&(apage->data[apage->slot[-next->slotNo]]);
    alignedKlen = ALIGNED_LENGTH(entry->klen);

    if(entry->nObjects < 0){
        if(IS_NILPAGEID(next->overflow)){
            MAKE_PAGEID(next->overflow, next->leaf.volNo, *((ShortPageID*)(&entry->kval[alignedKlen])));
            next->oidArrayElemNo = 0;
        }else{
            Boolean freePageFlag;

            e = BfM_GetTrain(&next->overflow, (char**)&opage, PAGE_BUF);

            freePageFlag = (opage->hdr.type == FREEPAGE) ? TRUE:FALSE;

            e = BfM_FreeTrain(&next->overflow, PAGE_BUF);

            if(freePageFlag){
                MAKE_PAGEID(next->overflow, next->leaf.volNo, *((ShortPageID*)(&entry->kval[alignedKlen])));
                next->oidArrayElemNo = 0;;
            }
        }
        e = BfM_FreeTrain(&next->leaf, PAGE_BUF);

        //e = btm_FetchObjectIdInOverflow(next, compOp, &found);

        if(!found){
            tCursor = *next;
            e = edubtm_FetchNext(kdesc, kval, compOp, &tCursor, next);
        }
    }else{
        oidArray = (ObjectID*)&entry->kval[alignedKlen];
        found = btm_BinarySearchOidArray(oidArray, &current->oid, entry->nObjects, &oidArrayElemNo);

        if(compOp == SM_EQ || compOp == SM_LT || compOp == SM_LE || compOp == SM_EOF)
            next->oidArrayElemNo = oidArrayElemNo;
        else
            next->oidArrayElemNo = (found) ? oidArrayElemNo:(oidArrayElemNo + 1);

        MAKE_PAGEID(next->overflow, next->leaf.volNo, NIL);

        e = BfM_FreeTrain(&next->leaf, PAGE_BUF);

        tCursor = *next;
        e = edubtm_FetchNext(kdesc, kval, compOp, &tCursor, next);
    }
    
    return(eNOERROR);

    retraverse:
    e = BfM_FreeTrain(&next->leaf, PAGE_BUF);

    return(eNOERROR);
    
} /* EduBtM_FetchNext() */



/*@================================
 * edubtm_FetchNext()
 *================================*/
/*
 * Function: Four edubtm_FetchNext(KeyDesc*, KeyValue*, Four,
 *                              BtreeCursor*, BtreeCursor*)
 *
 * Description:
 * (Following description is for original ODYSSEUS/COSMOS BtM.
 *  For ODYSSEUS/EduCOSMOS EduBtM, refer to the EduBtM project manual.)
 *
 *  Get the next item. We assume that the current cursor is valid; that is.
 *  'current' rightly points to an existing ObjectID.
 *
 * Returns:
 *  Error code
 *    eBADCOMPOP_BTM
 *    some errors caused by function calls
 */
Four edubtm_FetchNext(
    KeyDesc  		*kdesc,		/* IN key descriptor */
    KeyValue 		*kval,		/* IN key value of stop condition */
    Four     		compOp,		/* IN comparison operator of stop condition */
    BtreeCursor 	*current,	/* IN current cursor */
    BtreeCursor 	*next)		/* OUT next cursor */
{
	/* These local variables are used in the solution code. However, you don��t have to use all these variables in your code, and you may also declare and use additional local variables if needed. */
    Four 		e;		/* error number */
    Four 		cmp;		/* comparison result */
    Two 		alignedKlen;	/* aligned length of a key length */
    PageID 		leaf;		/* temporary PageID of a leaf page */
    PageID 		overflow;	/* temporary PageID of an overflow page */
    ObjectID 		*oidArray;	/* array of ObjectIDs */
    BtreeLeaf 		*apage;		/* pointer to a buffer holding a leaf page */
    BtreeOverflow 	*opage;		/* pointer to a buffer holding an overflow page */
    btm_LeafEntry 	*entry;		/* pointer to a leaf entry */    
    
    
    /* Error check whether using not supported functionality by EduBtM */
    int i;
    for(i=0; i<kdesc->nparts; i++)
    {
        if(kdesc->kpart[i].type!=SM_INT && kdesc->kpart[i].type!=SM_VARSTRING)
            ERR(eNOTSUPPORTED_EDUBTM);
    }

    *next = *current;

    if(compOp == SM_EQ || compOp == SM_LT || compOp == SM_LE || compOp == SM_EOF){
        if(IS_NILPAGEID(next->overflow)){
            e = BfM_GetTrain(&next->leaf, (char**)&apage, PAGE_BUF);

            entry = (btm_LeafEntry*)&(apage->data[apage->slot[-next->slotNo]]);

            next->oidArrayElemNo ++;

            if(next->oidArrayElemNo < entry->nObjects){
                oidArray = (ObjectID*)&entry->kval[ALIGNED_LENGTH(entry->klen)];
                next->oid = oidArray[next->oidArrayElemNo];
                next->flag = CURSOR_ON;

                e = BfM_FreeTrain(&next->leaf, PAGE_BUF);

                return(eNOERROR);
            }
        }else{
            e = BfM_GetTrain(&next->overflow, (char**)&opage, PAGE_BUF);

            next->oidArrayElemNo++;

            if(next->oidArrayElemNo >= opage->hdr.nObjects && opage->hdr.nextPage != NIL){
                MAKE_PAGEID(overflow, next->overflow.volNo, opage->hdr.nextPage);

                e = BfM_FreeTrain(&next->overflow, PAGE_BUF);

                next->overflow = overflow;

                e = BfM_GetTrain(&next->overflow, (char**)&opage, PAGE_BUF);

                next->oidArrayElemNo = 0;
            }

            if(next->oidArrayElemNo < opage->hdr.nObjects){
                next->oid = opage->oid[next->oidArrayElemNo];
                next->flag  = CURSOR_ON;
            }else
            {
                next->flag = CURSOR_INVALID;
            }
            e = BfM_FreeTrain(&next->overflow, PAGE_BUF);

            if(next->flag == CURSOR_ON) return(eNOERROR);

            e = BfM_GetTrain(&next->leaf, (char**)&apage, PAGE_BUF);
        }

        if(compOp == SM_EQ){
            next->flag = CURSOR_EOS;

            e = BfM_FreeTrain(&next->leaf, PAGE_BUF);

            return (eNOERROR);
        }
        next -> slotNo++;

        if(next->slotNo >= apage->hdr.nSlots && apage->hdr.nextPage != NIL){
            MAKE_PAGEID(leaf, next->leaf.volNo, apage->hdr.nextPage);

            e = BfM_FreeTrain(&next->leaf, PAGE_BUF);

            next->leaf = leaf;

            e = BfM_GetTrain(&next->leaf, (char**)&apage, PAGE_BUF);

            next->slotNo = 0;
        }
        if(next->slotNo < apage->hdr.nSlots){
            entry = (btm_LeafEntry*)&(apage->data[apage->slot[-next->slotNo]]);
            alignedKlen = ALIGNED_LENGTH(entry->klen);

            if (compOp != SM_EOF) {
                cmp = edubtm_KeyCompare(kdesc, (KeyValue*)&entry->klen, kval);
                if (cmp == GREAT || (compOp == SM_LT && cmp == EQUAL)) {
                    e = BfM_FreeTrain(handle, &next->leaf, PAGE_BUF);
                    
                    next->flag = CURSOR_EOS;
                    return(eNOERROR);
                }
	        }

            if(entry->nObjects < 0){
                MAKE_PAGEID(next->overflow, next->leaf.volNo, *((ShortPageID*)&entry->kval[alignedKlen]));

                e = BfM_GetTrain(&next->overflow, (char**)&opage, PAGE_BUF);

                next->oidArrayElemNo = 0;
                next->oid = opage->oid[next->oidArrayElemNo];

                e  = BfM_FreeTrain(&next->overflow, PAGE_BUF);
            }else{
                MAKE_PAGEID(next->overflow, next->leaf.volNo, NIL);
                next->oidArrayElemNo = 0;
                next->oid = *((ObjectID*)&entry->kval[alignedKlen]);
            }

            memcpy((char*)&next->key, (char*)&entry->klen, entry->klen + sizeof(Two));
            next->flag = CURSOR_ON;

        }else
        {
            next->flag = CURSOR_EOS;
        }
        e = BfM_FreeTrain(&next->leaf, PAGE_BUF);
        
    }else{
        if(IS_NILPAGEID(next->overflow)){
            e = BfM_GetTrain(&next->leaf, (char**)&apage, PAGE_BUF);
            entry = (btm_LeafEntry*)&apage->data[apage->slot[-next->slotNo]];

            next->oidArrayElemNo --;

            if(next->oidArrayElemNo >= 0){
                oidArray = (ObjectID*)&entry->kval[ALIGNED_LENGTH(entry->klen)];
                next->oid = oidArray[next->oidArrayElemNo];
                next->flag = CURSOR_ON;

                e = BfM_FreeTrain(&next->leaf, PAGE_BUF);

                return(eNOERROR);
            }
        }else{
            e = BfM_GetTrain(&next->overflow, (char**)&opage, PAGE_BUF);

            next->oidArrayElemNo--;

            if (next->oidArrayElemNo < 0 && opage->hdr.prevPage != NIL) {
                MAKE_PAGEID(overflow, next->overflow.volNo, opage->hdr.prevPage);
                
                e = BfM_FreeTrain(&next->overflow, PAGE_BUF);

                next->overflow = overflow;

                e = BfM_GetTrain(&next->overflow, (char**)&opage, PAGE_BUF);

                next->oidArrayElemNo = opage->hdr.nObjects -1;
            }
            if(next->oidArrayElemNo >= 0){
                next->oid = opage->oid[next->oidArrayElemNo];
                next->flag = CURSOR_ON;
            }else
            {
                next->flag = CURSOR_INVALID;
            }

            e = BfM_FreeTrain(&next->overflow, PAGE_BUF);

            if (next->flag == CURSOR_ON) return(eNOERROR);

            e = BfM_GetTrain(&next->leaf, (char**)&apage, PAGE_BUF);
            
        }

        next->slotNo--;

        if (next->slotNo < 0 && apage->hdr.prevPage != NIL) {
            MAKE_PAGEID(leaf, next->leaf.volNo, apage->hdr.prevPage);

            e = BfM_FreeTrain(&next->leaf, PAGE_BUF);
            next->leaf =leaf;

            e = BfM_GetTrain(&next->leaf, (char**)&apage, PAGE_BUF);

            next->slotNo = apage->hdr.nSlots - 1;
        }
        if (next->slotNo >= 0) {
            entry = (btm_LeafEntry*)&(apage->data[apage->slot[-next->slotNo]]);
            alignedKlen = ALIGNED_LENGTH(entry->klen);

            if (compOp != SM_BOF) {
                cmp = edubtm_KeyCompare(kdesc, (KeyValue*)&entry->klen, kval);

                if (cmp == LESS || (compOp == SM_GT && cmp == EQUAL)) {
                    e = BfM_FreeTrain(&next->leaf, PAGE_BUF);

                    next->flag = CURSOR_EOS;
                    return(eNOERROR);
                }
            }

            if(entry->nObjects < 0){
                MAKE_PAGEID(next->overflow, next->leaf.volNo, *((ShortPageID*)&entry->kval[alignedKlen]));

                e = BfM_GetTrain(&next->overflow, (char**)&opage, PAGE_BUF);

                while(opage->hdr.nextPage != NIL) {
                    MAKE_PAGEID(overflow, next->overflow.volNo, opage->hdr.nextPage );

                    e = BfM_FreeTrain(&next->overflow, PAGE_BUF);

                    next->overflow = overflow;
                    e = BfM_GetTrain(&next->overflow, (char**)&apage, PAGE_BUF);
                }
                next->oidArrayElemNo = opage->hdr.nObjects - 1;
                next->oid = opage->oid[next->oidArrayElemNo];

                e = BfM_FreeTrain(&next->overflow, PAGE_BUF);
            }else{
                MAKE_PAGEID(next->overflow, next->leaf.volNo, NIL);
                next->oidArrayElemNo = entry->nObjects - 1;
                oidArray =(ObjectID*)&entry->kval[alignedKlen];
                next->oid = oidArray[next->oidArrayElemNo];
            }

            memcpy((char*)&next->key, (char*)&entry->klen, entry->klen+sizeof(Two));	
            next->flag = CURSOR_ON;
        }else
            next->flag = CURSOR_ON;
        
        e = BfM_FreeTrain(&next->leaf, PAGE_BUF);
    }

    
    return(eNOERROR);
    
} /* edubtm_FetchNext() */
