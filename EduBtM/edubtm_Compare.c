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
 * Module: edubtm_Compare.c
 *
 * Description : 
 *  This file includes two compare routines, one for keys used in Btree Index
 *  and another for ObjectIDs.
 *
 * Exports: 
 *  Four edubtm_KeyCompare(KeyDesc*, KeyValue*, KeyValue*)
 *  Four edubtm_ObjectIdComp(ObjectID*, ObjectID*)
 */


#include <string.h>
#include "EduBtM_common.h"
#include "EduBtM_Internal.h"



/*@================================
 * edubtm_KeyCompare()
 *================================*/
/*
 * Function: Four edubtm_KeyCompare(KeyDesc*, KeyValue*, KeyValue*)
 *
 * Description:
 * (Following description is for original ODYSSEUS/COSMOS BtM.
 *  For ODYSSEUS/EduCOSMOS EduBtM, refer to the EduBtM project manual.)
 *
 *  Compare key1 with key2.
 *  key1 and key2 are described by the given parameter "kdesc".
 *
 * Returns:
 *  result of omparison (positive numbers)
 *    EQUAL : key1 and key2 are same
 *    GREAT : key1 is greater than key2
 *    LESS  : key1 is less than key2
 *
 * Note:
 *  We assume that the input data are all valid.
 *  User should check the KeyDesc is valid.
 */
Four edubtm_KeyCompare(
    KeyDesc                     *kdesc,		/* IN key descriptor for key1 and key2 */
    KeyValue                    *key1,		/* IN the first key value */
    KeyValue                    *key2)		/* IN the second key value */
{
	/* These local variables are used in the solution code. However, you don��t have to use all these variables in your code, and you may also declare and use additional local variables if needed. */
    register unsigned char      *left;          /* left key value */
    register unsigned char      *right;         /* right key value */
    Two                         i;              /* index for # of key parts */
    Two                         j;              /* temporary variable */
    Two                         kpartSize;      /* size of the current kpart */
    Two                         len1, len2;	/* string length */
    Two_Invariable              s1, s2;         /* 2-byte short values */
    Four_Invariable             i1, i2;         /* 4-byte int values */
    Four_Invariable             l1, l2;         /* 4-byte long values */
    Eight_Invariable            ll1, ll2;       /* 8-byte long long values */
    float                       f1, f2;         /* float values */
    double                      d1, d2;		/* double values */
    PageID                      pid1, pid2;	/* PageID values */
    OID                         oid1, oid2;     /* OID values */
    

    /* Error check whether using not supported functionality by EduBtM */
    for(i=0; i<kdesc->nparts; i++)
    {
        if(kdesc->kpart[i].type!=SM_INT && kdesc->kpart[i].type!=SM_VARSTRING)
            ERR(eNOTSUPPORTED_EDUBTM);
    }

    left = (unsigned char*)&(key1->val[0]);
    right = (unsigned char*)&(key2->val[0]);
    for(i = 0;i < kdesc->nparts;i++) {
        switch (kdesc->kpart[i].type) {
            case SM_SHORT:
                memcpy((char*)&s1, (char*)left, sizeof(Two_Invariable)); 
                memcpy((char*)&s2, (char*)right, sizeof(Two_Invariable));

                if (s1 > s2) return(GREAT);
                else if(s1 < s2) return(LESS);

                kpartSize = sizeof(Two_Invariable); 
                break;
            case SM_INT:
                memcpy((char*)&i1, (char*)left, sizeof(Four_Invariable)); 
                memcpy((char*)&i2, (char*)right, sizeof(Four_Invariable));

                if (i1 > i2) return(GREAT);
                else if (i1 < i2) return(LESS);

                kpartSize = sizeof(Four_Invariable); 

                break;
            case SM_LONG:
                memcpy((char*)&l1, (char*)left, sizeof(Four_Invariable)); 
                memcpy((char*)&l2, (char*)right, sizeof(Four_Invariable)); 

                if (l1 > l2) return(GREAT);
                else if (l1 < l2) return(LESS);

                kpartSize = sizeof(Four_Invariable); 
                break;

            case SM_LONG_LONG:
                memcpy((char*)&ll1, (char*)left, sizeof(Eight_Invariable)); 
                memcpy((char*)&ll2, (char*)right, sizeof(Eight_Invariable)); 

                if (ll1 > ll2) return(GREAT);
                else if (ll1 < ll2) return(LESS);

                kpartSize = sizeof(Eight_Invariable); 

                break;
            case SM_FLOAT:
                memcpy((char*)&f1, (char*)left, sizeof(float));
                memcpy((char*)&f2, (char*)right, sizeof(float));

                if (f1 > f2) return(GREAT);
                else if (f1 < f2) return(LESS);

                kpartSize = sizeof(float);
                break;

            case SM_DOUBLE:
                memcpy((char*)&d1, (char*)left, sizeof(double));
                memcpy((char*)&d2, (char*)right, sizeof(double));

                if (d1 > d2) return(GREAT);
                else if (d1 < d2) return(LESS);

                kpartSize = sizeof(double);

                break;
            case SM_STRING:
                for (j = 0; j < kdesc->kpart[i].length; j++, left++, right++) {
                    if (*left > *right) return(GREAT);	
                    else if (*left < *right) return(LESS);
                }

                kpartSize = 0;
                break;
            case SM_VARSTRING:
                memcpy((char*)&len1, (char*)left, sizeof(Two));
                memcpy((char*)&len2, (char*)right, sizeof(Two));

                left += sizeof(Two);
                right += sizeof(Two);

                for (j = MIN(len1, len2); j > 0; j--, left++, right++) {
                    if (*left > *right) return(GREAT);	
                    else if (*left < *right) return(LESS);
                }

                if (len1 > len2) return(GREAT);
                else if (len1 < len2) return(LESS);

                kpartSize = 0;	

                break;
            case SM_PAGEID:
            case SM_FILEID:
            case SM_INDEXID:
                memcpy((char*)&pid1, (char*)left, SM_PAGEID_SIZE);
                memcpy((char*)&pid2, (char*)right, SM_PAGEID_SIZE);

                if (pid1.volNo > pid2.volNo) return(GREAT);
                if (pid1.volNo < pid2.volNo) return(LESS);

                if (pid1.pageNo > pid2.pageNo) return(GREAT);
                if (pid1.pageNo < pid2.pageNo) return(LESS);

                kpartSize = sizeof(PageID);

                break;
            case SM_OID:
                memcpy((char*)&oid1, (char*)left, SM_OID_SIZE);
                memcpy((char*)&oid2, (char*)right, SM_OID_SIZE);

                if (oid1.volNo > oid2.volNo) return(GREAT);
                if (oid1.volNo < oid2.volNo) return(LESS);

                if (oid1.pageNo > oid2.pageNo) return(GREAT);
                if (oid1.pageNo < oid2.pageNo) return(LESS);
                
                if (oid1.slotNo > oid2.slotNo) return(GREAT);
                if (oid1.slotNo < oid2.slotNo) return(LESS);

                kpartSize = sizeof(OID);
                break;
        }

        left += kpartSize;
        right += kpartSize;
    }

    return(EQUAL);
    
}   /* edubtm_KeyCompare() */
