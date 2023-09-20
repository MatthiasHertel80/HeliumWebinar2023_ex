
#include <stdio.h>
#include <RTE_Components.h>

#include "EventRecorder.h"              // Keil.ARM Compiler::Compiler:Event Recorder


#include "arm_math.h"                   // ARM::CMSIS:DSP


#include CMSIS_device_header

#define VISIB_ATTR __attribute__ ((noinline))

#define ROWS 16
#define COLS 16



VISIB_ATTR void mat_by_vec_mult_q7(const q7_t * pSrcMat, const q7_t * pSrcVec, q7_t * pDstVec,
                                   uint32_t numRows, uint32_t numCols)
{
    const q7_t     *pInMat;     /* input data matrix pointer of Q7 type */
    const q7_t     *pInVec;     /* input data vector pointer of Q7 type */
    uint32_t        i, colCnt;  /* loop counters */

    i = 0u;

    while (numRows > 0) {

        q31_t           sum = 0;
        pInVec = pSrcVec;
        pInMat = pSrcMat + i;

        colCnt = numCols;
        while (colCnt > 0) {
            sum += *pInMat++ * *pInVec++;
            colCnt--;
        }
        *pDstVec++ = (q7_t) (__SSAT((sum >> 7), 8));
        i = i + numCols;
        numRows--;
    }
}



VISIB_ATTR void mat_by_vec_mult_q7_mve(const q7_t * pSrcMat, const q7_t * pSrcVec, q7_t * pDstVec,
                                       uint32_t numRows, uint32_t numCols)
{
    const q7_t     *pInMat;
    const q7_t     *pInVec;     /* input data vector pointer of Q7 type */
    uint16_t        blkCnt;     /* loop counters */
    uint32_t        i;          /* loop counters */

    i = 0u;

    while (numRows > 0) {
        q31_t           acc;
        q7x16_t         vecInMat, vecInVec;

        pInVec = pSrcVec;
        pInMat = pSrcMat + i;

        acc = 0L;

        blkCnt = numCols >> 4;
        while (blkCnt > 0U) {
            vecInMat = vld1q(pInMat);
            pInMat += 16;
            vecInVec = vld1q(pInVec);
            pInVec += 16;

            acc = vmladavaq(acc, vecInVec, vecInMat);
            blkCnt--;
        }

        blkCnt = numCols & 0xF;
        if (blkCnt > 0U) {
            mve_pred16_t    p0 = vctp8q(blkCnt);

            vecInMat = vld1q(pInMat);
            vecInVec = vldrbq_z_s8(pInVec, p0);
            acc = vmladavaq(acc, vecInVec, vecInMat);
        }
        *pDstVec++ = __SSAT(acc >> 7, 8);
        i = i + numCols;
        numRows--;
    }
}



VISIB_ATTR void mat_by_vec_mult_q7_mve_tp(const q7_t * pSrcMat, const q7_t * pSrcVec,
                                          q7_t * pDstVec, uint32_t numRows, uint32_t numCols)
{
    const q7_t     *pInMat;
    const q7_t     *pInVec;     /* input data vector pointer of Q7 type */
    int32_t         blkCnt;     /* loop counters */
    uint32_t        i;          /* loop counters */

    i = 0u;

    while (numRows > 0) {
        q31_t           acc;
        q7x16_t         vecInMat, vecInVec;

        pInVec = pSrcVec;
        pInMat = pSrcMat + i;

        acc = 0L;
        blkCnt = numCols;
        while (blkCnt > 0) {
            mve_pred16_t    p = vctp8q(blkCnt);

            vecInMat = vld1q_z(pInMat, p);
            pInMat += 16;
            vecInVec = vld1q_z(pInVec, p);
            pInVec += 16;

            acc = vmladavaq_p(acc, vecInVec, vecInMat, p);
            blkCnt -= 16;
        }

        *pDstVec++ = __SSAT(acc >> 7, 8);
        i = i + numCols;
        numRows--;
    }
}


/* Define main entry point.  */
int main (void)
{
    /* Configure Systick for each millisec */
    SysTick_Config(SystemCoreClock/1000);
	
	  EventRecorderInitialize (EventRecordAll, 1);

	    int             nbRows = ROWS;
    int             nbCols = COLS;

    __ALIGNED(8) int8_t          srcMatrix[ROWS * COLS];
    __ALIGNED(8) int8_t          srcVector[COLS];

    __ALIGNED(8) int8_t          dstVectorBaseLine[ROWS] = { 0 };
    __ALIGNED(8) int8_t          dstVectorHelium[ROWS] = { 0 };
    __ALIGNED(8) int8_t          dstVectorCmsisDsp[ROWS] = { 0 };

    /* fill source matrix / vector */
    for (int i = 0; i < nbRows * nbCols; i++)
        srcMatrix[i] = (int8_t) i;      /* will wrap after +127 */

    for (int i = 0; i < nbCols; i++)
        srcVector[i] = (int8_t) i;

while (1) {		
		
		 EventStartA (0);        
		 mat_by_vec_mult_q7(srcMatrix, srcVector, dstVectorBaseLine, nbRows, nbCols);
		 EventStopA (0);
	
		 EventStartA (1);		
		 mat_by_vec_mult_q7_mve(srcMatrix, srcVector, dstVectorHelium, nbRows, nbCols);
		 EventStopA (1);
		 
		 EventStartA (2);
		 mat_by_vec_mult_q7_mve_tp(srcMatrix, srcVector, dstVectorHelium, nbRows, nbCols);
	   EventStopA (2);
	
	   EventStartA(3);
	   arm_matrix_instance_q7 SrcMat;
     SrcMat.numRows = nbRows;
     SrcMat.numCols = nbCols;
     SrcMat.pData = srcMatrix;
     arm_mat_vec_mult_q7(&SrcMat, srcVector, dstVectorCmsisDsp);
	   EventStopA(3);
}
		

}
