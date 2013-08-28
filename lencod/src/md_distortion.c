
/*!
 ***************************************************************************
 * \file md_distortion.c
 *
 * \brief
 *    Main macroblock mode decision functions and helpers
 *
 **************************************************************************
 */

#include <math.h>
#include <limits.h>
#include <float.h>

#include "global.h"
#include "rdopt_coding_state.h"
#include "mb_access.h"
#include "intrarefresh.h"
#include "image.h"
#include "transform8x8.h"
#include "ratectl.h"
#include "mode_decision.h"
#include "fmo.h"
#include "me_umhex.h"
#include "me_umhexsmp.h"
#include "macroblock.h"
#include "mv_search.h"
#include "md_distortion.h"


void setupDistortion(Slice *currSlice)
{

  currSlice->getDistortion = distortionSSE;


}

/*!
 ***********************************************************************
 * \brief
 *    compute generic SSE
 ***********************************************************************
 */
int64 compute_SSE(imgpel **imgRef, imgpel **imgSrc, int xRef, int xSrc, int ySize, int xSize)
{
  int i, j;
  imgpel *lineRef, *lineSrc;
  int64 distortion = 0;

  for (j = 0; j < ySize; j++)
  {
    lineRef = &imgRef[j][xRef];    
    lineSrc = &imgSrc[j][xSrc];

    for (i = 0; i < xSize; i++)
      distortion += iabs2( *lineRef++ - *lineSrc++ );
  }
  return distortion;
}

int64 compute_SSE_cr(imgpel **imgRef, imgpel **imgSrc, int xRef, int xSrc, int ySize, int xSize)
{
  int i, j;
  imgpel *lineRef, *lineSrc;
  int64 distortion = 0;

  for (j = 0; j < ySize; j++)
  {
    lineRef = &imgRef[j][xRef];    
    lineSrc = &imgSrc[j][xSrc];

    for (i = 0; i < xSize; i++)
      distortion += iabs2( *lineRef++ - *lineSrc++ );
  }

  return distortion;
}

/*!
 ***********************************************************************
 * \brief
 *    compute 16x16 SSE
 ***********************************************************************
 */
int64 compute_SSE16x16(imgpel **imgRef, imgpel **imgSrc, int xRef, int xSrc)
{
  int i, j;
  imgpel *lineRef, *lineSrc;
  int64 distortion = 0;

  for (j = 0; j < MB_BLOCK_SIZE; j++)
  {
    lineRef = &imgRef[j][xRef];    
    lineSrc = &imgSrc[j][xSrc];

    for (i = 0; i < MB_BLOCK_SIZE; i++)
      distortion += iabs2( *lineRef++ - *lineSrc++ );
  }
  return distortion;
}

/*!
 ***********************************************************************
 * \brief
 *    compute 8x8 SSE
 ***********************************************************************
 */
int64 compute_SSE8x8(imgpel **imgRef, imgpel **imgSrc, int xRef, int xSrc)
{
  int i, j;
  imgpel *lineRef, *lineSrc;
  int64 distortion = 0;

  for (j = 0; j < BLOCK_SIZE_8x8; j++)
  {
    lineRef = &imgRef[j][xRef];    
    lineSrc = &imgSrc[j][xSrc];

    for (i = 0; i < BLOCK_SIZE_8x8; i++)
      distortion += iabs2( *lineRef++ - *lineSrc++ );
  }
  return distortion;
}


/*!
 ***********************************************************************
 * \brief
 *    compute 4x4 SSE
 ***********************************************************************
 */
int64 compute_SSE4x4(imgpel **imgRef, imgpel **imgSrc, int xRef, int xSrc)
{
  int i, j;
  imgpel *lineRef, *lineSrc;
  int64 distortion = 0;

  for (j = 0; j < BLOCK_SIZE; j++)
  {
    lineRef = &imgRef[j][xRef];    
    lineSrc = &imgSrc[j][xSrc];

    for (i = 0; i < BLOCK_SIZE; i++)
      distortion += iabs2( *lineRef++ - *lineSrc++ );
  }

  return distortion;
}

/*!
*************************************************************************************
* \brief
*    SSE distortion calculation for a macroblock
*************************************************************************************
*/
int64 distortionSSE(Macroblock *currMB) 
{
  ImageParameters *p_Img = currMB->p_Img;
  InputParameters *p_Inp = currMB->p_Inp;
  int64 distortionY = 0;
  int64 distortionCr[2] = {0, 0};

  // LUMA
  distortionY = compute_SSE16x16(&p_Img->pCurImg[currMB->opix_y], &p_Img->enc_picture->p_curr_img[currMB->pix_y], currMB->pix_x, currMB->pix_x);

  // CHROMA
  if ((p_Img->yuv_format != YUV400) && !IS_INDEPENDENT(p_Inp))
  {
    distortionCr[0] = compute_SSE_cr(&p_Img->pImgOrg[1][currMB->opix_c_y], &p_Img->enc_picture->imgUV[0][currMB->pix_c_y], currMB->pix_c_x, currMB->pix_c_x, p_Img->mb_cr_size_y, p_Img->mb_cr_size_x);
    distortionCr[1] = compute_SSE_cr(&p_Img->pImgOrg[2][currMB->opix_c_y], &p_Img->enc_picture->imgUV[1][currMB->pix_c_y], currMB->pix_c_x, currMB->pix_c_x, p_Img->mb_cr_size_y, p_Img->mb_cr_size_x);
  }

  return (int64)( distortionY * p_Inp->WeightY + distortionCr[0] * p_Inp->WeightCb + distortionCr[1] * p_Inp->WeightCr );
}


