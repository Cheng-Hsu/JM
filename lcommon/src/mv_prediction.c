/*!
 *************************************************************************************
 * \file mv_prediction.c
 *
 * \brief
 *    Motion Vector Prediction Functions
 *
 *  \author
 *      Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Alexis Michael Tourapis  <alexismt@ieee.org>
 *      - Karsten S�hring          <suehring@hhi.de>
 *************************************************************************************
 */

#include "global.h"

/*!
 ************************************************************************
 * \brief
 *    Get motion vector predictor
 ************************************************************************
 */
static void GetMotionVectorPredictorMBAFF (Macroblock *currMB, 
                                    PixelPos *block,        // <--> block neighbors
                                    short  pmv[2],
                                    short  ref_frame,
                                    char   **refPic,
                                    short  ***tmp_mv,
                                    int    mb_x,
                                    int    mb_y,
                                    int    blockshape_x,
                                    int    blockshape_y)
{
  int mv_a, mv_b, mv_c, pred_vec=0;
  int mvPredType, rFrameL, rFrameU, rFrameUR;
  int hv;
  ImageParameters *p_Img = currMB->p_Img;

  mvPredType = MVPRED_MEDIAN;


  if (currMB->mb_field)
  {
    rFrameL  = block[0].available
      ? (p_Img->mb_data[block[0].mb_addr].mb_field
      ? refPic[block[0].pos_y][block[0].pos_x]
    : refPic[block[0].pos_y][block[0].pos_x] * 2) : -1;
    rFrameU  = block[1].available
      ? (p_Img->mb_data[block[1].mb_addr].mb_field
      ? refPic[block[1].pos_y][block[1].pos_x]
    : refPic[block[1].pos_y][block[1].pos_x] * 2) : -1;
    rFrameUR = block[2].available
      ? (p_Img->mb_data[block[2].mb_addr].mb_field
      ? refPic[block[2].pos_y][block[2].pos_x]
    : refPic[block[2].pos_y][block[2].pos_x] * 2) : -1;
  }
  else
  {
    rFrameL = block[0].available
      ? (p_Img->mb_data[block[0].mb_addr].mb_field
      ? refPic[block[0].pos_y][block[0].pos_x] >>1
      : refPic[block[0].pos_y][block[0].pos_x]) : -1;
    rFrameU  = block[1].available
      ? (p_Img->mb_data[block[1].mb_addr].mb_field
      ? refPic[block[1].pos_y][block[1].pos_x] >>1
      : refPic[block[1].pos_y][block[1].pos_x]) : -1;
    rFrameUR = block[2].available
      ? (p_Img->mb_data[block[2].mb_addr].mb_field
      ? refPic[block[2].pos_y][block[2].pos_x] >>1
      : refPic[block[2].pos_y][block[2].pos_x]) : -1;
  }


  /* Prediction if only one of the neighbors uses the reference frame
  *  we are checking
  */
  if(rFrameL == ref_frame && rFrameU != ref_frame && rFrameUR != ref_frame)       
    mvPredType = MVPRED_L;
  else if(rFrameL != ref_frame && rFrameU == ref_frame && rFrameUR != ref_frame)  
    mvPredType = MVPRED_U;
  else if(rFrameL != ref_frame && rFrameU != ref_frame && rFrameUR == ref_frame)  
    mvPredType = MVPRED_UR;
  // Directional predictions
  if(blockshape_x == 8 && blockshape_y == 16)
  {
    if(mb_x == 0)
    {
      if(rFrameL == ref_frame)
        mvPredType = MVPRED_L;
    }
    else
    {
      if( rFrameUR == ref_frame)
        mvPredType = MVPRED_UR;
    }
  }
  else if(blockshape_x == 16 && blockshape_y == 8)
  {
    if(mb_y == 0)
    {
      if(rFrameU == ref_frame)
        mvPredType = MVPRED_U;
    }
    else
    {
      if(rFrameL == ref_frame)
        mvPredType = MVPRED_L;
    }
  }

  for (hv=0; hv < 2; hv++)
  {
    if (hv == 0)
    {
      mv_a = block[0].available ? tmp_mv[block[0].pos_y][block[0].pos_x][hv] : 0;
      mv_b = block[1].available ? tmp_mv[block[1].pos_y][block[1].pos_x][hv] : 0;
      mv_c = block[2].available ? tmp_mv[block[2].pos_y][block[2].pos_x][hv] : 0;
    }
    else
    {
      if (currMB->mb_field)
      {
        mv_a = block[0].available  ? p_Img->mb_data[block[0].mb_addr].mb_field
          ? tmp_mv[block[0].pos_y][block[0].pos_x][hv]
        : tmp_mv[block[0].pos_y][block[0].pos_x][hv] / 2
          : 0;
        mv_b = block[1].available  ? p_Img->mb_data[block[1].mb_addr].mb_field
          ? tmp_mv[block[1].pos_y][block[1].pos_x][hv]
        : tmp_mv[block[1].pos_y][block[1].pos_x][hv] / 2
          : 0;
        mv_c = block[2].available  ? p_Img->mb_data[block[2].mb_addr].mb_field
          ? tmp_mv[block[2].pos_y][block[2].pos_x][hv]
        : tmp_mv[block[2].pos_y][block[2].pos_x][hv] / 2
          : 0;
      }
      else
      {
        mv_a = block[0].available  ? p_Img->mb_data[block[0].mb_addr].mb_field
          ? tmp_mv[block[0].pos_y][block[0].pos_x][hv] * 2
          : tmp_mv[block[0].pos_y][block[0].pos_x][hv]
        : 0;
        mv_b = block[1].available  ? p_Img->mb_data[block[1].mb_addr].mb_field
          ? tmp_mv[block[1].pos_y][block[1].pos_x][hv] * 2
          : tmp_mv[block[1].pos_y][block[1].pos_x][hv]
        : 0;
        mv_c = block[2].available  ? p_Img->mb_data[block[2].mb_addr].mb_field
          ? tmp_mv[block[2].pos_y][block[2].pos_x][hv] * 2
          : tmp_mv[block[2].pos_y][block[2].pos_x][hv]
        : 0;
      }
    }

    switch (mvPredType)
    {
    case MVPRED_MEDIAN:
      if(!(block[1].available || block[2].available))
      {
        pred_vec = mv_a;
      }
      else
      {
        pred_vec = mv_a + mv_b + mv_c - imin(mv_a, imin(mv_b, mv_c)) - imax(mv_a, imax(mv_b ,mv_c));
      }
      break;
    case MVPRED_L:
      pred_vec = mv_a;
      break;
    case MVPRED_U:
      pred_vec = mv_b;
      break;
    case MVPRED_UR:
      pred_vec = mv_c;
      break;
    default:
      break;
    }

    pmv[hv] = (short) pred_vec;
  }
}

/*!
 ************************************************************************
 * \brief
 *    Get motion vector predictor
 ************************************************************************
 */
static void GetMotionVectorPredictorNormal (Macroblock *currMB, 
                                     PixelPos *block,      // <--> block neighbors
                                     short  pmv[2],
                                     short  ref_frame,
                                     char   **refPic,
                                     short  ***tmp_mv,
                                     int    mb_x,
                                     int    mb_y,
                                     int    blockshape_x,
                                     int    blockshape_y)
{
  int mv_a, mv_b, mv_c, pred_vec = 0;
  int mvPredType, rFrameL, rFrameU, rFrameUR;
  int hv;
  int mva =0;
  int mvb =0;
  int mvc =0;
  int L =0;
  //int count=0;

  mvPredType = MVPRED_MEDIAN;

  rFrameL    = block[0].available ? refPic[block[0].pos_y][block[0].pos_x] : -1;
  rFrameU    = block[1].available ? refPic[block[1].pos_y][block[1].pos_x] : -1;
  rFrameUR   = block[2].available ? refPic[block[2].pos_y][block[2].pos_x] : -1;
 
//  printf("ref_frame=%d\n",ref_frame);
  /* Prediction if only one of the neighbors uses the reference frame
  *  we are checking
  */
  if(rFrameL == ref_frame && rFrameU != ref_frame && rFrameUR != ref_frame)       
    mvPredType = MVPRED_L;
  else if(rFrameL != ref_frame && rFrameU == ref_frame && rFrameUR != ref_frame)  
    mvPredType = MVPRED_U;
  else if(rFrameL != ref_frame && rFrameU != ref_frame && rFrameUR == ref_frame)  
    mvPredType = MVPRED_UR;
  // Directional predictions
  if(blockshape_x == 8 && blockshape_y == 16)
  {
    if(mb_x == 0)
    {
      if(rFrameL == ref_frame)
        mvPredType = MVPRED_L;
    }
    else
    {
      if(rFrameUR == ref_frame)
        mvPredType = MVPRED_UR;
    }
  }
  else if(blockshape_x == 16 && blockshape_y == 8)
  {
    if(mb_y == 0)
    {
      if(rFrameU == ref_frame)
        mvPredType = MVPRED_U;
    }
    else
    {
      if(rFrameL == ref_frame)
        mvPredType = MVPRED_L;
    }
  }

  for (hv=0; hv < 2; hv++)
  {
    mv_a = block[0].available ? tmp_mv[block[0].pos_y][block[0].pos_x][hv] : 0;
    mv_b = block[1].available ? tmp_mv[block[1].pos_y][block[1].pos_x][hv] : 0;
    mv_c = block[2].available ? tmp_mv[block[2].pos_y][block[2].pos_x][hv] : 0;   
	mva += mv_a;
	mvb += mv_b;
	mvc += mv_c;
	if(mva>=mvb && mvb>=mvc) L=mva;
	if(mvb>=mvc && mvc>=mva) L=mvb;
	if(mvc>=mva && mva>=mvb) L=mvc;
    switch (mvPredType)
    {
    case MVPRED_MEDIAN:
      if(!(block[1].available || block[2].available))
      {
        pred_vec = mv_a;
      }
      else
      {
        pred_vec = mv_a + mv_b + mv_c - imin(mv_a, imin(mv_b, mv_c)) - imax(mv_a, imax(mv_b ,mv_c));
      }
      break;
    case MVPRED_L:
      pred_vec = mv_a;
      break;
    case MVPRED_U:
      pred_vec = mv_b;
      break;
    case MVPRED_UR:
      pred_vec = mv_c;
      break;
    default:
      break;
    }
	
    pmv[hv] = (short) pred_vec;
	 
  }
	currMB->pmva =mva;
	currMB->pmvb =mvb;
	currMB->pmvc =mvc;
	currMB->L =L;
 // mvb=(tmp_mv[block[1].pos_y][block[1].pos_x][0])+(tmp_mv[block[1].pos_y][block[1].pos_x][1]);
  //mvc=(tmp_mv[block[1].pos_y][block[2].pos_x][0])+(tmp_mv[block[1].pos_y][block[2].pos_x][1]);
 /*if(currMB->mbAddrX==164){
		
	if((blockshape_x==16)&(blockshape_y==16)){
	printf ("mvPredType=%d\n", mvPredType);
	printf ("mb_x=%d\tmb_y=%d\n",mb_x,mb_y);
    printf ("blockshape_x=%d\tblockshape_y=%d\n",blockshape_x,blockshape_y);  
	printf ("mva=(%d )\n",mva);
	printf ("mvb=(%d )\n",mvb);	
	printf ("mvc=(%d )\n",mvc);
	printf ("L=(%d )\n",L);

	currMB->ccount=currMB->ccount+1;  printf ("count=%d\n",currMB->ccount); 
	//printf ("mv_a=(%d , %d)\n",tmp_mv[block[0].pos_y][block[0].pos_x][0],tmp_mv[block[0].pos_y][block[0].pos_x][1]);
	//printf ("mv_b=(%d , %d)\n",tmp_mv[block[1].pos_y][block[1].pos_x][0],tmp_mv[block[1].pos_y][block[1].pos_x][1]);
	//printf ("mv_c=(%d , %d)\n",tmp_mv[block[1].pos_y][block[2].pos_x][0],tmp_mv[block[1].pos_y][block[2].pos_x][1]);
	printf ("mv=(%d , %d)\n",pmv[0],pmv[1]);
	printf ("=====================\n");
	  }system("pause");
	}
 */
}

void InitMotionVectorPrediction(Macroblock *currMB, int MbaffFrameFlag)
{
  if (MbaffFrameFlag)
    currMB->GetMVPredictor = GetMotionVectorPredictorMBAFF;
  else
    currMB->GetMVPredictor = GetMotionVectorPredictorNormal;
}
