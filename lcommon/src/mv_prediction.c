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
 *      - Karsten Sühring          <suehring@hhi.de>
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
  short mva =0;
  short mvb =0;
  short mvc =0;
  short L =0;
  short pmva_b44x1[4][2];
  short pmva_b44y1[4][2];
  short pmva_b44x2[4][2];
  short pmva_b44y2[4][2];
  short pmva_c44x1[4][2];
  short pmva_c44y1[4][2];
  short pmva_c44x2[4][2];
  short pmva_c44y2[4][2];
  short pmva_d44x1[4];
  short pmva_d44y1[4];
  short pmva_d44x2[4];
  short pmva_d44y2[4];
  short pmva44x[4][2];
  short pmva44y[4][2];
  int b;
  short pmvabx[2];
  short pmvaby[2];
  short pmvacx[2];
  short pmvacy[2];
  short pmvadx;
  short pmvady;
  
  short pmvb_b44x1[4][2];
  short pmvb_b44y1[4][2];
  short pmvb_b44x2[4][2];
  short pmvb_b44y2[4][2];
  short pmvb_c44x1[4][2];
  short pmvb_c44y1[4][2];
  short pmvb_c44x2[4][2];
  short pmvb_c44y2[4][2];
  short pmvb_d44x1[4];
  short pmvb_d44y1[4];
  short pmvb_d44x2[4];
  short pmvb_d44y2[4];
  short pmvb44x[4][2];
  short pmvb44y[4][2];
  short pmvbbx[2];
  short pmvbby[2];
  short pmvbcx[2];
  short pmvbcy[2];
  short pmvbdx;
  short pmvbdy;
  
  short pmvc_b44x1[4][2];
  short pmvc_b44y1[4][2];
  short pmvc_b44x2[4][2];
  short pmvc_b44y2[4][2];
  short pmvc_c44x1[4][2];
  short pmvc_c44y1[4][2];
  short pmvc_c44x2[4][2];
  short pmvc_c44y2[4][2];
  short pmvc_d44x1[4];
  short pmvc_d44y1[4];
  short pmvc_d44x2[4];
  short pmvc_d44y2[4];
  short pmvc44x[4][2];
  short pmvc44y[4][2];
  short pmvcbx[2];
  short pmvcby[2];
  short pmvccx[2];
  short pmvccy[2];
  short pmvcdx;
  short pmvcdy;
 
  //int count=0;
  Slice *currSlice = currMB->p_slice;
  int mbAddrX=currMB->mbAddrX;
  
 /* for(i=0;i<4;i++){
	  for(j=0;j<4;j++){
		currSlice->all_mymv[mbAddrX][i][j][0]=0;
		currSlice->all_mymv[mbAddrX][i][j][1]=0;
		}	
    }*/
	
  if(currMB->mbAddrX!=0){	
	if(block[0].available){
		pmva_b44x1[0][0] = (currSlice->all_mymv[mbAddrX-1][0][0][0]+currSlice->all_mymv[mbAddrX-1][0][1][0])/2;
		pmva_b44y1[0][1] = (currSlice->all_mymv[mbAddrX-1][0][0][1]+currSlice->all_mymv[mbAddrX-1][0][1][1])/2;
		pmva_b44x2[0][0] = (currSlice->all_mymv[mbAddrX-1][1][0][0]+currSlice->all_mymv[mbAddrX-1][1][0][0])/2;
		pmva_b44y2[0][1] = (currSlice->all_mymv[mbAddrX-1][1][1][1]+currSlice->all_mymv[mbAddrX-1][1][1][1])/2;
		pmva_c44x1[0][0] = (currSlice->all_mymv[mbAddrX-1][0][0][0]+currSlice->all_mymv[mbAddrX-1][1][0][0])/2;
		pmva_c44y1[0][1] = (currSlice->all_mymv[mbAddrX-1][0][0][1]+currSlice->all_mymv[mbAddrX-1][1][0][1])/2;
		pmva_c44x2[0][0] = (currSlice->all_mymv[mbAddrX-1][0][1][0]+currSlice->all_mymv[mbAddrX-1][1][1][0])/2;
		pmva_c44y2[0][1] = (currSlice->all_mymv[mbAddrX-1][0][1][1]+currSlice->all_mymv[mbAddrX-1][1][1][1])/2;	
		
		pmva_b44x1[1][0] = (currSlice->all_mymv[mbAddrX-1][2][0][0]+currSlice->all_mymv[mbAddrX-1][2][1][0])/2;
		pmva_b44y1[1][1] = (currSlice->all_mymv[mbAddrX-1][2][0][1]+currSlice->all_mymv[mbAddrX-1][2][1][1])/2;
		pmva_b44x2[1][0] = (currSlice->all_mymv[mbAddrX-1][3][0][0]+currSlice->all_mymv[mbAddrX-1][3][1][0])/2;
		pmva_b44y2[1][1] = (currSlice->all_mymv[mbAddrX-1][3][0][1]+currSlice->all_mymv[mbAddrX-1][3][1][1])/2;
		pmva_c44x1[1][0] = (currSlice->all_mymv[mbAddrX-1][2][0][0]+currSlice->all_mymv[mbAddrX-1][3][0][0])/2;
		pmva_c44y1[1][1] = (currSlice->all_mymv[mbAddrX-1][2][0][1]+currSlice->all_mymv[mbAddrX-1][3][0][1])/2;
		pmva_c44x2[1][0] = (currSlice->all_mymv[mbAddrX-1][2][1][0]+currSlice->all_mymv[mbAddrX-1][3][1][0])/2;
		pmva_c44y2[1][1] = (currSlice->all_mymv[mbAddrX-1][2][1][1]+currSlice->all_mymv[mbAddrX-1][3][1][1])/2;
		
		pmva_b44x1[2][0] = (currSlice->all_mymv[mbAddrX-1][0][2][0]+currSlice->all_mymv[mbAddrX-1][0][3][0])/2;
		pmva_b44y1[2][1] = (currSlice->all_mymv[mbAddrX-1][0][2][1]+currSlice->all_mymv[mbAddrX-1][0][3][1])/2;
		pmva_b44x2[2][0] = (currSlice->all_mymv[mbAddrX-1][1][2][0]+currSlice->all_mymv[mbAddrX-1][1][3][0])/2;
		pmva_b44y2[2][1] = (currSlice->all_mymv[mbAddrX-1][1][2][1]+currSlice->all_mymv[mbAddrX-1][1][3][1])/2;
		pmva_c44x1[2][0] = (currSlice->all_mymv[mbAddrX-1][0][2][0]+currSlice->all_mymv[mbAddrX-1][1][2][0])/2;
		pmva_c44y1[2][1] = (currSlice->all_mymv[mbAddrX-1][0][2][1]+currSlice->all_mymv[mbAddrX-1][1][2][1])/2;
		pmva_c44x2[2][0] = (currSlice->all_mymv[mbAddrX-1][0][3][0]+currSlice->all_mymv[mbAddrX-1][1][3][0])/2;
		pmva_c44y2[2][1] = (currSlice->all_mymv[mbAddrX-1][0][3][1]+currSlice->all_mymv[mbAddrX-1][1][3][1])/2;
		
		pmva_b44x1[3][0] = (currSlice->all_mymv[mbAddrX-1][2][2][0]+currSlice->all_mymv[mbAddrX-1][2][3][0])/2;
		pmva_b44y1[3][1] = (currSlice->all_mymv[mbAddrX-1][2][2][1]+currSlice->all_mymv[mbAddrX-1][2][3][1])/2;
		pmva_b44x2[3][0] = (currSlice->all_mymv[mbAddrX-1][3][2][0]+currSlice->all_mymv[mbAddrX-1][3][3][0])/2;
		pmva_b44y2[3][1] = (currSlice->all_mymv[mbAddrX-1][3][2][1]+currSlice->all_mymv[mbAddrX-1][3][3][1])/2;
		pmva_c44x1[3][0] = (currSlice->all_mymv[mbAddrX-1][2][2][0]+currSlice->all_mymv[mbAddrX-1][3][2][0])/2;
		pmva_c44y1[3][1] = (currSlice->all_mymv[mbAddrX-1][2][2][1]+currSlice->all_mymv[mbAddrX-1][3][2][1])/2;
		pmva_c44x2[3][0] = (currSlice->all_mymv[mbAddrX-1][2][3][0]+currSlice->all_mymv[mbAddrX-1][3][3][0])/2;
		pmva_c44y2[3][1] = (currSlice->all_mymv[mbAddrX-1][2][3][1]+currSlice->all_mymv[mbAddrX-1][3][3][1])/2;
  
  	for(b=0;b<4;b++){
	
		
		
		pmva_d44x1[b] = (pmva_b44x1[b][0]+pmva_c44x1[b][0]);
		pmva_d44y1[b] = (pmva_b44y1[b][1]+pmva_c44y1[b][1]);
		pmva_d44x2[b] = (pmva_b44x2[b][0]+pmva_c44x2[b][0]);
		pmva_d44y2[b] = (pmva_b44y2[b][1]+pmva_c44y2[b][1]);
	//printf("y1[%d][0]%d\n",b,pmva_d44y1[b]);
	//printf("y2[%d][1]%d\n",b,pmva_d44y2[b]);
	
	
    
		
		pmva44x[b][0] = (pmva_d44x1[b]+pmva_d44x2[b])/4;
		pmva44y[b][1] = (pmva_d44y1[b]+pmva_d44y2[b])/4;
		//first finish
	//printf("pmva44x[%d][1]=%d %d %d\n",b,pmva44x[b][0],(pmva_d44x1[b]+pmva_d44x2[b]),(pmva_d44x1[b]+pmva_d44x2[b])/4);
	//printf("pmva44y[%d][1]=%d %d %d\n",b,pmva44y[b][1],(pmva_d44y1[b]+pmva_d44y2[b]),(pmva_d44y1[b]+pmva_d44y2[b])/4);



	}
	//system("pause");
	pmvabx[0]=(pmva44x[0][0]+pmva44x[2][0])/2;
	pmvaby[0]=(pmva44y[0][1]+pmva44y[2][1])/2;
	pmvabx[1]=(pmva44x[1][0]+pmva44x[3][0])/2;
	pmvaby[1]=(pmva44y[1][1]+pmva44y[3][1])/2;
	
	
	pmvacx[0]=(pmva44x[0][0]+pmva44x[1][0])/2;
	pmvacy[0]=(pmva44y[0][1]+pmva44y[1][1])/2;
	pmvacx[1]=(pmva44x[2][0]+pmva44x[3][0])/2;
	pmvacy[1]=(pmva44y[2][1]+pmva44y[3][1])/2;
  
    pmvadx=(pmvabx[0]+pmvacx[0]+pmvabx[1]+pmvacx[1])/4;
	pmvady=(pmvaby[0]+pmvacy[0]+pmvaby[1]+pmvacy[1])/4;
	}
	//end a
	if(block[1].available){
		pmvb_b44x1[0][0] = (currSlice->all_mymv[mbAddrX-22][0][0][0]+currSlice->all_mymv[mbAddrX-22][0][1][0])/2;
		pmvb_b44y1[0][1] = (currSlice->all_mymv[mbAddrX-22][0][0][1]+currSlice->all_mymv[mbAddrX-22][0][1][1])/2;
		pmvb_b44x2[0][0] = (currSlice->all_mymv[mbAddrX-22][1][0][0]+currSlice->all_mymv[mbAddrX-22][1][0][0])/2;
		pmvb_b44y2[0][1] = (currSlice->all_mymv[mbAddrX-22][1][1][1]+currSlice->all_mymv[mbAddrX-22][1][1][1])/2;
		pmvb_c44x1[0][0] = (currSlice->all_mymv[mbAddrX-22][0][0][0]+currSlice->all_mymv[mbAddrX-22][1][0][0])/2;
		pmvb_c44y1[0][1] = (currSlice->all_mymv[mbAddrX-22][0][0][1]+currSlice->all_mymv[mbAddrX-22][1][0][1])/2;
		pmvb_c44x2[0][0] = (currSlice->all_mymv[mbAddrX-22][0][1][0]+currSlice->all_mymv[mbAddrX-22][1][1][0])/2;
		pmvb_c44y2[0][1] = (currSlice->all_mymv[mbAddrX-22][0][1][1]+currSlice->all_mymv[mbAddrX-22][1][1][1])/2;	
		
		pmvb_b44x1[1][0] = (currSlice->all_mymv[mbAddrX-22][2][0][0]+currSlice->all_mymv[mbAddrX-22][2][1][0])/2;
		pmvb_b44y1[1][1] = (currSlice->all_mymv[mbAddrX-22][2][0][1]+currSlice->all_mymv[mbAddrX-22][2][1][1])/2;
		pmvb_b44x2[1][0] = (currSlice->all_mymv[mbAddrX-22][3][0][0]+currSlice->all_mymv[mbAddrX-22][3][1][0])/2;
		pmvb_b44y2[1][1] = (currSlice->all_mymv[mbAddrX-22][3][0][1]+currSlice->all_mymv[mbAddrX-22][3][1][1])/2;
		pmvb_c44x1[1][0] = (currSlice->all_mymv[mbAddrX-22][2][0][0]+currSlice->all_mymv[mbAddrX-22][3][0][0])/2;
		pmvb_c44y1[1][1] = (currSlice->all_mymv[mbAddrX-22][2][0][1]+currSlice->all_mymv[mbAddrX-22][3][0][1])/2;
		pmvb_c44x2[1][0] = (currSlice->all_mymv[mbAddrX-22][2][1][0]+currSlice->all_mymv[mbAddrX-22][3][1][0])/2;
		pmvb_c44y2[1][1] = (currSlice->all_mymv[mbAddrX-22][2][1][1]+currSlice->all_mymv[mbAddrX-22][3][1][1])/2;
		
		pmvb_b44x1[2][0] = (currSlice->all_mymv[mbAddrX-22][0][2][0]+currSlice->all_mymv[mbAddrX-22][0][3][0])/2;
		pmvb_b44y1[2][1] = (currSlice->all_mymv[mbAddrX-22][0][2][1]+currSlice->all_mymv[mbAddrX-22][0][3][1])/2;
		pmvb_b44x2[2][0] = (currSlice->all_mymv[mbAddrX-22][1][2][0]+currSlice->all_mymv[mbAddrX-22][1][3][0])/2;
		pmvb_b44y2[2][1] = (currSlice->all_mymv[mbAddrX-22][1][2][1]+currSlice->all_mymv[mbAddrX-22][1][3][1])/2;
		pmvb_c44x1[2][0] = (currSlice->all_mymv[mbAddrX-22][0][2][0]+currSlice->all_mymv[mbAddrX-22][1][2][0])/2;
		pmvb_c44y1[2][1] = (currSlice->all_mymv[mbAddrX-22][0][2][1]+currSlice->all_mymv[mbAddrX-22][1][2][1])/2;
		pmvb_c44x2[2][0] = (currSlice->all_mymv[mbAddrX-22][0][3][0]+currSlice->all_mymv[mbAddrX-22][1][3][0])/2;
		pmvb_c44y2[2][1] = (currSlice->all_mymv[mbAddrX-22][0][3][1]+currSlice->all_mymv[mbAddrX-22][1][3][1])/2;
		
		pmvb_b44x1[3][0] = (currSlice->all_mymv[mbAddrX-22][2][2][0]+currSlice->all_mymv[mbAddrX-22][2][3][0])/2;
		pmvb_b44y1[3][1] = (currSlice->all_mymv[mbAddrX-22][2][2][1]+currSlice->all_mymv[mbAddrX-22][2][3][1])/2;
		pmvb_b44x2[3][0] = (currSlice->all_mymv[mbAddrX-22][3][2][0]+currSlice->all_mymv[mbAddrX-22][3][3][0])/2;
		pmvb_b44y2[3][1] = (currSlice->all_mymv[mbAddrX-22][3][2][1]+currSlice->all_mymv[mbAddrX-22][3][3][1])/2;
		pmvb_c44x1[3][0] = (currSlice->all_mymv[mbAddrX-22][2][2][0]+currSlice->all_mymv[mbAddrX-22][3][2][0])/2;
		pmvb_c44y1[3][1] = (currSlice->all_mymv[mbAddrX-22][2][2][1]+currSlice->all_mymv[mbAddrX-22][3][2][1])/2;
		pmvb_c44x2[3][0] = (currSlice->all_mymv[mbAddrX-22][2][3][0]+currSlice->all_mymv[mbAddrX-22][3][3][0])/2;
		pmvb_c44y2[3][1] = (currSlice->all_mymv[mbAddrX-22][2][3][1]+currSlice->all_mymv[mbAddrX-22][3][3][1])/2;
  
  	for(b=0;b<4;b++){
	
		
		
		pmvb_d44x1[b] = (pmvb_b44x1[b][0]+pmvb_c44x1[b][0]);
		pmvb_d44y1[b] = (pmvb_b44y1[b][1]+pmvb_c44y1[b][1]);
		pmvb_d44x2[b] = (pmvb_b44x2[b][0]+pmvb_c44x2[b][0]);
		pmvb_d44y2[b] = (pmvb_b44y2[b][1]+pmvb_c44y2[b][1]);
		
		
	
    
		
		pmvb44x[b][0] = (pmvb_d44x1[b]+pmvb_d44x2[b])/4;
		pmvb44y[b][1] = (pmvb_d44y1[b]+pmvb_d44y2[b])/4;
		//first finish
	//printf("pmvb44x[%d][1]=%d %d %d\n",b,pmvb44x[b][0],(pmvb_d44x1[b]+pmvb_d44x2[b]),(pmvb_d44x1[b]+pmvb_d44x2[b])/4);
	//printf("pmvb44y[%d][1]=%d %d %d\n",b,pmvb44y[b][1],(pmvb_d44y1[b]+pmvb_d44y2[b]),(pmvb_d44y1[b]+pmvb_d44y2[b])/4);
	}
	
	pmvbbx[0]=(pmvb44x[0][0]+pmvb44x[2][0])/2;
	pmvbby[0]=(pmvb44y[0][1]+pmvb44y[2][1])/2;
	pmvbbx[1]=(pmvb44x[1][0]+pmvb44x[3][0])/2;
	pmvbby[1]=(pmvb44y[1][1]+pmvb44y[3][1])/2;
	
	pmvbcx[0]=(pmvb44x[0][0]+pmvb44x[1][0])/2;
	pmvbcy[0]=(pmvb44y[0][1]+pmvb44y[1][1])/2;
	pmvbcx[1]=(pmvb44x[2][0]+pmvb44x[3][0])/2;
	pmvbcy[1]=(pmvb44y[2][1]+pmvb44y[3][1])/2;
  
    pmvbdx=(pmvbbx[0]+pmvbcx[0]+pmvbbx[1]+pmvbcx[1])/4;
	pmvbdy=(pmvbby[0]+pmvbcy[0]+pmvbby[1]+pmvbcy[1])/4;
	}
	//end b
	if(block[2].available){
	    pmvc_b44x1[0][0] = (currSlice->all_mymv[mbAddrX-21][0][0][0]+currSlice->all_mymv[mbAddrX-21][0][1][0])/2;
		pmvc_b44y1[0][1] = (currSlice->all_mymv[mbAddrX-21][0][0][1]+currSlice->all_mymv[mbAddrX-21][0][1][1])/2;
		pmvc_b44x2[0][0] = (currSlice->all_mymv[mbAddrX-21][1][0][0]+currSlice->all_mymv[mbAddrX-21][1][0][0])/2;
		pmvc_b44y2[0][1] = (currSlice->all_mymv[mbAddrX-21][1][1][1]+currSlice->all_mymv[mbAddrX-21][1][1][1])/2;
		pmvc_c44x1[0][0] = (currSlice->all_mymv[mbAddrX-21][0][0][0]+currSlice->all_mymv[mbAddrX-21][1][0][0])/2;
		pmvc_c44y1[0][1] = (currSlice->all_mymv[mbAddrX-21][0][0][1]+currSlice->all_mymv[mbAddrX-21][1][0][1])/2;
		pmvc_c44x2[0][0] = (currSlice->all_mymv[mbAddrX-21][0][1][0]+currSlice->all_mymv[mbAddrX-21][1][1][0])/2;
		pmvc_c44y2[0][1] = (currSlice->all_mymv[mbAddrX-21][0][1][1]+currSlice->all_mymv[mbAddrX-21][1][1][1])/2;	
		
		pmvc_b44x1[1][0] = (currSlice->all_mymv[mbAddrX-21][2][0][0]+currSlice->all_mymv[mbAddrX-21][2][1][0])/2;
		pmvc_b44y1[1][1] = (currSlice->all_mymv[mbAddrX-21][2][0][1]+currSlice->all_mymv[mbAddrX-21][2][1][1])/2;
		pmvc_b44x2[1][0] = (currSlice->all_mymv[mbAddrX-21][3][0][0]+currSlice->all_mymv[mbAddrX-21][3][1][0])/2;
		pmvc_b44y2[1][1] = (currSlice->all_mymv[mbAddrX-21][3][0][1]+currSlice->all_mymv[mbAddrX-21][3][1][1])/2;
		pmvc_c44x1[1][0] = (currSlice->all_mymv[mbAddrX-21][2][0][0]+currSlice->all_mymv[mbAddrX-21][3][0][0])/2;
		pmvc_c44y1[1][1] = (currSlice->all_mymv[mbAddrX-21][2][0][1]+currSlice->all_mymv[mbAddrX-21][3][0][1])/2;
		pmvc_c44x2[1][0] = (currSlice->all_mymv[mbAddrX-21][2][1][0]+currSlice->all_mymv[mbAddrX-21][3][1][0])/2;
		pmvc_c44y2[1][1] = (currSlice->all_mymv[mbAddrX-21][2][1][1]+currSlice->all_mymv[mbAddrX-21][3][1][1])/2;
		
		pmvc_b44x1[2][0] = (currSlice->all_mymv[mbAddrX-21][0][2][0]+currSlice->all_mymv[mbAddrX-21][0][3][0])/2;
		pmvc_b44y1[2][1] = (currSlice->all_mymv[mbAddrX-21][0][2][1]+currSlice->all_mymv[mbAddrX-21][0][3][1])/2;
		pmvc_b44x2[2][0] = (currSlice->all_mymv[mbAddrX-21][1][2][0]+currSlice->all_mymv[mbAddrX-21][1][3][0])/2;
		pmvc_b44y2[2][1] = (currSlice->all_mymv[mbAddrX-21][1][2][1]+currSlice->all_mymv[mbAddrX-21][1][3][1])/2;
		pmvc_c44x1[2][0] = (currSlice->all_mymv[mbAddrX-21][0][2][0]+currSlice->all_mymv[mbAddrX-21][1][2][0])/2;
		pmvc_c44y1[2][1] = (currSlice->all_mymv[mbAddrX-21][0][2][1]+currSlice->all_mymv[mbAddrX-21][1][2][1])/2;
		pmvc_c44x2[2][0] = (currSlice->all_mymv[mbAddrX-21][0][3][0]+currSlice->all_mymv[mbAddrX-21][1][3][0])/2;
		pmvc_c44y2[2][1] = (currSlice->all_mymv[mbAddrX-21][0][3][1]+currSlice->all_mymv[mbAddrX-21][1][3][1])/2;
		
		pmvc_b44x1[3][0] = (currSlice->all_mymv[mbAddrX-21][2][2][0]+currSlice->all_mymv[mbAddrX-21][2][3][0])/2;
		pmvc_b44y1[3][1] = (currSlice->all_mymv[mbAddrX-21][2][2][1]+currSlice->all_mymv[mbAddrX-21][2][3][1])/2;
		pmvc_b44x2[3][0] = (currSlice->all_mymv[mbAddrX-21][3][2][0]+currSlice->all_mymv[mbAddrX-21][3][3][0])/2;
		pmvc_b44y2[3][1] = (currSlice->all_mymv[mbAddrX-21][3][2][1]+currSlice->all_mymv[mbAddrX-21][3][3][1])/2;
		pmvc_c44x1[3][0] = (currSlice->all_mymv[mbAddrX-21][2][2][0]+currSlice->all_mymv[mbAddrX-21][3][2][0])/2;
		pmvc_c44y1[3][1] = (currSlice->all_mymv[mbAddrX-21][2][2][1]+currSlice->all_mymv[mbAddrX-21][3][2][1])/2;
		pmvc_c44x2[3][0] = (currSlice->all_mymv[mbAddrX-21][2][3][0]+currSlice->all_mymv[mbAddrX-21][3][3][0])/2;
		pmvc_c44y2[3][1] = (currSlice->all_mymv[mbAddrX-21][2][3][1]+currSlice->all_mymv[mbAddrX-21][3][3][1])/2;
  
  	for(b=0;b<4;b++){
	
		
		
		pmvc_d44x1[b] = (pmvc_b44x1[b][0]+pmvc_c44x1[b][0]);
		pmvc_d44y1[b] = (pmvc_b44y1[b][1]+pmvc_c44y1[b][1]);
		pmvc_d44x2[b] = (pmvc_b44x2[b][0]+pmvc_c44x2[b][0]);
		pmvc_d44y2[b] = (pmvc_b44y2[b][1]+pmvc_c44y2[b][1]);
		
		
	
    
		
		pmvc44x[b][0] = (pmvc_d44x1[b]+pmvc_d44x2[b])/4;
		pmvc44y[b][1] = (pmvc_d44y1[b]+pmvc_d44y2[b])/4;
		//first finish
	//printf("pmvc44x[%d][1]=%d %d %d\n",b,pmvc44x[b][0],(pmvc_d44x1[b]+pmvc_d44x2[b]),(pmvc_d44x1[b]+pmvc_d44x2[b])/4);
	//printf("pmvc44y[%d][1]=%d %d %d\n",b,pmvc44y[b][1],(pmvc_d44y1[b]+pmvc_d44y2[b]),(pmvc_d44y1[b]+pmvc_d44y2[b])/4);
	}
	
	pmvcbx[0]=(pmvc44x[0][0]+pmvc44x[2][0])/2;
	pmvcby[0]=(pmvc44y[0][1]+pmvc44y[2][1])/2;
	pmvcbx[1]=(pmvc44x[1][0]+pmvc44x[3][0])/2;
	pmvcby[1]=(pmvc44y[1][1]+pmvc44y[3][1])/2;
	
	pmvccx[0]=(pmvc44x[0][0]+pmvc44x[1][0])/2;
	pmvccy[0]=(pmvc44y[0][1]+pmvc44y[1][1])/2;
	pmvccx[1]=(pmvc44x[2][0]+pmvc44x[3][0])/2;
	pmvccy[1]=(pmvc44y[2][1]+pmvc44y[3][1])/2;
  
    pmvcdx=(pmvcbx[0]+pmvccx[0]+pmvcbx[1]+pmvccx[1])/4;
	pmvcdy=(pmvcby[0]+pmvccy[0]+pmvcby[1]+pmvccy[1])/4;	
	
}
	
	if(currMB->mb_x==0){
		L=pmvbdx+pmvbdy; 
		pmv[0] = (short) pmvbdx;
		pmv[1] = (short) pmvbdy;
		} //boundary checked
	else if(currMB->mb_y==0) {
		L=pmvadx+pmvady; 
		pmv[0] = (short) pmvadx;
		pmv[1] = (short) pmvady;
		}
	else if(currMB->mb_x==21) {
		L=pmvadx+pmvady; 
		pmv[0] = (short) pmvadx;
		pmv[1] = (short) pmvady;
		}
	else
	{
	mva = pmvadx+pmvady;
	mvb = pmvbdx+pmvbdy; 
	mvc = pmvcdx+pmvcdy; 
	if(mva>=mvb && mva>=mvc) 
	{
		L=mva;
		pmv[0] = (short) pmvadx;
		pmv[1] = (short) pmvady;
		}
	if(mvb>=mvc && mvb>=mva) 
	{
		L=mvb;
		pmv[0] = (short) pmvbdx;
		pmv[1] = (short) pmvbdy;
		}
	if(mvc>=mva && mvc>=mvb)
	{
		L=mvc;
		pmv[0] = (short) pmvcdx;
		pmv[1] = (short) pmvcdy;
	}
	
	}
	
	//currMB->pmva =mva;
	//currMB->pmvb =mvb;
	//currMB->pmvc =mvc;
	currMB->L =L;
}
else{
	
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
    //mv_a = block[0].available ? tmp_mv[block[0].pos_y][block[0].pos_x][hv] : 0;
    //mv_b = block[1].available ? tmp_mv[block[1].pos_y][block[1].pos_x][hv] : 0;
    //mv_c = block[2].available ? tmp_mv[block[2].pos_y][block[2].pos_x][hv] : 0;

   
	
	mv_a = block[0].available ? tmp_mv[block[0].pos_y][block[0].pos_x][hv] : 0;
    mv_b = block[1].available ? tmp_mv[block[1].pos_y][block[1].pos_x][hv] : 0;
    mv_c = block[2].available ? tmp_mv[block[2].pos_y][block[2].pos_x][hv] : 0;
	//printf ("mva=(%d )\n",mva);
	//printf ("mvb=(%d )\n",mvb);	
	//printf ("mvc=(%d )\n",mvc);
	/*
	if(currMB->mb_x==0)L=mvb;  //boundary checked
	else if(currMB->mb_y==0) L=mvb;
	else if(currMB->mb_x==21) L=mva;
	else
	{
	mva += mv_a;
	mvb += mv_b;
	mvc += mv_c;
	if(mva>=mvb && mvb>=mvc) L=mva;
	if(mvb>=mvc && mvc>=mva) L=mvb;
	if(mvc>=mva && mva>=mvb) L=mvc;
	}
	*/
//	printf ("L=(%d )\n",L);
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
	//currMB->pmva =mva;
	//currMB->pmvb =mvb;
	//currMB->pmvc =mvc;
	//currMB->L =L;
	}
	//printf ("mv=(%d , %d)\n",pmv[0],pmv[1]);
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
