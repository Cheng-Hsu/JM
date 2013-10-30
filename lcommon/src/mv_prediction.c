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
  float mva =0;
  float mvb =0;
  float mvc =0;
  float mvd =0;
  float L =0;
  float pmva_b44x1[4][2];
  float pmva_b44y1[4][2];
  float pmva_b44x2[4][2];
  float pmva_b44y2[4][2];
  float pmva_c44x1[4][2];
  float pmva_c44y1[4][2];
  float pmva_c44x2[4][2];
  float pmva_c44y2[4][2];
  float pmva_d44x1[4];
  float pmva_d44y1[4];
  float pmva_d44x2[4];
  float pmva_d44y2[4];
  float pmva44x[4][2];
  float pmva44y[4][2];

  float pmvabx[2];
  float pmvaby[2];
  float pmvacx[2];
  float pmvacy[2];
  float pmvadx;
  float pmvady;
  
  float pmvb_b44x1[4][2];
  float pmvb_b44y1[4][2];
  float pmvb_b44x2[4][2];
  float pmvb_b44y2[4][2];
  float pmvb_c44x1[4][2];
  float pmvb_c44y1[4][2];
  float pmvb_c44x2[4][2];
  float pmvb_c44y2[4][2];
  float pmvb_d44x1[4];
  float pmvb_d44y1[4];
  float pmvb_d44x2[4];
  float pmvb_d44y2[4];
  float pmvb44x[4][2];
  float pmvb44y[4][2];
  float pmvbbx[2];
  float pmvbby[2];
  float pmvbcx[2];
  float pmvbcy[2];
  float pmvbdx;
  float pmvbdy;
  
  float pmvc_b44x1[4][2];
  float pmvc_b44y1[4][2];
  float pmvc_b44x2[4][2];
  float pmvc_b44y2[4][2];
  float pmvc_c44x1[4][2];
  float pmvc_c44y1[4][2];
  float pmvc_c44x2[4][2];
  float pmvc_c44y2[4][2];
  float pmvc_d44x1[4];
  float pmvc_d44y1[4];
  float pmvc_d44x2[4];
  float pmvc_d44y2[4];
  float pmvc44x[4][2];
  float pmvc44y[4][2];
  float pmvcbx[2];
  float pmvcby[2];
  float pmvccx[2];
  float pmvccy[2];
  float pmvcdx;
  float pmvcdy;
  
  float pmvd_b44x1[4][2];
  float pmvd_b44y1[4][2];
  float pmvd_b44x2[4][2];
  float pmvd_b44y2[4][2];
  float pmvd_c44x1[4][2];
  float pmvd_c44y1[4][2];
  float pmvd_c44x2[4][2];
  float pmvd_c44y2[4][2];
  float pmvd_d44x1[4];
  float pmvd_d44y1[4];
  float pmvd_d44x2[4];
  float pmvd_d44y2[4];
  float pmvd44x[4][2];
  float pmvd44y[4][2];
  float pmvdbx[2];
  float pmvdby[2];
  float pmvdcx[2];
  float pmvdcy[2];
  float pmvddx;
  float pmvddy;
  
  int a=0;
  int b=0;
  int c=0;
  int d=0;
  int a1=1;
  int b1=1;
  int c1=1;
  int d1=1;
  //int count=0;
  Slice *currSlice = currMB->p_slice;
  int mbAddrX=currMB->mbAddrX;
  int frame_num=currSlice->frame_num;
 /* for(i=0;i<4;i++){
	  for(j=0;j<4;j++){
		currSlice->all_mymv[mbAddrX][i][j][0]=0;
		currSlice->all_mymv[mbAddrX][i][j][1]=0;
		}	
    }*/
	

	
		
	
  if((currMB->mbAddrX!=0 && currMB->mode0_flag==0)){	
	if(block[0].available){
		
	if(currSlice->mymv_best[frame_num][mbAddrX-1][0][0]==0){
		pmvadx=currSlice->all_mymv[frame_num][mbAddrX-1][0][4][0];
		pmvady=currSlice->all_mymv[frame_num][mbAddrX-1][0][4][1];
		a1=0;
	}
	
	if(currSlice->mymv_best[frame_num][mbAddrX-1][0][0]==1){
		pmvadx=currSlice->all_mymv[frame_num][mbAddrX-1][0][4][0];
		pmvady=currSlice->all_mymv[frame_num][mbAddrX-1][0][4][1];
		a1=0;
	}
	 if(currSlice->mymv_best[frame_num][mbAddrX-1][0][0]==2){
	
		pmvadx=(currSlice->all_mymv[frame_num][mbAddrX-1][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-1][2][4][0]+
				currSlice->all_mymv[frame_num][mbAddrX-1][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-1][0][6][0])/4;
		pmvady=(currSlice->all_mymv[frame_num][mbAddrX-1][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-1][2][4][1]+
				currSlice->all_mymv[frame_num][mbAddrX-1][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-1][0][6][1])/4;
		a1=0;
	}
	 if(currSlice->mymv_best[frame_num][mbAddrX-1][0][0]==3){
	
		pmvadx=(currSlice->all_mymv[frame_num][mbAddrX-1][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-1][0][6][0]+
				currSlice->all_mymv[frame_num][mbAddrX-1][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-1][2][4][0])/4;
		pmvady=(currSlice->all_mymv[frame_num][mbAddrX-1][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-1][0][6][1]+
				currSlice->all_mymv[frame_num][mbAddrX-1][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-1][2][4][1])/4;
		a1=0;
	}
	
	
	 if((currSlice->mymv_best[frame_num][mbAddrX-1][0][0] && currSlice->mymv_best[frame_num][mbAddrX-1][0][1] && currSlice->mymv_best[frame_num][mbAddrX-1][0][2] && currSlice->mymv_best[frame_num][mbAddrX-1][0][3])==4){
		pmvabx[0]=(currSlice->all_mymv[frame_num][mbAddrX-1][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-1][2][4][0])/2;
		pmvaby[0]=(currSlice->all_mymv[frame_num][mbAddrX-1][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-1][2][4][1])/2;
		pmvabx[1]=(currSlice->all_mymv[frame_num][mbAddrX-1][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-1][2][6][0])/2;
		pmvaby[1]=(currSlice->all_mymv[frame_num][mbAddrX-1][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-1][2][6][1])/2;
		
		
		pmvacx[0]=(currSlice->all_mymv[frame_num][mbAddrX-1][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-1][0][2][0])/2;
		pmvacy[0]=(currSlice->all_mymv[frame_num][mbAddrX-1][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-1][0][2][1])/2;
		pmvacx[1]=(currSlice->all_mymv[frame_num][mbAddrX-1][2][4][0]+currSlice->all_mymv[frame_num][mbAddrX-1][2][6][0])/2;
		pmvacy[1]=(currSlice->all_mymv[frame_num][mbAddrX-1][2][4][1]+currSlice->all_mymv[frame_num][mbAddrX-1][2][6][1])/2;
	  
		pmvadx=(pmvabx[0]+pmvacx[0]+pmvabx[1]+pmvacx[1])/4;
		pmvady=(pmvaby[0]+pmvacy[0]+pmvaby[1]+pmvacy[1])/4;
		a1=0;
	}

	
	if(currSlice->mymv_best[frame_num][mbAddrX-1][0][3]==7){
	    a=1;
		pmva_b44x1[0][0] = (currSlice->all_mymv[frame_num][mbAddrX-1][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-1][0][5][0])/2;
		pmva_b44y1[0][1] = (currSlice->all_mymv[frame_num][mbAddrX-1][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-1][0][5][1])/2;
		pmva_b44x2[0][0] = (currSlice->all_mymv[frame_num][mbAddrX-1][1][4][0]+currSlice->all_mymv[frame_num][mbAddrX-1][1][5][0])/2;
		pmva_b44y2[0][1] = (currSlice->all_mymv[frame_num][mbAddrX-1][1][4][1]+currSlice->all_mymv[frame_num][mbAddrX-1][1][5][1])/2;
		pmva_c44x1[0][0] = (currSlice->all_mymv[frame_num][mbAddrX-1][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-1][1][4][0])/2;
		pmva_c44y1[0][1] = (currSlice->all_mymv[frame_num][mbAddrX-1][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-1][1][4][1])/2;
		pmva_c44x2[0][0] = (currSlice->all_mymv[frame_num][mbAddrX-1][0][5][0]+currSlice->all_mymv[frame_num][mbAddrX-1][1][5][0])/2;
		pmva_c44y2[0][1] = (currSlice->all_mymv[frame_num][mbAddrX-1][0][5][1]+currSlice->all_mymv[frame_num][mbAddrX-1][1][5][1])/2;	
		pmva_d44x1[0] = (pmva_b44x1[0][0]+pmva_c44x1[0][0]);
		pmva_d44y1[0] = (pmva_b44y1[0][1]+pmva_c44y1[0][1]);
		pmva_d44x2[0] = (pmva_b44x2[0][0]+pmva_c44x2[0][0]);
		pmva_d44y2[0] = (pmva_b44y2[0][1]+pmva_c44y2[0][1]);
	
		pmva44x[0][0] = (pmva_d44x1[0]+pmva_d44x2[0])/4;
		pmva44y[0][1] = (pmva_d44y1[0]+pmva_d44y2[0])/4;
	
		}
		
	 if(currSlice->mymv_best[frame_num][mbAddrX-1][0][2]==6 || currSlice->mymv_best[frame_num][mbAddrX-1][0][1]==5){
		a=1;
		pmva44x[0][0]=(currSlice->all_mymv[frame_num][mbAddrX-1][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-1][0][2][0]+
					   currSlice->all_mymv[frame_num][mbAddrX-1][0][3][0]+currSlice->all_mymv[frame_num][mbAddrX-1][-1][2][0])/4;
		pmva44y[0][1]=(currSlice->all_mymv[frame_num][mbAddrX-1][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-1][0][2][1]+
					   currSlice->all_mymv[frame_num][mbAddrX-1][0][3][1]+currSlice->all_mymv[frame_num][mbAddrX-1][-1][2][1])/4;
		}
		else if(currSlice->mymv_best[frame_num][mbAddrX-1][0][0]==4){
		a=1;
		pmva44x[0][0]=currSlice->all_mymv[frame_num][mbAddrX-1][0][2][0];
		pmva44y[0][1]=currSlice->all_mymv[frame_num][mbAddrX-1][0][2][1];
		}
	if(currSlice->mymv_best[frame_num][mbAddrX-1][1][3]==7){
	a=1;
		pmva_b44x1[1][0] = (currSlice->all_mymv[frame_num][mbAddrX-1][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-1][0][3][0])/2;
		pmva_b44y1[1][1] = (currSlice->all_mymv[frame_num][mbAddrX-1][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-1][0][3][1])/2;
		pmva_b44x2[1][0] = (currSlice->all_mymv[frame_num][mbAddrX-1][1][2][0]+currSlice->all_mymv[frame_num][mbAddrX-1][1][3][0])/2;
		pmva_b44y2[1][1] = (currSlice->all_mymv[frame_num][mbAddrX-1][1][2][1]+currSlice->all_mymv[frame_num][mbAddrX-1][1][3][1])/2;
		pmva_c44x1[1][0] = (currSlice->all_mymv[frame_num][mbAddrX-1][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-1][1][2][0])/2;
		pmva_c44y1[1][1] = (currSlice->all_mymv[frame_num][mbAddrX-1][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-1][1][2][1])/2;
		pmva_c44x2[1][0] = (currSlice->all_mymv[frame_num][mbAddrX-1][0][3][0]+currSlice->all_mymv[frame_num][mbAddrX-1][1][3][0])/2;
		pmva_c44y2[1][1] = (currSlice->all_mymv[frame_num][mbAddrX-1][0][3][1]+currSlice->all_mymv[frame_num][mbAddrX-1][1][3][1])/2;
		
		pmva_d44x1[1] = (pmva_b44x1[1][0]+pmva_c44x1[1][0]);
		pmva_d44y1[1] = (pmva_b44y1[1][1]+pmva_c44y1[1][1]);
		pmva_d44x2[1] = (pmva_b44x2[1][0]+pmva_c44x2[1][0]);
		pmva_d44y2[1] = (pmva_b44y2[1][1]+pmva_c44y2[1][1]);
	
		pmva44x[1][0] = (pmva_d44x1[1]+pmva_d44x2[1])/4;
		pmva44y[1][1] = (pmva_d44y1[1]+pmva_d44y2[1])/4;
		
		}
	 if(currSlice->mymv_best[frame_num][mbAddrX-1][1][2]==6 || currSlice->mymv_best[frame_num][mbAddrX-1][1][1]==5){
		a=1;
		pmva44x[1][0]=(currSlice->all_mymv[frame_num][mbAddrX-1][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-1][0][4][0]+
					   currSlice->all_mymv[frame_num][mbAddrX-1][0][5][0]+currSlice->all_mymv[frame_num][mbAddrX-1][-1][4][0])/4;
		pmva44y[1][1]=(currSlice->all_mymv[frame_num][mbAddrX-1][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-1][0][4][1]+
					   currSlice->all_mymv[frame_num][mbAddrX-1][0][5][1]+currSlice->all_mymv[frame_num][mbAddrX-1][-1][4][1])/4;
		}
		else if(currSlice->mymv_best[frame_num][mbAddrX-1][0][1]==4){
		a=1;
		pmva44x[1][0]=currSlice->all_mymv[frame_num][mbAddrX-1][0][4][0];
		pmva44y[1][1]=currSlice->all_mymv[frame_num][mbAddrX-1][0][4][1];
		}
	if(currSlice->mymv_best[frame_num][mbAddrX-1][2][3]==7){
		a=1;
		pmva_b44x1[2][0] = (currSlice->all_mymv[frame_num][mbAddrX-1][2][4][0]+currSlice->all_mymv[frame_num][mbAddrX-1][2][5][0])/2;
		pmva_b44y1[2][1] = (currSlice->all_mymv[frame_num][mbAddrX-1][2][4][1]+currSlice->all_mymv[frame_num][mbAddrX-1][2][5][1])/2;
		pmva_b44x2[2][0] = (currSlice->all_mymv[frame_num][mbAddrX-1][3][4][0]+currSlice->all_mymv[frame_num][mbAddrX-1][3][5][0])/2;
		pmva_b44y2[2][1] = (currSlice->all_mymv[frame_num][mbAddrX-1][3][4][1]+currSlice->all_mymv[frame_num][mbAddrX-1][3][5][1])/2;
		pmva_c44x1[2][0] = (currSlice->all_mymv[frame_num][mbAddrX-1][2][4][0]+currSlice->all_mymv[frame_num][mbAddrX-1][3][4][0])/2;
		pmva_c44y1[2][1] = (currSlice->all_mymv[frame_num][mbAddrX-1][2][4][1]+currSlice->all_mymv[frame_num][mbAddrX-1][3][4][1])/2;
		pmva_c44x2[2][0] = (currSlice->all_mymv[frame_num][mbAddrX-1][2][5][0]+currSlice->all_mymv[frame_num][mbAddrX-1][3][5][0])/2;
		pmva_c44y2[2][1] = (currSlice->all_mymv[frame_num][mbAddrX-1][2][5][1]+currSlice->all_mymv[frame_num][mbAddrX-1][3][5][1])/2;
		
		pmva_d44x1[2] = (pmva_b44x1[2][0]+pmva_c44x1[2][0]);
		pmva_d44y1[2] = (pmva_b44y1[2][1]+pmva_c44y1[2][1]);
		pmva_d44x2[2] = (pmva_b44x2[2][0]+pmva_c44x2[2][0]);
		pmva_d44y2[2] = (pmva_b44y2[2][1]+pmva_c44y2[2][1]);
	
		pmva44x[2][0] = (pmva_d44x1[2]+pmva_d44x2[2])/4;
		pmva44y[2][1] = (pmva_d44y1[2]+pmva_d44y2[2])/4;
		
		}
	 if(currSlice->mymv_best[frame_num][mbAddrX-1][2][2]==6 || currSlice->mymv_best[frame_num][mbAddrX-1][2][1]==5){
		a=1;
		pmva44x[2][0]=(currSlice->all_mymv[frame_num][mbAddrX-1][2][4][0]+currSlice->all_mymv[frame_num][mbAddrX-1][2][4][0]+
					   currSlice->all_mymv[frame_num][mbAddrX-1][2][5][0]+currSlice->all_mymv[frame_num][mbAddrX-1][1][4][0])/4;
		pmva44y[2][1]=(currSlice->all_mymv[frame_num][mbAddrX-1][2][4][1]+currSlice->all_mymv[frame_num][mbAddrX-1][2][4][1]+
					   currSlice->all_mymv[frame_num][mbAddrX-1][2][5][1]+currSlice->all_mymv[frame_num][mbAddrX-1][1][4][1])/4;
		}
		else if(currSlice->mymv_best[frame_num][mbAddrX-1][0][2]==4){
		a=1;
		pmva44x[2][0]=currSlice->all_mymv[frame_num][mbAddrX-1][2][4][0];
		pmva44y[2][1]=currSlice->all_mymv[frame_num][mbAddrX-1][2][4][1];
		}
	if(currSlice->mymv_best[frame_num][mbAddrX-1][3][3]==7){
		a=1;
		pmva_b44x1[3][0] = (currSlice->all_mymv[frame_num][mbAddrX-1][2][6][0]+currSlice->all_mymv[frame_num][mbAddrX-1][2][7][0])/2;
		pmva_b44y1[3][1] = (currSlice->all_mymv[frame_num][mbAddrX-1][2][6][1]+currSlice->all_mymv[frame_num][mbAddrX-1][2][7][1])/2;
		pmva_b44x2[3][0] = (currSlice->all_mymv[frame_num][mbAddrX-1][3][6][0]+currSlice->all_mymv[frame_num][mbAddrX-1][3][7][0])/2;
		pmva_b44y2[3][1] = (currSlice->all_mymv[frame_num][mbAddrX-1][3][6][1]+currSlice->all_mymv[frame_num][mbAddrX-1][3][7][1])/2;
		pmva_c44x1[3][0] = (currSlice->all_mymv[frame_num][mbAddrX-1][2][6][0]+currSlice->all_mymv[frame_num][mbAddrX-1][3][6][0])/2;
		pmva_c44y1[3][1] = (currSlice->all_mymv[frame_num][mbAddrX-1][2][6][1]+currSlice->all_mymv[frame_num][mbAddrX-1][3][6][1])/2;
		pmva_c44x2[3][0] = (currSlice->all_mymv[frame_num][mbAddrX-1][2][7][0]+currSlice->all_mymv[frame_num][mbAddrX-1][3][7][0])/2;
		pmva_c44y2[3][1] = (currSlice->all_mymv[frame_num][mbAddrX-1][2][7][1]+currSlice->all_mymv[frame_num][mbAddrX-1][3][7][1])/2;
		
		pmva_d44x1[3] = (pmva_b44x1[3][0]+pmva_c44x1[3][0]);
		pmva_d44y1[3] = (pmva_b44y1[3][1]+pmva_c44y1[3][1]);
		pmva_d44x2[3] = (pmva_b44x2[3][0]+pmva_c44x2[3][0]);
		pmva_d44y2[3] = (pmva_b44y2[3][1]+pmva_c44y2[3][1]);
	
		pmva44x[3][0] = (pmva_d44x1[3]+pmva_d44x2[3])/4;
		pmva44y[3][1] = (pmva_d44y1[3]+pmva_d44y2[3])/4;
		
		}
	 if(currSlice->mymv_best[frame_num][mbAddrX-1][3][2]==6 || currSlice->mymv_best[frame_num][mbAddrX-1][3][1]==5){
		a=1;
		pmva44x[3][0]=(currSlice->all_mymv[frame_num][mbAddrX-1][2][6][0]+currSlice->all_mymv[frame_num][mbAddrX-1][2][6][0]+
					   currSlice->all_mymv[frame_num][mbAddrX-1][2][7][0]+currSlice->all_mymv[frame_num][mbAddrX-1][1][6][0])/4;
		pmva44y[3][1]=(currSlice->all_mymv[frame_num][mbAddrX-1][2][6][1]+currSlice->all_mymv[frame_num][mbAddrX-1][2][6][1]+
					   currSlice->all_mymv[frame_num][mbAddrX-1][2][7][1]+currSlice->all_mymv[frame_num][mbAddrX-1][1][6][1])/4;
		}
		else if(currSlice->mymv_best[frame_num][mbAddrX-1][0][3]==4){
		a=1;
		pmva44x[3][0]=currSlice->all_mymv[frame_num][mbAddrX-1][2][6][0];
		pmva44y[3][1]=currSlice->all_mymv[frame_num][mbAddrX-1][2][6][1];
		}
	
	if(a && a1){

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
	}
	//end a
	
	if(block[1].available){
	
	if(currSlice->mymv_best[frame_num][mbAddrX-22][0][0]==0){
		pmvbdx=currSlice->all_mymv[frame_num][mbAddrX-22][0][4][0];
		pmvbdy=currSlice->all_mymv[frame_num][mbAddrX-22][0][4][1];
		b1=0;
	}
	
	if(currSlice->mymv_best[frame_num][mbAddrX-22][0][0]==1){
		pmvbdx=currSlice->all_mymv[frame_num][mbAddrX-22][0][4][0];
		pmvbdy=currSlice->all_mymv[frame_num][mbAddrX-22][0][4][1];
		b1=0;
	}

	if(currSlice->mymv_best[frame_num][mbAddrX-22][0][0]==2){
	
		pmvbdx=(currSlice->all_mymv[frame_num][mbAddrX-22][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-22][2][4][0]+
				currSlice->all_mymv[frame_num][mbAddrX-22][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-22][0][6][0])/4;
		pmvbdy=(currSlice->all_mymv[frame_num][mbAddrX-22][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-22][2][4][1]+
				currSlice->all_mymv[frame_num][mbAddrX-22][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-22][0][6][1])/4;
		b1=0;
	}
	 if(currSlice->mymv_best[frame_num][mbAddrX-22][0][0]==3){
	
		pmvbdx=(currSlice->all_mymv[frame_num][mbAddrX-22][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-22][0][6][0]+
				currSlice->all_mymv[frame_num][mbAddrX-22][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-22][2][4][0])/4;
		pmvbdy=(currSlice->all_mymv[frame_num][mbAddrX-22][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-22][0][6][1]+
				currSlice->all_mymv[frame_num][mbAddrX-22][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-22][2][4][1])/4;
		b1=0;
	}
	
	 if((currSlice->mymv_best[frame_num][mbAddrX-22][0][0] && currSlice->mymv_best[frame_num][mbAddrX-22][0][1] && currSlice->mymv_best[frame_num][mbAddrX-22][0][2] && currSlice->mymv_best[frame_num][mbAddrX-22][0][3])==4){
		pmvbbx[0]=(currSlice->all_mymv[frame_num][mbAddrX-22][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-22][2][4][0])/2;
		pmvbby[0]=(currSlice->all_mymv[frame_num][mbAddrX-22][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-22][2][4][1])/2;
		pmvbbx[1]=(currSlice->all_mymv[frame_num][mbAddrX-22][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-22][2][6][0])/2;
		pmvbby[1]=(currSlice->all_mymv[frame_num][mbAddrX-22][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-22][2][6][1])/2;
		
		
		pmvbcx[0]=(currSlice->all_mymv[frame_num][mbAddrX-22][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-22][0][2][0])/2;
		pmvbcy[0]=(currSlice->all_mymv[frame_num][mbAddrX-22][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-22][0][2][1])/2;
		pmvbcx[1]=(currSlice->all_mymv[frame_num][mbAddrX-22][2][4][0]+currSlice->all_mymv[frame_num][mbAddrX-22][2][6][0])/2;
		pmvbcy[1]=(currSlice->all_mymv[frame_num][mbAddrX-22][2][4][1]+currSlice->all_mymv[frame_num][mbAddrX-22][2][6][1])/2;
	  
		pmvbdx=(pmvbbx[0]+pmvbcx[0]+pmvbbx[1]+pmvbcx[1])/4;
		pmvbdy=(pmvbby[0]+pmvbcy[0]+pmvbby[1]+pmvbcy[1])/4;
		b1=0;
	}
	 
	
	if(currSlice->mymv_best[frame_num][mbAddrX-22][0][3]==7){
		b=1;
		pmvb_b44x1[0][0] = (currSlice->all_mymv[frame_num][mbAddrX-22][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-22][0][5][0])/2;
		pmvb_b44y1[0][1] = (currSlice->all_mymv[frame_num][mbAddrX-22][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-22][0][5][1])/2;
		pmvb_b44x2[0][0] = (currSlice->all_mymv[frame_num][mbAddrX-22][1][4][0]+currSlice->all_mymv[frame_num][mbAddrX-22][1][5][0])/2;
		pmvb_b44y2[0][1] = (currSlice->all_mymv[frame_num][mbAddrX-22][1][4][1]+currSlice->all_mymv[frame_num][mbAddrX-22][1][5][1])/2;
		pmvb_c44x1[0][0] = (currSlice->all_mymv[frame_num][mbAddrX-22][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-22][1][4][0])/2;
		pmvb_c44y1[0][1] = (currSlice->all_mymv[frame_num][mbAddrX-22][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-22][1][4][1])/2;
		pmvb_c44x2[0][0] = (currSlice->all_mymv[frame_num][mbAddrX-22][0][5][0]+currSlice->all_mymv[frame_num][mbAddrX-22][1][5][0])/2;
		pmvb_c44y2[0][1] = (currSlice->all_mymv[frame_num][mbAddrX-22][0][5][1]+currSlice->all_mymv[frame_num][mbAddrX-22][1][5][1])/2;	
		
		pmvb_d44x1[0] = (pmvb_b44x1[0][0]+pmvb_c44x1[0][0]);
		pmvb_d44y1[0] = (pmvb_b44y1[0][1]+pmvb_c44y1[0][1]);
		pmvb_d44x2[0] = (pmvb_b44x2[0][0]+pmvb_c44x2[0][0]);
		pmvb_d44y2[0] = (pmvb_b44y2[0][1]+pmvb_c44y2[0][1]);
		
		pmvb44x[0][0] = (pmvb_d44x1[0]+pmvb_d44x2[0])/4;
		pmvb44y[0][1] = (pmvb_d44y1[0]+pmvb_d44y2[0])/4;
		}
	if(currSlice->mymv_best[frame_num][mbAddrX-22][0][2]==6 || currSlice->mymv_best[frame_num][mbAddrX-22][0][1]==5){
		b=1;
		pmvb44x[0][0]=(currSlice->all_mymv[frame_num][mbAddrX-22][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-22][0][2][0]+
					   currSlice->all_mymv[frame_num][mbAddrX-22][0][3][0]+currSlice->all_mymv[frame_num][mbAddrX-22][-1][2][0])/4;
		pmvb44y[0][1]=(currSlice->all_mymv[frame_num][mbAddrX-22][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-22][0][2][1]+
					   currSlice->all_mymv[frame_num][mbAddrX-22][0][3][1]+currSlice->all_mymv[frame_num][mbAddrX-22][-1][2][1])/4;
		}
		else if(currSlice->mymv_best[frame_num][mbAddrX-22][0][0]==4){
		b=1;
		pmvb44x[0][0]=currSlice->all_mymv[frame_num][mbAddrX-22][0][2][0];
		pmvb44y[0][1]=currSlice->all_mymv[frame_num][mbAddrX-22][0][2][1];
		}
	if(currSlice->mymv_best[frame_num][mbAddrX-22][1][3]==7){	
		b=1;
		pmvb_b44x1[1][0] = (currSlice->all_mymv[frame_num][mbAddrX-22][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-22][0][3][0])/2;
		pmvb_b44y1[1][1] = (currSlice->all_mymv[frame_num][mbAddrX-22][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-22][0][3][1])/2;
		pmvb_b44x2[1][0] = (currSlice->all_mymv[frame_num][mbAddrX-22][1][2][0]+currSlice->all_mymv[frame_num][mbAddrX-22][1][3][0])/2;
		pmvb_b44y2[1][1] = (currSlice->all_mymv[frame_num][mbAddrX-22][1][2][1]+currSlice->all_mymv[frame_num][mbAddrX-22][1][3][1])/2;
		pmvb_c44x1[1][0] = (currSlice->all_mymv[frame_num][mbAddrX-22][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-22][1][2][0])/2;
		pmvb_c44y1[1][1] = (currSlice->all_mymv[frame_num][mbAddrX-22][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-22][1][2][1])/2;
		pmvb_c44x2[1][0] = (currSlice->all_mymv[frame_num][mbAddrX-22][0][3][0]+currSlice->all_mymv[frame_num][mbAddrX-22][1][3][0])/2;
		pmvb_c44y2[1][1] = (currSlice->all_mymv[frame_num][mbAddrX-22][0][3][1]+currSlice->all_mymv[frame_num][mbAddrX-22][1][3][1])/2;
		
		pmvb_d44x1[1] = (pmvb_b44x1[1][0]+pmvb_c44x1[1][0]);
		pmvb_d44y1[1] = (pmvb_b44y1[1][1]+pmvb_c44y1[1][1]);
		pmvb_d44x2[1] = (pmvb_b44x2[1][0]+pmvb_c44x2[1][0]);
		pmvb_d44y2[1] = (pmvb_b44y2[1][1]+pmvb_c44y2[1][1]);

		pmvb44x[1][0] = (pmvb_d44x1[1]+pmvb_d44x2[1])/4;
		pmvb44y[1][1] = (pmvb_d44y1[1]+pmvb_d44y2[1])/4;
		}
	 if(currSlice->mymv_best[frame_num][mbAddrX-22][1][2]==6 || currSlice->mymv_best[frame_num][mbAddrX-22][1][1]==5){
		b=1;
		pmvb44x[1][0]=(currSlice->all_mymv[frame_num][mbAddrX-22][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-22][0][4][0]+
					   currSlice->all_mymv[frame_num][mbAddrX-22][0][5][0]+currSlice->all_mymv[frame_num][mbAddrX-22][-1][4][0])/4;
		pmvb44y[1][1]=(currSlice->all_mymv[frame_num][mbAddrX-22][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-22][0][4][1]+
					   currSlice->all_mymv[frame_num][mbAddrX-22][0][5][1]+currSlice->all_mymv[frame_num][mbAddrX-22][-1][4][1])/4;
		}
		else if(currSlice->mymv_best[frame_num][mbAddrX-22][0][1]==4){
		b=1;
		pmvb44x[1][0]=currSlice->all_mymv[frame_num][mbAddrX-22][0][4][0];
		pmvb44y[1][1]=currSlice->all_mymv[frame_num][mbAddrX-22][0][4][1];
		}
	if(currSlice->mymv_best[frame_num][mbAddrX-22][2][3]==7){
		b=1;
		pmvb_b44x1[2][0] = (currSlice->all_mymv[frame_num][mbAddrX-22][2][4][0]+currSlice->all_mymv[frame_num][mbAddrX-22][2][5][0])/2;
		pmvb_b44y1[2][1] = (currSlice->all_mymv[frame_num][mbAddrX-22][2][4][1]+currSlice->all_mymv[frame_num][mbAddrX-22][2][5][1])/2;
		pmvb_b44x2[2][0] = (currSlice->all_mymv[frame_num][mbAddrX-22][3][4][0]+currSlice->all_mymv[frame_num][mbAddrX-22][3][5][0])/2;
		pmvb_b44y2[2][1] = (currSlice->all_mymv[frame_num][mbAddrX-22][3][4][1]+currSlice->all_mymv[frame_num][mbAddrX-22][3][5][1])/2;
		pmvb_c44x1[2][0] = (currSlice->all_mymv[frame_num][mbAddrX-22][2][4][0]+currSlice->all_mymv[frame_num][mbAddrX-22][3][4][0])/2;
		pmvb_c44y1[2][1] = (currSlice->all_mymv[frame_num][mbAddrX-22][2][4][1]+currSlice->all_mymv[frame_num][mbAddrX-22][3][4][1])/2;
		pmvb_c44x2[2][0] = (currSlice->all_mymv[frame_num][mbAddrX-22][2][5][0]+currSlice->all_mymv[frame_num][mbAddrX-22][3][5][0])/2;
		pmvb_c44y2[2][1] = (currSlice->all_mymv[frame_num][mbAddrX-22][2][5][1]+currSlice->all_mymv[frame_num][mbAddrX-22][3][5][1])/2;
		
		pmvb_d44x1[2] = (pmvb_b44x1[2][0]+pmvb_c44x1[2][0]);
		pmvb_d44y1[2] = (pmvb_b44y1[2][1]+pmvb_c44y1[2][1]);
		pmvb_d44x2[2] = (pmvb_b44x2[2][0]+pmvb_c44x2[2][0]);
		pmvb_d44y2[2] = (pmvb_b44y2[2][1]+pmvb_c44y2[2][1]);
		

		pmvb44x[2][0] = (pmvb_d44x1[2]+pmvb_d44x2[2])/4;
		pmvb44y[2][1] = (pmvb_d44y1[2]+pmvb_d44y2[2])/4;
		}
	if(currSlice->mymv_best[frame_num][mbAddrX-22][2][2]==6 || currSlice->mymv_best[frame_num][mbAddrX-22][2][1]==5){
		b=1;
		pmvb44x[2][0]=(currSlice->all_mymv[frame_num][mbAddrX-22][2][4][0]+currSlice->all_mymv[frame_num][mbAddrX-22][2][4][0]+
					   currSlice->all_mymv[frame_num][mbAddrX-22][2][5][0]+currSlice->all_mymv[frame_num][mbAddrX-22][1][4][0])/4;
		pmvb44y[2][1]=(currSlice->all_mymv[frame_num][mbAddrX-22][2][4][1]+currSlice->all_mymv[frame_num][mbAddrX-22][2][4][1]+
					   currSlice->all_mymv[frame_num][mbAddrX-22][2][5][1]+currSlice->all_mymv[frame_num][mbAddrX-22][1][4][1])/4;
		}
		else if(currSlice->mymv_best[frame_num][mbAddrX-22][0][3]==4){
		b=1;
		pmvb44x[2][0]=currSlice->all_mymv[frame_num][mbAddrX-22][2][4][0];
		pmvb44y[2][1]=currSlice->all_mymv[frame_num][mbAddrX-22][2][4][1];
		}
	if(currSlice->mymv_best[frame_num][mbAddrX-22][3][3]==7){
		b=1;
		pmvb_b44x1[3][0] = (currSlice->all_mymv[frame_num][mbAddrX-22][2][6][0]+currSlice->all_mymv[frame_num][mbAddrX-22][2][7][0])/2;
		pmvb_b44y1[3][1] = (currSlice->all_mymv[frame_num][mbAddrX-22][2][6][1]+currSlice->all_mymv[frame_num][mbAddrX-22][2][7][1])/2;
		pmvb_b44x2[3][0] = (currSlice->all_mymv[frame_num][mbAddrX-22][3][6][0]+currSlice->all_mymv[frame_num][mbAddrX-22][3][7][0])/2;
		pmvb_b44y2[3][1] = (currSlice->all_mymv[frame_num][mbAddrX-22][3][6][1]+currSlice->all_mymv[frame_num][mbAddrX-22][3][7][1])/2;
		pmvb_c44x1[3][0] = (currSlice->all_mymv[frame_num][mbAddrX-22][2][6][0]+currSlice->all_mymv[frame_num][mbAddrX-22][3][6][0])/2;
		pmvb_c44y1[3][1] = (currSlice->all_mymv[frame_num][mbAddrX-22][2][6][1]+currSlice->all_mymv[frame_num][mbAddrX-22][3][6][1])/2;
		pmvb_c44x2[3][0] = (currSlice->all_mymv[frame_num][mbAddrX-22][2][7][0]+currSlice->all_mymv[frame_num][mbAddrX-22][3][7][0])/2;
		pmvb_c44y2[3][1] = (currSlice->all_mymv[frame_num][mbAddrX-22][2][7][1]+currSlice->all_mymv[frame_num][mbAddrX-22][3][7][1])/2;
		
		pmvb_d44x1[3] = (pmvb_b44x1[3][0]+pmvb_c44x1[3][0]);
		pmvb_d44y1[3] = (pmvb_b44y1[3][1]+pmvb_c44y1[3][1]);
		pmvb_d44x2[3] = (pmvb_b44x2[3][0]+pmvb_c44x2[3][0]);
		pmvb_d44y2[3] = (pmvb_b44y2[3][1]+pmvb_c44y2[3][1]);
		
		pmvb44x[3][0] = (pmvb_d44x1[3]+pmvb_d44x2[3])/4;
		pmvb44y[3][1] = (pmvb_d44y1[3]+pmvb_d44y2[3])/4;
		}
	if(currSlice->mymv_best[frame_num][mbAddrX-22][3][2]==6 || currSlice->mymv_best[frame_num][mbAddrX-22][3][1]==5){
		b=1;
		pmvb44x[3][0]=(currSlice->all_mymv[frame_num][mbAddrX-22][2][6][0]+currSlice->all_mymv[frame_num][mbAddrX-22][2][6][0]+
					   currSlice->all_mymv[frame_num][mbAddrX-22][2][7][0]+currSlice->all_mymv[frame_num][mbAddrX-22][1][6][0])/4;
		pmvb44y[3][1]=(currSlice->all_mymv[frame_num][mbAddrX-22][2][6][1]+currSlice->all_mymv[frame_num][mbAddrX-22][2][6][1]+
					   currSlice->all_mymv[frame_num][mbAddrX-22][2][7][1]+currSlice->all_mymv[frame_num][mbAddrX-22][1][6][1])/4;
		}
		else if(currSlice->mymv_best[frame_num][mbAddrX-22][0][3]==4){
		b=1;
		pmvb44x[3][0]=currSlice->all_mymv[frame_num][mbAddrX-22][2][6][0];
		pmvb44y[3][1]=currSlice->all_mymv[frame_num][mbAddrX-22][2][6][1];
		}
	
	if(b && b1){
	
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
	
	}
	//end b
	if(block[2].available){
	    
	if(currSlice->mymv_best[frame_num][mbAddrX-21][0][0]==0){
		pmvcdx=currSlice->all_mymv[frame_num][mbAddrX-1][0][4][0];
		pmvcdy=currSlice->all_mymv[frame_num][mbAddrX-1][0][4][1];
		c1=0;
	}	
		
	if(currSlice->mymv_best[frame_num][mbAddrX-21][0][0]==1){
		pmvcdx=currSlice->all_mymv[frame_num][mbAddrX-21][0][4][0];
		pmvcdy=currSlice->all_mymv[frame_num][mbAddrX-21][0][4][1];
		c1=0;
	}	
 
	if(currSlice->mymv_best[frame_num][mbAddrX-21][0][0]==2){
	
		pmvcdx=(currSlice->all_mymv[frame_num][mbAddrX-21][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-21][2][4][0]+
				currSlice->all_mymv[frame_num][mbAddrX-21][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-21][0][6][0])/4;
		
		pmvcdy=(currSlice->all_mymv[frame_num][mbAddrX-21][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-21][2][4][1]+
				currSlice->all_mymv[frame_num][mbAddrX-21][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-21][0][6][1])/4;
		c1=0;
	}
	if(currSlice->mymv_best[frame_num][mbAddrX-21][0][0]==3){
	
		pmvcdx=(currSlice->all_mymv[frame_num][mbAddrX-21][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-21][0][6][0]+
				currSlice->all_mymv[frame_num][mbAddrX-21][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-21][2][4][0])/4;
		
		pmvcdy=(currSlice->all_mymv[frame_num][mbAddrX-21][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-21][0][6][1]+
				currSlice->all_mymv[frame_num][mbAddrX-21][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-21][2][4][1])/4;
		c1=0;
	}

	
	if((currSlice->mymv_best[frame_num][mbAddrX-21][0][0] && currSlice->mymv_best[frame_num][mbAddrX-21][0][1] && currSlice->mymv_best[frame_num][mbAddrX-21][0][2] && currSlice->mymv_best[frame_num][mbAddrX-21][0][3])==4){
		pmvcbx[0]=(currSlice->all_mymv[frame_num][mbAddrX-21][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-21][2][4][0])/2;
		pmvcby[0]=(currSlice->all_mymv[frame_num][mbAddrX-21][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-21][2][4][1])/2;
		pmvcbx[1]=(currSlice->all_mymv[frame_num][mbAddrX-21][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-21][2][6][0])/2;
		pmvcby[1]=(currSlice->all_mymv[frame_num][mbAddrX-21][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-21][2][6][1])/2;
		
		
		pmvccx[0]=(currSlice->all_mymv[frame_num][mbAddrX-21][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-21][0][2][0])/2;
		pmvccy[0]=(currSlice->all_mymv[frame_num][mbAddrX-21][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-21][0][2][1])/2;
		pmvccx[1]=(currSlice->all_mymv[frame_num][mbAddrX-21][2][4][0]+currSlice->all_mymv[frame_num][mbAddrX-21][2][6][0])/2;
		pmvccy[1]=(currSlice->all_mymv[frame_num][mbAddrX-21][2][4][1]+currSlice->all_mymv[frame_num][mbAddrX-21][2][6][1])/2;
	  
		pmvcdx=(pmvcbx[0]+pmvccx[0]+pmvcbx[1]+pmvccx[1])/4;
		pmvcdy=(pmvcby[0]+pmvccy[0]+pmvcby[1]+pmvccy[1])/4;
		c1=0;
	}
	
	
		
	if(currSlice->mymv_best[frame_num][mbAddrX-21][0][3]==7){
		c=1;
		pmvc_b44x1[0][0] = (currSlice->all_mymv[frame_num][mbAddrX-21][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-21][0][5][0])/2;
		pmvc_b44y1[0][1] = (currSlice->all_mymv[frame_num][mbAddrX-21][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-21][0][5][1])/2;
		pmvc_b44x2[0][0] = (currSlice->all_mymv[frame_num][mbAddrX-21][1][4][0]+currSlice->all_mymv[frame_num][mbAddrX-21][1][5][0])/2;
		pmvc_b44y2[0][1] = (currSlice->all_mymv[frame_num][mbAddrX-21][1][4][1]+currSlice->all_mymv[frame_num][mbAddrX-21][1][5][1])/2;
		pmvc_c44x1[0][0] = (currSlice->all_mymv[frame_num][mbAddrX-21][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-21][1][4][0])/2;
		pmvc_c44y1[0][1] = (currSlice->all_mymv[frame_num][mbAddrX-21][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-21][1][4][1])/2;
		pmvc_c44x2[0][0] = (currSlice->all_mymv[frame_num][mbAddrX-21][0][5][0]+currSlice->all_mymv[frame_num][mbAddrX-21][1][5][0])/2;
		pmvc_c44y2[0][1] = (currSlice->all_mymv[frame_num][mbAddrX-21][0][5][1]+currSlice->all_mymv[frame_num][mbAddrX-21][1][5][1])/2;	
		
		pmvc_d44x1[0] = (pmvc_b44x1[0][0]+pmvc_c44x1[0][0]);
		pmvc_d44y1[0] = (pmvc_b44y1[0][1]+pmvc_c44y1[0][1]);
		pmvc_d44x2[0] = (pmvc_b44x2[0][0]+pmvc_c44x2[0][0]);
		pmvc_d44y2[0] = (pmvc_b44y2[0][1]+pmvc_c44y2[0][1]);

		pmvc44x[0][0] = (pmvc_d44x1[0]+pmvc_d44x2[0])/4;
		pmvc44y[0][1] = (pmvc_d44y1[0]+pmvc_d44y2[0])/4;
		}
	if(currSlice->mymv_best[frame_num][mbAddrX-21][0][2]==6 || currSlice->mymv_best[frame_num][mbAddrX-21][0][1]==5){
		c=1;
		pmvc44x[0][0]=(currSlice->all_mymv[frame_num][mbAddrX-21][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-21][0][2][0]+
					   currSlice->all_mymv[frame_num][mbAddrX-21][0][3][0]+currSlice->all_mymv[frame_num][mbAddrX-21][-1][2][0])/4;
		pmvc44y[0][1]=(currSlice->all_mymv[frame_num][mbAddrX-21][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-21][0][2][1]+
					   currSlice->all_mymv[frame_num][mbAddrX-21][0][3][1]+currSlice->all_mymv[frame_num][mbAddrX-21][-1][2][1])/4;
		}
		else if(currSlice->mymv_best[frame_num][mbAddrX-21][0][0]==4){
		c=1;
		pmvc44x[0][0]=currSlice->all_mymv[frame_num][mbAddrX-21][0][2][0];
		pmvc44y[0][1]=currSlice->all_mymv[frame_num][mbAddrX-21][0][2][1];
		}
		if(currSlice->mymv_best[frame_num][mbAddrX-21][1][3]==7){
		c=1;
		pmvc_b44x1[1][0] = (currSlice->all_mymv[frame_num][mbAddrX-21][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-21][0][3][0])/2;
		pmvc_b44y1[1][1] = (currSlice->all_mymv[frame_num][mbAddrX-21][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-21][0][3][1])/2;
		pmvc_b44x2[1][0] = (currSlice->all_mymv[frame_num][mbAddrX-21][1][2][0]+currSlice->all_mymv[frame_num][mbAddrX-21][1][3][0])/2;
		pmvc_b44y2[1][1] = (currSlice->all_mymv[frame_num][mbAddrX-21][1][2][1]+currSlice->all_mymv[frame_num][mbAddrX-21][1][3][1])/2;
		pmvc_c44x1[1][0] = (currSlice->all_mymv[frame_num][mbAddrX-21][0][2][0]+currSlice->all_mymv[frame_num][mbAddrX-21][1][2][0])/2;
		pmvc_c44y1[1][1] = (currSlice->all_mymv[frame_num][mbAddrX-21][0][2][1]+currSlice->all_mymv[frame_num][mbAddrX-21][1][2][1])/2;
		pmvc_c44x2[1][0] = (currSlice->all_mymv[frame_num][mbAddrX-21][0][3][0]+currSlice->all_mymv[frame_num][mbAddrX-21][1][3][0])/2;
		pmvc_c44y2[1][1] = (currSlice->all_mymv[frame_num][mbAddrX-21][0][3][1]+currSlice->all_mymv[frame_num][mbAddrX-21][1][3][1])/2;
		
		pmvc_d44x1[1] = (pmvc_b44x1[1][0]+pmvc_c44x1[1][0]);
		pmvc_d44y1[1] = (pmvc_b44y1[1][1]+pmvc_c44y1[1][1]);
		pmvc_d44x2[1] = (pmvc_b44x2[1][0]+pmvc_c44x2[1][0]);
		pmvc_d44y2[1] = (pmvc_b44y2[1][1]+pmvc_c44y2[1][1]);

		pmvc44x[1][0] = (pmvc_d44x1[1]+pmvc_d44x2[1])/4;
		pmvc44y[1][1] = (pmvc_d44y1[1]+pmvc_d44y2[1])/4;
		}
	if(currSlice->mymv_best[frame_num][mbAddrX-21][1][2]==6 || currSlice->mymv_best[frame_num][mbAddrX-21][1][1]==5){
		c=1;
		pmvc44x[1][0]=(currSlice->all_mymv[frame_num][mbAddrX-21][0][4][0]+currSlice->all_mymv[frame_num][mbAddrX-21][0][4][0]+
					   currSlice->all_mymv[frame_num][mbAddrX-21][0][5][0]+currSlice->all_mymv[frame_num][mbAddrX-21][-1][4][0])/4;
		pmvc44y[1][1]=(currSlice->all_mymv[frame_num][mbAddrX-21][0][4][1]+currSlice->all_mymv[frame_num][mbAddrX-21][0][4][1]+
					   currSlice->all_mymv[frame_num][mbAddrX-21][0][5][1]+currSlice->all_mymv[frame_num][mbAddrX-21][-1][4][1])/4;
		}
		else if(currSlice->mymv_best[frame_num][mbAddrX-21][0][1]==4){
		c=1;
		pmvc44x[1][0]=currSlice->all_mymv[frame_num][mbAddrX-21][0][4][0];
		pmvc44y[1][1]=currSlice->all_mymv[frame_num][mbAddrX-21][0][4][1];
		}
		if(currSlice->mymv_best[mbAddrX-21][2][3]==7){
		c=1;
		pmvc_b44x1[2][0] = (currSlice->all_mymv[frame_num][mbAddrX-21][2][4][0]+currSlice->all_mymv[frame_num][mbAddrX-21][2][5][0])/2;
		pmvc_b44y1[2][1] = (currSlice->all_mymv[frame_num][mbAddrX-21][2][4][1]+currSlice->all_mymv[frame_num][mbAddrX-21][2][5][1])/2;
		pmvc_b44x2[2][0] = (currSlice->all_mymv[frame_num][mbAddrX-21][3][4][0]+currSlice->all_mymv[frame_num][mbAddrX-21][3][5][0])/2;
		pmvc_b44y2[2][1] = (currSlice->all_mymv[frame_num][mbAddrX-21][3][4][1]+currSlice->all_mymv[frame_num][mbAddrX-21][3][5][1])/2;
		pmvc_c44x1[2][0] = (currSlice->all_mymv[frame_num][mbAddrX-21][2][4][0]+currSlice->all_mymv[frame_num][mbAddrX-21][3][4][0])/2;
		pmvc_c44y1[2][1] = (currSlice->all_mymv[frame_num][mbAddrX-21][2][4][1]+currSlice->all_mymv[frame_num][mbAddrX-21][3][4][1])/2;
		pmvc_c44x2[2][0] = (currSlice->all_mymv[frame_num][mbAddrX-21][2][5][0]+currSlice->all_mymv[frame_num][mbAddrX-21][3][5][0])/2;
		pmvc_c44y2[2][1] = (currSlice->all_mymv[frame_num][mbAddrX-21][2][5][1]+currSlice->all_mymv[frame_num][mbAddrX-21][3][5][1])/2;
		
		pmvc_d44x1[2] = (pmvc_b44x1[2][0]+pmvc_c44x1[2][0]);
		pmvc_d44y1[2] = (pmvc_b44y1[2][1]+pmvc_c44y1[2][1]);
		pmvc_d44x2[2] = (pmvc_b44x2[2][0]+pmvc_c44x2[2][0]);
		pmvc_d44y2[2] = (pmvc_b44y2[2][1]+pmvc_c44y2[2][1]);

		pmvc44x[2][0] = (pmvc_d44x1[2]+pmvc_d44x2[2])/4;
		pmvc44y[2][1] = (pmvc_d44y1[2]+pmvc_d44y2[2])/4;
		}
	if(currSlice->mymv_best[frame_num][mbAddrX-21][2][2]==6 || currSlice->mymv_best[frame_num][mbAddrX-21][2][1]==5){
		c=1;
		pmvc44x[2][0]=(currSlice->all_mymv[frame_num][mbAddrX-21][2][4][0]+currSlice->all_mymv[frame_num][mbAddrX-21][2][4][0]+
					   currSlice->all_mymv[frame_num][mbAddrX-21][2][5][0]+currSlice->all_mymv[frame_num][mbAddrX-21][1][4][0])/4;
		pmvc44y[2][1]=(currSlice->all_mymv[frame_num][mbAddrX-21][2][4][1]+currSlice->all_mymv[frame_num][mbAddrX-21][2][4][1]+
					   currSlice->all_mymv[frame_num][mbAddrX-21][2][5][1]+currSlice->all_mymv[frame_num][mbAddrX-21][1][4][1])/4;
		}
		else if(currSlice->mymv_best[frame_num][mbAddrX-21][0][2]==4){
		c=1;
		pmvc44x[2][0]=currSlice->all_mymv[frame_num][mbAddrX-21][2][4][0];
		pmvc44y[2][1]=currSlice->all_mymv[frame_num][mbAddrX-21][2][4][1];
		}
		if(currSlice->mymv_best[frame_num][mbAddrX-21][3][3]==7){
		c=1;
		pmvc_b44x1[3][0] = (currSlice->all_mymv[frame_num][mbAddrX-21][2][6][0]+currSlice->all_mymv[frame_num][mbAddrX-21][2][7][0])/2;
		pmvc_b44y1[3][1] = (currSlice->all_mymv[frame_num][mbAddrX-21][2][6][1]+currSlice->all_mymv[frame_num][mbAddrX-21][2][7][1])/2;
		pmvc_b44x2[3][0] = (currSlice->all_mymv[frame_num][mbAddrX-21][3][6][0]+currSlice->all_mymv[frame_num][mbAddrX-21][3][7][0])/2;
		pmvc_b44y2[3][1] = (currSlice->all_mymv[frame_num][mbAddrX-21][3][6][1]+currSlice->all_mymv[frame_num][mbAddrX-21][3][7][1])/2;
		pmvc_c44x1[3][0] = (currSlice->all_mymv[frame_num][mbAddrX-21][2][6][0]+currSlice->all_mymv[frame_num][mbAddrX-21][3][6][0])/2;
		pmvc_c44y1[3][1] = (currSlice->all_mymv[frame_num][mbAddrX-21][2][6][1]+currSlice->all_mymv[frame_num][mbAddrX-21][3][6][1])/2;
		pmvc_c44x2[3][0] = (currSlice->all_mymv[frame_num][mbAddrX-21][2][7][0]+currSlice->all_mymv[frame_num][mbAddrX-21][3][7][0])/2;
		pmvc_c44y2[3][1] = (currSlice->all_mymv[frame_num][mbAddrX-21][2][7][1]+currSlice->all_mymv[frame_num][mbAddrX-21][3][7][1])/2;
		
		pmvc_d44x1[3] = (pmvc_b44x1[3][0]+pmvc_c44x1[3][0]);
		pmvc_d44y1[3] = (pmvc_b44y1[3][1]+pmvc_c44y1[3][1]);
		pmvc_d44x2[3] = (pmvc_b44x2[3][0]+pmvc_c44x2[3][0]);
		pmvc_d44y2[3] = (pmvc_b44y2[3][1]+pmvc_c44y2[3][1]);

		pmvc44x[3][0] = (pmvc_d44x1[3]+pmvc_d44x2[3])/4;
		pmvc44y[3][1] = (pmvc_d44y1[3]+pmvc_d44y2[3])/4;
		}
	if(currSlice->mymv_best[frame_num][mbAddrX-21][3][2]==6 || currSlice->mymv_best[frame_num][mbAddrX-21][3][1]==5){
		c=1;
		pmvc44x[3][0]=(currSlice->all_mymv[frame_num][mbAddrX-21][2][6][0]+currSlice->all_mymv[frame_num][mbAddrX-21][2][6][0]+
					   currSlice->all_mymv[frame_num][mbAddrX-21][2][7][0]+currSlice->all_mymv[frame_num][mbAddrX-21][1][6][0])/4;
		pmvc44y[3][1]=(currSlice->all_mymv[frame_num][mbAddrX-21][2][6][1]+currSlice->all_mymv[frame_num][mbAddrX-21][2][6][1]+
					   currSlice->all_mymv[frame_num][mbAddrX-21][2][7][1]+currSlice->all_mymv[frame_num][mbAddrX-21][1][6][1])/4;
		}
		else if(currSlice->mymv_best[frame_num][mbAddrX-21][0][3]==4){
		c=1;
		pmvc44x[3][0]=currSlice->all_mymv[frame_num][mbAddrX-21][2][6][0];
		pmvc44y[3][1]=currSlice->all_mymv[frame_num][mbAddrX-21][2][6][1];
		}
		
	if(c && c1){
	
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
}
if(currSlice->frame_num >1)
	{											   			
		//printf("frame_num=%d  pmvd=%d\n",frame_num,currSlice->mymv_best[frame_num-1][mbAddrX][0][0]);
	if(currSlice->mymv_best[frame_num-1][mbAddrX][0][0]==0){
		pmvddx=currSlice->all_mymv[frame_num-1][mbAddrX][0][4][0];
		pmvddy=currSlice->all_mymv[frame_num-1][mbAddrX][0][4][1];
		d1=0;
	}
	
	if(currSlice->mymv_best[frame_num-1][mbAddrX][0][0]==1){
		pmvddx=currSlice->all_mymv[frame_num-1][mbAddrX][0][4][0];
		pmvddy=currSlice->all_mymv[frame_num-1][mbAddrX][0][4][1];
		d1=0;
	}
	 if(currSlice->mymv_best[frame_num-1][mbAddrX][0][0]==2){
	
		pmvddx=(currSlice->all_mymv[frame_num-1][mbAddrX][0][2][0]+currSlice->all_mymv[frame_num-1][mbAddrX][2][4][0]+
				currSlice->all_mymv[frame_num-1][mbAddrX][0][4][0]+currSlice->all_mymv[frame_num-1][mbAddrX][0][6][0])/4;
		pmvddy=(currSlice->all_mymv[frame_num-1][mbAddrX][0][2][1]+currSlice->all_mymv[frame_num-1][mbAddrX][2][4][1]+
				currSlice->all_mymv[frame_num-1][mbAddrX][0][4][1]+currSlice->all_mymv[frame_num-1][mbAddrX][0][6][1])/4;
		d1=0;
	}
	 if(currSlice->mymv_best[frame_num-1][mbAddrX][0][0]==3){
	
		pmvddx=(currSlice->all_mymv[frame_num-1][mbAddrX][0][4][0]+currSlice->all_mymv[frame_num-1][mbAddrX][0][6][0]+
				currSlice->all_mymv[frame_num-1][mbAddrX][0][2][0]+currSlice->all_mymv[frame_num-1][mbAddrX][2][4][0])/4;
		pmvddy=(currSlice->all_mymv[frame_num-1][mbAddrX][0][4][1]+currSlice->all_mymv[frame_num-1][mbAddrX][0][6][1]+
				currSlice->all_mymv[frame_num-1][mbAddrX][0][2][1]+currSlice->all_mymv[frame_num-1][mbAddrX][2][4][1])/4;
		d1=0;
	}
	
	
	 if((currSlice->mymv_best[frame_num-1][mbAddrX][0][0] && currSlice->mymv_best[frame_num-1][mbAddrX][0][1] && currSlice->mymv_best[frame_num-1][mbAddrX][0][2] && currSlice->mymv_best[frame_num-1][mbAddrX][0][3])==4){
		pmvdbx[0]=(currSlice->all_mymv[frame_num-1][mbAddrX][0][2][0]+currSlice->all_mymv[frame_num-1][mbAddrX][2][4][0])/2;
		pmvdby[0]=(currSlice->all_mymv[frame_num-1][mbAddrX][0][2][1]+currSlice->all_mymv[frame_num-1][mbAddrX][2][4][1])/2;
		pmvdbx[1]=(currSlice->all_mymv[frame_num-1][mbAddrX][0][4][0]+currSlice->all_mymv[frame_num-1][mbAddrX][2][6][0])/2;
		pmvdby[1]=(currSlice->all_mymv[frame_num-1][mbAddrX][0][4][1]+currSlice->all_mymv[frame_num-1][mbAddrX][2][6][1])/2;
		
		
		pmvdcx[0]=(currSlice->all_mymv[frame_num-1][mbAddrX][0][4][0]+currSlice->all_mymv[frame_num-1][mbAddrX][0][2][0])/2;
		pmvdcy[0]=(currSlice->all_mymv[frame_num-1][mbAddrX][0][4][1]+currSlice->all_mymv[frame_num-1][mbAddrX][0][2][1])/2;
		pmvdcx[1]=(currSlice->all_mymv[frame_num-1][mbAddrX][2][4][0]+currSlice->all_mymv[frame_num-1][mbAddrX][2][6][0])/2;
		pmvdcy[1]=(currSlice->all_mymv[frame_num-1][mbAddrX][2][4][1]+currSlice->all_mymv[frame_num-1][mbAddrX][2][6][1])/2;
	  
		pmvddx=(pmvdbx[0]+pmvdcx[0]+pmvdbx[1]+pmvdcx[1])/4;
		pmvddy=(pmvdby[0]+pmvdcy[0]+pmvdby[1]+pmvdcy[1])/4;
		d1=0;
	}

	
	if(currSlice->mymv_best[frame_num-1][mbAddrX][0][3]==7){
	    d=1;
		pmvd_b44x1[0][0] = (currSlice->all_mymv[frame_num-1][mbAddrX][0][4][0]+currSlice->all_mymv[frame_num-1][mbAddrX][0][5][0])/2;
		pmvd_b44y1[0][1] = (currSlice->all_mymv[frame_num-1][mbAddrX][0][4][1]+currSlice->all_mymv[frame_num-1][mbAddrX][0][5][1])/2;
		pmvd_b44x2[0][0] = (currSlice->all_mymv[frame_num-1][mbAddrX][1][4][0]+currSlice->all_mymv[frame_num-1][mbAddrX][1][5][0])/2;
		pmvd_b44y2[0][1] = (currSlice->all_mymv[frame_num-1][mbAddrX][1][4][1]+currSlice->all_mymv[frame_num-1][mbAddrX][1][5][1])/2;
		pmvd_c44x1[0][0] = (currSlice->all_mymv[frame_num-1][mbAddrX][0][4][0]+currSlice->all_mymv[frame_num-1][mbAddrX][1][4][0])/2;
		pmvd_c44y1[0][1] = (currSlice->all_mymv[frame_num-1][mbAddrX][0][4][1]+currSlice->all_mymv[frame_num-1][mbAddrX][1][4][1])/2;
		pmvd_c44x2[0][0] = (currSlice->all_mymv[frame_num-1][mbAddrX][0][5][0]+currSlice->all_mymv[frame_num-1][mbAddrX][1][5][0])/2;
		pmvd_c44y2[0][1] = (currSlice->all_mymv[frame_num-1][mbAddrX][0][5][1]+currSlice->all_mymv[frame_num-1][mbAddrX][1][5][1])/2;	
		pmvd_d44x1[0] = (pmvd_b44x1[0][0]+pmvd_c44x1[0][0]);
		pmvd_d44y1[0] = (pmvd_b44y1[0][1]+pmvd_c44y1[0][1]);
		pmvd_d44x2[0] = (pmvd_b44x2[0][0]+pmvd_c44x2[0][0]);
		pmvd_d44y2[0] = (pmvd_b44y2[0][1]+pmvd_c44y2[0][1]);
	
		pmvd44x[0][0] = (pmvd_d44x1[0]+pmvd_d44x2[0])/4;
		pmvd44y[0][1] = (pmvd_d44y1[0]+pmvd_d44y2[0])/4;
	
		}
		
	 if(currSlice->mymv_best[frame_num-1][mbAddrX][0][2]==6 || currSlice->mymv_best[frame_num-1][mbAddrX][0][1]==5){
		d=1;
		pmvd44x[0][0]=(currSlice->all_mymv[frame_num-1][mbAddrX][0][2][0]+currSlice->all_mymv[frame_num-1][mbAddrX][0][2][0]+
					   currSlice->all_mymv[frame_num-1][mbAddrX][0][3][0]+currSlice->all_mymv[frame_num-1][mbAddrX][-1][2][0])/4;
		pmvd44y[0][1]=(currSlice->all_mymv[frame_num-1][mbAddrX][0][2][1]+currSlice->all_mymv[frame_num-1][mbAddrX][0][2][1]+
					   currSlice->all_mymv[frame_num-1][mbAddrX][0][3][1]+currSlice->all_mymv[frame_num-1][mbAddrX][-1][2][1])/4;
		}
		else if(currSlice->mymv_best[frame_num-1][mbAddrX][0][0]==4){
		a=1;
		pmvd44x[0][0]=currSlice->all_mymv[frame_num-1][mbAddrX][0][2][0];
		pmvd44y[0][1]=currSlice->all_mymv[frame_num-1][mbAddrX][0][2][1];
		}
	if(currSlice->mymv_best[frame_num-1][mbAddrX][1][3]==7){
		d=1;
		pmvd_b44x1[1][0] = (currSlice->all_mymv[frame_num-1][mbAddrX][0][2][0]+currSlice->all_mymv[frame_num-1][mbAddrX][0][3][0])/2;
		pmvd_b44y1[1][1] = (currSlice->all_mymv[frame_num-1][mbAddrX][0][2][1]+currSlice->all_mymv[frame_num-1][mbAddrX][0][3][1])/2;
		pmvd_b44x2[1][0] = (currSlice->all_mymv[frame_num-1][mbAddrX][1][2][0]+currSlice->all_mymv[frame_num-1][mbAddrX][1][3][0])/2;
		pmvd_b44y2[1][1] = (currSlice->all_mymv[frame_num-1][mbAddrX][1][2][1]+currSlice->all_mymv[frame_num-1][mbAddrX][1][3][1])/2;
		pmvd_c44x1[1][0] = (currSlice->all_mymv[frame_num-1][mbAddrX][0][2][0]+currSlice->all_mymv[frame_num-1][mbAddrX][1][2][0])/2;
		pmvd_c44y1[1][1] = (currSlice->all_mymv[frame_num-1][mbAddrX][0][2][1]+currSlice->all_mymv[frame_num-1][mbAddrX][1][2][1])/2;
		pmvd_c44x2[1][0] = (currSlice->all_mymv[frame_num-1][mbAddrX][0][3][0]+currSlice->all_mymv[frame_num-1][mbAddrX][1][3][0])/2;
		pmvd_c44y2[1][1] = (currSlice->all_mymv[frame_num-1][mbAddrX][0][3][1]+currSlice->all_mymv[frame_num-1][mbAddrX][1][3][1])/2;
		
		pmvd_d44x1[1] = (pmvd_b44x1[1][0]+pmvd_c44x1[1][0]);
		pmvd_d44y1[1] = (pmvd_b44y1[1][1]+pmvd_c44y1[1][1]);
		pmvd_d44x2[1] = (pmvd_b44x2[1][0]+pmvd_c44x2[1][0]);
		pmvd_d44y2[1] = (pmvd_b44y2[1][1]+pmvd_c44y2[1][1]);
	
		pmvd44x[1][0] = (pmvd_d44x1[1]+pmvd_d44x2[1])/4;
		pmvd44y[1][1] = (pmvd_d44y1[1]+pmvd_d44y2[1])/4;
		
		}
	 if(currSlice->mymv_best[frame_num-1][mbAddrX][1][2]==6 || currSlice->mymv_best[frame_num-1][mbAddrX][1][1]==5){
		d=1;
		pmvd44x[1][0]=(currSlice->all_mymv[frame_num-1][mbAddrX][0][4][0]+currSlice->all_mymv[frame_num-1][mbAddrX][0][4][0]+
					   currSlice->all_mymv[frame_num-1][mbAddrX][0][5][0]+currSlice->all_mymv[frame_num-1][mbAddrX][-1][4][0])/4;
		pmvd44y[1][1]=(currSlice->all_mymv[frame_num-1][mbAddrX][0][4][1]+currSlice->all_mymv[frame_num-1][mbAddrX][0][4][1]+
					   currSlice->all_mymv[frame_num-1][mbAddrX][0][5][1]+currSlice->all_mymv[frame_num-1][mbAddrX][-1][4][1])/4;
		}
		else if(currSlice->mymv_best[frame_num-1][mbAddrX][0][1]==4){
		d=1;
		pmvd44x[1][0]=currSlice->all_mymv[frame_num-1][mbAddrX][0][4][0];
		pmvd44y[1][1]=currSlice->all_mymv[frame_num-1][mbAddrX][0][4][1];
		}
	if(currSlice->mymv_best[frame_num-1][mbAddrX][2][3]==7){
		d=1;
		pmvd_b44x1[2][0] = (currSlice->all_mymv[frame_num-1][mbAddrX][2][4][0]+currSlice->all_mymv[frame_num-1][mbAddrX][2][5][0])/2;
		pmvd_b44y1[2][1] = (currSlice->all_mymv[frame_num-1][mbAddrX][2][4][1]+currSlice->all_mymv[frame_num-1][mbAddrX][2][5][1])/2;
		pmvd_b44x2[2][0] = (currSlice->all_mymv[frame_num-1][mbAddrX][3][4][0]+currSlice->all_mymv[frame_num-1][mbAddrX][3][5][0])/2;
		pmvd_b44y2[2][1] = (currSlice->all_mymv[frame_num-1][mbAddrX][3][4][1]+currSlice->all_mymv[frame_num-1][mbAddrX][3][5][1])/2;
		pmvd_c44x1[2][0] = (currSlice->all_mymv[frame_num-1][mbAddrX][2][4][0]+currSlice->all_mymv[frame_num-1][mbAddrX][3][4][0])/2;
		pmvd_c44y1[2][1] = (currSlice->all_mymv[frame_num-1][mbAddrX][2][4][1]+currSlice->all_mymv[frame_num-1][mbAddrX][3][4][1])/2;
		pmvd_c44x2[2][0] = (currSlice->all_mymv[frame_num-1][mbAddrX][2][5][0]+currSlice->all_mymv[frame_num-1][mbAddrX][3][5][0])/2;
		pmvd_c44y2[2][1] = (currSlice->all_mymv[frame_num-1][mbAddrX][2][5][1]+currSlice->all_mymv[frame_num-1][mbAddrX][3][5][1])/2;
		
		pmvd_d44x1[2] = (pmvd_b44x1[2][0]+pmvd_c44x1[2][0]);
		pmvd_d44y1[2] = (pmvd_b44y1[2][1]+pmvd_c44y1[2][1]);
		pmvd_d44x2[2] = (pmvd_b44x2[2][0]+pmvd_c44x2[2][0]);
		pmvd_d44y2[2] = (pmvd_b44y2[2][1]+pmvd_c44y2[2][1]);
	
		pmvd44x[2][0] = (pmvd_d44x1[2]+pmvd_d44x2[2])/4;
		pmvd44y[2][1] = (pmvd_d44y1[2]+pmvd_d44y2[2])/4;
		
		}
	 if(currSlice->mymv_best[frame_num-1][mbAddrX][2][2]==6 || currSlice->mymv_best[frame_num-1][mbAddrX][2][1]==5){
		d=1;
		pmvd44x[2][0]=(currSlice->all_mymv[frame_num-1][mbAddrX][2][4][0]+currSlice->all_mymv[frame_num-1][mbAddrX][2][4][0]+
					   currSlice->all_mymv[frame_num-1][mbAddrX][2][5][0]+currSlice->all_mymv[frame_num-1][mbAddrX][1][4][0])/4;
		pmvd44y[2][1]=(currSlice->all_mymv[frame_num-1][mbAddrX][2][4][1]+currSlice->all_mymv[frame_num-1][mbAddrX][2][4][1]+
					   currSlice->all_mymv[frame_num-1][mbAddrX][2][5][1]+currSlice->all_mymv[frame_num-1][mbAddrX][1][4][1])/4;
		}
		else if(currSlice->mymv_best[frame_num-1][mbAddrX][0][2]==4){
		d=1;
		pmvd44x[2][0]=currSlice->all_mymv[frame_num-1][mbAddrX][2][4][0];
		pmvd44y[2][1]=currSlice->all_mymv[frame_num-1][mbAddrX][2][4][1];
		}
	if(currSlice->mymv_best[frame_num-1][mbAddrX][3][3]==7){
		d=1;
		pmvd_b44x1[3][0] = (currSlice->all_mymv[frame_num-1][mbAddrX][2][6][0]+currSlice->all_mymv[frame_num-1][mbAddrX][2][7][0])/2;
		pmvd_b44y1[3][1] = (currSlice->all_mymv[frame_num-1][mbAddrX][2][6][1]+currSlice->all_mymv[frame_num-1][mbAddrX][2][7][1])/2;
		pmvd_b44x2[3][0] = (currSlice->all_mymv[frame_num-1][mbAddrX][3][6][0]+currSlice->all_mymv[frame_num-1][mbAddrX][3][7][0])/2;
		pmvd_b44y2[3][1] = (currSlice->all_mymv[frame_num-1][mbAddrX][3][6][1]+currSlice->all_mymv[frame_num-1][mbAddrX][3][7][1])/2;
		pmvd_c44x1[3][0] = (currSlice->all_mymv[frame_num-1][mbAddrX][2][6][0]+currSlice->all_mymv[frame_num-1][mbAddrX][3][6][0])/2;
		pmvd_c44y1[3][1] = (currSlice->all_mymv[frame_num-1][mbAddrX][2][6][1]+currSlice->all_mymv[frame_num-1][mbAddrX][3][6][1])/2;
		pmvd_c44x2[3][0] = (currSlice->all_mymv[frame_num-1][mbAddrX][2][7][0]+currSlice->all_mymv[frame_num-1][mbAddrX][3][7][0])/2;
		pmvd_c44y2[3][1] = (currSlice->all_mymv[frame_num-1][mbAddrX][2][7][1]+currSlice->all_mymv[frame_num-1][mbAddrX][3][7][1])/2;
		
		pmvd_d44x1[3] = (pmvd_b44x1[3][0]+pmvd_c44x1[3][0]);
		pmvd_d44y1[3] = (pmvd_b44y1[3][1]+pmvd_c44y1[3][1]);
		pmvd_d44x2[3] = (pmvd_b44x2[3][0]+pmvd_c44x2[3][0]);
		pmvd_d44y2[3] = (pmvd_b44y2[3][1]+pmvd_c44y2[3][1]);
	
		pmvd44x[3][0] = (pmvd_d44x1[3]+pmvd_d44x2[3])/4;
		pmvd44y[3][1] = (pmvd_d44y1[3]+pmvd_d44y2[3])/4;
		
		}
	 if(currSlice->mymv_best[frame_num-1][mbAddrX][3][2]==6 || currSlice->mymv_best[frame_num-1][mbAddrX][3][1]==5){
		d=1;
		pmvd44x[3][0]=(currSlice->all_mymv[frame_num-1][mbAddrX][2][6][0]+currSlice->all_mymv[frame_num-1][mbAddrX][2][6][0]+
					   currSlice->all_mymv[frame_num-1][mbAddrX][2][7][0]+currSlice->all_mymv[frame_num-1][mbAddrX][1][6][0])/4;
		pmvd44y[3][1]=(currSlice->all_mymv[frame_num-1][mbAddrX][2][6][1]+currSlice->all_mymv[frame_num-1][mbAddrX][2][6][1]+
					   currSlice->all_mymv[frame_num-1][mbAddrX][2][7][1]+currSlice->all_mymv[frame_num-1][mbAddrX][1][6][1])/4;
		}
		else if(currSlice->mymv_best[frame_num-1][mbAddrX][0][3]==4){
		d=1;
		pmvd44x[3][0]=currSlice->all_mymv[frame_num-1][mbAddrX][2][6][0];
		pmvd44y[3][1]=currSlice->all_mymv[frame_num-1][mbAddrX][2][6][1];
		}
	
	if(d && d1){

	pmvdbx[0]=(pmvd44x[0][0]+pmvd44x[2][0])/2;
	pmvdby[0]=(pmvd44y[0][1]+pmvd44y[2][1])/2;
	pmvdbx[1]=(pmvd44x[1][0]+pmvd44x[3][0])/2;
	pmvdby[1]=(pmvd44y[1][1]+pmvd44y[3][1])/2;
	
	
	pmvdcx[0]=(pmvd44x[0][0]+pmvd44x[1][0])/2;
	pmvdcy[0]=(pmvd44y[0][1]+pmvd44y[1][1])/2;
	pmvdcx[1]=(pmvd44x[2][0]+pmvd44x[3][0])/2;
	pmvdcy[1]=(pmvd44y[2][1]+pmvd44y[3][1])/2;
  
    pmvddx=(pmvdbx[0]+pmvdcx[0]+pmvdbx[1]+pmvdcx[1])/4;
	pmvddy=(pmvdby[0]+pmvdcy[0]+pmvdby[1]+pmvdcy[1])/4;
	
	}
	
	}
	

		

	
	if(currMB->mb_x==0){
		if(block[1].available)  L=fabs(pmvbdx)+fabs(pmvbdy); 
	//	pmv[0] = (short) pmvbdx;
	//	pmv[1] = (short) pmvbdy;
		} //boundary checked
	else if(currMB->mb_y==0) {
		if(block[0].available)	L=fabs(pmvadx)+fabs(pmvady); 
	//	pmv[0] = (short) pmvadx;
	//	pmv[1] = (short) pmvady;
		}
	else if(currMB->mb_x==21) {
		if(block[0].available)	L=fabs(pmvadx)+fabs(pmvady); 
		//pmv[0] = (short) pmvadx;
		//pmv[1] = (short) pmvady;
		}
		
	else
	{
	if(block[0].available)			mva = fabs(pmvadx) + fabs(pmvady);
	if(block[1].available)			mvb = fabs(pmvbdx) + fabs(pmvbdy); 
	if(block[2].available)			mvc = fabs(pmvcdx) + fabs(pmvcdy); 
	if(currSlice->frame_num >1)		mvd = fabs(pmvddx) + fabs(pmvddy); 
	
	if(mva>=mvb && mva>=mvc && mva>=mvd) 
	{
		L=mva;
		//pmv[0] = (short) pmvadx;
		//pmv[1] = (short) pmvady;
		}
	if(mvb>=mvc && mvb>=mva && mvb>=mvd) 
	{
		L=mvb;
		//pmv[0] = (short) pmvbdx;
		//pmv[1] = (short) pmvbdy;
		}
	if(mvc>=mva && mvc>=mvb && mvc>=mvd)
	{
		L=mvc;
		//pmv[0] = (short) pmvcdx;
		//pmv[1] = (short) pmvcdy;
	}
	if(mvd>=mva && mvd>=mvb && mvd>=mvc)
	{
		L=mvd;
		//pmv[0] = (short) pmvcdx;
		//pmv[1] = (short) pmvcdy;
	}
	}
	//printf("pmvadx=%d pmvady=%d\n",pmvadx,pmvady);
	  // 	printf("pmvbdx=%d pmvbdy=%d\n",pmvbdx,pmvbdy);
	//	printf("pmvd=%d\n",mvd);
	//currMB->pmva =mva;
	//currMB->pmvb =mvb;
	//currMB->pmvc =mvc;
	currMB->L[mbAddrX] =L;
	//if(currMB->L[mbAddrX] ==2)//system("pause");
	//printf ("currMB->L[mbAddr]=%d=( %f )\n",mbAddrX,currMB->L[mbAddrX]);
}

if(currMB->mode0_flag == 1){
  mvPredType = MVPRED_MEDIAN;

  rFrameL    = block[0].available ? refPic[block[0].pos_y][block[0].pos_x] : -1;
  rFrameU    = block[1].available ? refPic[block[1].pos_y][block[1].pos_x] : -1;
  rFrameUR   = block[2].available ? refPic[block[2].pos_y][block[2].pos_x] : -1;
 

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
	//printf("tmp_mv[block[0].pos_y][block[0].pos_x][hv]=%d\n",tmp_mv[block[0].pos_y][block[0].pos_x][hv]);
//	printf("tmp_mv[block[1].pos_y][block[0].pos_x][hv]=%d\n",tmp_mv[block[1].pos_y][block[1].pos_x][hv]);
//	printf("tmp_mv[block[2].pos_y][block[0].pos_x][hv]=%d\n",tmp_mv[block[2].pos_y][block[2].pos_x][hv]);
	//printf ("mva=(%d )\n",mva);
	//printf ("mvb=(%d )\n",mvb);	
	//printf ("mvc=(%d )\n",mvc);
/*
	mva += abs(mv_a);
	mvb += abs(mv_b);
	mvc += abs(mv_c);
	
	if(currMB->mb_x==0)L=mvb;  //boundary checked
	else if(currMB->mb_y==0) L=mva;
	else if(currMB->mb_x==21) L=mva;
	if(mva>=mvb && mva>=mvc) L=mva;
	if(mvb>=mvc && mvb>=mva) L=mvb;
	if(mvc>=mva && mvc>=mvb) L=mvc;
	currMB->L[mbAddrX] =L;
	
	//printf ("currMB->L[mbAddr-1=%d=( %d )\n",mbAddrX-1,currMB->L[mbAddrX-1]);
	
	*/
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
 
 //printf ("L=(%d )\n",currMB->L);

  /*
static void GetMotionVectorPredictorL (Macroblock *currMB, 
                                     PixelPos *block      // <--> block neighbors
                                     )
{
  int mv_a, mv_b, mv_c, pred_vec = 0;
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
  

	
  if(currMB->mbAddrX!=0){	
	if(block[0].available){
		
	if(currSlice->mymv_best[mbAddrX-1][0][3]==7){
		pmva_b44x1[0][0] = (currSlice->all_mymv[mbAddrX-1][0][4][0]+currSlice->all_mymv[mbAddrX-1][0][5][0])/2;
		pmva_b44y1[0][1] = (currSlice->all_mymv[mbAddrX-1][0][4][1]+currSlice->all_mymv[mbAddrX-1][0][5][1])/2;
		pmva_b44x2[0][0] = (currSlice->all_mymv[mbAddrX-1][1][4][0]+currSlice->all_mymv[mbAddrX-1][1][5][0])/2;
		pmva_b44y2[0][1] = (currSlice->all_mymv[mbAddrX-1][1][4][1]+currSlice->all_mymv[mbAddrX-1][1][5][1])/2;
		pmva_c44x1[0][0] = (currSlice->all_mymv[mbAddrX-1][0][4][0]+currSlice->all_mymv[mbAddrX-1][1][4][0])/2;
		pmva_c44y1[0][1] = (currSlice->all_mymv[mbAddrX-1][0][4][1]+currSlice->all_mymv[mbAddrX-1][1][4][1])/2;
		pmva_c44x2[0][0] = (currSlice->all_mymv[mbAddrX-1][0][5][0]+currSlice->all_mymv[mbAddrX-1][1][5][0])/2;
		pmva_c44y2[0][1] = (currSlice->all_mymv[mbAddrX-1][0][5][1]+currSlice->all_mymv[mbAddrX-1][1][5][1])/2;	
		
		pmva_d44x1[0] = (pmva_b44x1[0][0]+pmva_c44x1[0][0]);
		pmva_d44y1[0] = (pmva_b44y1[0][1]+pmva_c44y1[0][1]);
		pmva_d44x2[0] = (pmva_b44x2[0][0]+pmva_c44x2[0][0]);
		pmva_d44y2[0] = (pmva_b44y2[0][1]+pmva_c44y2[0][1]);
	
		pmva44x[0][0] = (pmva_d44x1[0]+pmva_d44x2[0])/4;
		pmva44y[0][1] = (pmva_d44y1[0]+pmva_d44y2[0])/4;
		}
	else if(currSlice->mymv_best[mbAddrX-1][0][2]==6 || currSlice->mymv_best[mbAddrX-1][0][1]==5){
		pmva44x[0][0]=(currSlice->all_mymv[mbAddrX-1][0][4][0]+currSlice->all_mymv[mbAddrX-1][0][4][0]+
					   currSlice->all_mymv[mbAddrX-1][0][5][0]+currSlice->all_mymv[mbAddrX-1][-1][4][0])/4;
		pmva44y[0][1]=(currSlice->all_mymv[mbAddrX-1][0][4][1]+currSlice->all_mymv[mbAddrX-1][0][4][1]+
					   currSlice->all_mymv[mbAddrX-1][0][5][1]+currSlice->all_mymv[mbAddrX-1][-1][4][1])/4;
		}
	if(currSlice->mymv_best[mbAddrX-1][1][3]==7){
		pmva_b44x1[1][0] = (currSlice->all_mymv[mbAddrX-1][0][2][0]+currSlice->all_mymv[mbAddrX-1][0][3][0])/2;
		pmva_b44y1[1][1] = (currSlice->all_mymv[mbAddrX-1][0][2][1]+currSlice->all_mymv[mbAddrX-1][0][3][1])/2;
		pmva_b44x2[1][0] = (currSlice->all_mymv[mbAddrX-1][1][2][0]+currSlice->all_mymv[mbAddrX-1][1][3][0])/2;
		pmva_b44y2[1][1] = (currSlice->all_mymv[mbAddrX-1][1][2][1]+currSlice->all_mymv[mbAddrX-1][1][3][1])/2;
		pmva_c44x1[1][0] = (currSlice->all_mymv[mbAddrX-1][0][2][0]+currSlice->all_mymv[mbAddrX-1][1][2][0])/2;
		pmva_c44y1[1][1] = (currSlice->all_mymv[mbAddrX-1][0][2][1]+currSlice->all_mymv[mbAddrX-1][1][2][1])/2;
		pmva_c44x2[1][0] = (currSlice->all_mymv[mbAddrX-1][0][3][0]+currSlice->all_mymv[mbAddrX-1][1][3][0])/2;
		pmva_c44y2[1][1] = (currSlice->all_mymv[mbAddrX-1][0][3][1]+currSlice->all_mymv[mbAddrX-1][1][3][1])/2;
		
		pmva_d44x1[1] = (pmva_b44x1[1][0]+pmva_c44x1[1][0]);
		pmva_d44y1[1] = (pmva_b44y1[1][1]+pmva_c44y1[1][1]);
		pmva_d44x2[1] = (pmva_b44x2[1][0]+pmva_c44x2[1][0]);
		pmva_d44y2[1] = (pmva_b44y2[1][1]+pmva_c44y2[1][1]);
	
		pmva44x[1][0] = (pmva_d44x1[1]+pmva_d44x2[1])/4;
		pmva44y[1][1] = (pmva_d44y1[1]+pmva_d44y2[1])/4;
		
		}
	else if(currSlice->mymv_best[mbAddrX-1][1][2]==6 || currSlice->mymv_best[mbAddrX-1][1][1]==5){
		pmva44x[1][0]=(currSlice->all_mymv[mbAddrX-1][0][2][0]+currSlice->all_mymv[mbAddrX-1][0][2][0]+
					   currSlice->all_mymv[mbAddrX-1][0][3][0]+currSlice->all_mymv[mbAddrX-1][-1][2][0])/4;
		pmva44y[1][1]=(currSlice->all_mymv[mbAddrX-1][0][2][1]+currSlice->all_mymv[mbAddrX-1][0][2][1]+
					   currSlice->all_mymv[mbAddrX-1][0][3][1]+currSlice->all_mymv[mbAddrX-1][-1][2][1])/4;
		}
	if(currSlice->mymv_best[mbAddrX-1][2][3]==7){
		pmva_b44x1[2][0] = (currSlice->all_mymv[mbAddrX-1][2][4][0]+currSlice->all_mymv[mbAddrX-1][2][5][0])/2;
		pmva_b44y1[2][1] = (currSlice->all_mymv[mbAddrX-1][2][4][1]+currSlice->all_mymv[mbAddrX-1][2][5][1])/2;
		pmva_b44x2[2][0] = (currSlice->all_mymv[mbAddrX-1][3][4][0]+currSlice->all_mymv[mbAddrX-1][3][5][0])/2;
		pmva_b44y2[2][1] = (currSlice->all_mymv[mbAddrX-1][3][4][1]+currSlice->all_mymv[mbAddrX-1][3][5][1])/2;
		pmva_c44x1[2][0] = (currSlice->all_mymv[mbAddrX-1][2][4][0]+currSlice->all_mymv[mbAddrX-1][3][4][0])/2;
		pmva_c44y1[2][1] = (currSlice->all_mymv[mbAddrX-1][2][4][1]+currSlice->all_mymv[mbAddrX-1][3][4][1])/2;
		pmva_c44x2[2][0] = (currSlice->all_mymv[mbAddrX-1][2][5][0]+currSlice->all_mymv[mbAddrX-1][3][5][0])/2;
		pmva_c44y2[2][1] = (currSlice->all_mymv[mbAddrX-1][2][5][1]+currSlice->all_mymv[mbAddrX-1][3][5][1])/2;
		
		pmva_d44x1[2] = (pmva_b44x1[2][0]+pmva_c44x1[2][0]);
		pmva_d44y1[2] = (pmva_b44y1[2][1]+pmva_c44y1[2][1]);
		pmva_d44x2[2] = (pmva_b44x2[2][0]+pmva_c44x2[2][0]);
		pmva_d44y2[2] = (pmva_b44y2[2][1]+pmva_c44y2[2][1]);
	
		pmva44x[2][0] = (pmva_d44x1[2]+pmva_d44x2[2])/4;
		pmva44y[2][1] = (pmva_d44y1[2]+pmva_d44y2[2])/4;
		
		}
	else if(currSlice->mymv_best[mbAddrX-1][2][2]==6 || currSlice->mymv_best[mbAddrX-1][2][1]==5){
		pmva44x[2][0]=(currSlice->all_mymv[mbAddrX-1][2][4][0]+currSlice->all_mymv[mbAddrX-1][2][4][0]+
					   currSlice->all_mymv[mbAddrX-1][2][5][0]+currSlice->all_mymv[mbAddrX-1][1][4][0])/4;
		pmva44y[2][1]=(currSlice->all_mymv[mbAddrX-1][2][4][1]+currSlice->all_mymv[mbAddrX-1][2][4][1]+
					   currSlice->all_mymv[mbAddrX-1][2][5][1]+currSlice->all_mymv[mbAddrX-1][1][4][1])/4;
		}
	if(currSlice->mymv_best[mbAddrX-1][3][3]==7){
		pmva_b44x1[3][0] = (currSlice->all_mymv[mbAddrX-1][2][6][0]+currSlice->all_mymv[mbAddrX-1][2][7][0])/2;
		pmva_b44y1[3][1] = (currSlice->all_mymv[mbAddrX-1][2][6][1]+currSlice->all_mymv[mbAddrX-1][2][7][1])/2;
		pmva_b44x2[3][0] = (currSlice->all_mymv[mbAddrX-1][3][6][0]+currSlice->all_mymv[mbAddrX-1][3][7][0])/2;
		pmva_b44y2[3][1] = (currSlice->all_mymv[mbAddrX-1][3][6][1]+currSlice->all_mymv[mbAddrX-1][3][7][1])/2;
		pmva_c44x1[3][0] = (currSlice->all_mymv[mbAddrX-1][2][6][0]+currSlice->all_mymv[mbAddrX-1][3][6][0])/2;
		pmva_c44y1[3][1] = (currSlice->all_mymv[mbAddrX-1][2][6][1]+currSlice->all_mymv[mbAddrX-1][3][6][1])/2;
		pmva_c44x2[3][0] = (currSlice->all_mymv[mbAddrX-1][2][7][0]+currSlice->all_mymv[mbAddrX-1][3][7][0])/2;
		pmva_c44y2[3][1] = (currSlice->all_mymv[mbAddrX-1][2][7][1]+currSlice->all_mymv[mbAddrX-1][3][7][1])/2;
		
		pmva_d44x1[3] = (pmva_b44x1[3][0]+pmva_c44x1[3][0]);
		pmva_d44y1[3] = (pmva_b44y1[3][1]+pmva_c44y1[3][1]);
		pmva_d44x2[3] = (pmva_b44x2[3][0]+pmva_c44x2[3][0]);
		pmva_d44y2[3] = (pmva_b44y2[3][1]+pmva_c44y2[3][1]);
	
		pmva44x[3][0] = (pmva_d44x1[3]+pmva_d44x2[3])/4;
		pmva44y[3][1] = (pmva_d44y1[3]+pmva_d44y2[3])/4;
		}
	else if(currSlice->mymv_best[mbAddrX-1][2][2]==6 || currSlice->mymv_best[mbAddrX-1][2][1]==5){
		pmva44x[3][0]=(currSlice->all_mymv[mbAddrX-1][2][6][0]+currSlice->all_mymv[mbAddrX-1][2][6][0]+
					   currSlice->all_mymv[mbAddrX-1][2][7][0]+currSlice->all_mymv[mbAddrX-1][1][6][0])/4;
		pmva44y[3][1]=(currSlice->all_mymv[mbAddrX-1][2][6][1]+currSlice->all_mymv[mbAddrX-1][2][6][1]+
					   currSlice->all_mymv[mbAddrX-1][2][7][1]+currSlice->all_mymv[mbAddrX-1][1][7][1])/4;
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
	
	 if(currSlice->mymv_best[mbAddrX-1][0][0]==4){
		pmvabx[0]=(currSlice->all_mymv[mbAddrX-1][0][4][0]+currSlice->all_mymv[mbAddrX-1][2][4][0])/2;
		pmvaby[0]=(currSlice->all_mymv[mbAddrX-1][0][4][1]+currSlice->all_mymv[mbAddrX-1][2][4][1])/2;
		pmvabx[1]=(currSlice->all_mymv[mbAddrX-1][0][2][0]+currSlice->all_mymv[mbAddrX-1][2][6][0])/2;
		pmvaby[1]=(currSlice->all_mymv[mbAddrX-1][0][2][1]+currSlice->all_mymv[mbAddrX-1][2][6][1])/2;
		
		
		pmvacx[0]=(currSlice->all_mymv[mbAddrX-1][0][4][0]+currSlice->all_mymv[mbAddrX-1][0][2][0])/2;
		pmvacy[0]=(currSlice->all_mymv[mbAddrX-1][0][4][1]+currSlice->all_mymv[mbAddrX-1][0][2][1])/2;
		pmvacx[1]=(currSlice->all_mymv[mbAddrX-1][2][4][0]+currSlice->all_mymv[mbAddrX-1][2][6][0])/2;
		pmvacy[1]=(currSlice->all_mymv[mbAddrX-1][2][4][1]+currSlice->all_mymv[mbAddrX-1][2][6][1])/2;
	  
		pmvadx=(pmvabx[0]+pmvacx[0]+pmvabx[1]+pmvacx[1])/4;
		pmvady=(pmvaby[0]+pmvacy[0]+pmvaby[1]+pmvacy[1])/4;
	
	}
	 if(currSlice->mymv_best[mbAddrX-1][0][0]==2){
	
		pmvadx=(currSlice->all_mymv[mbAddrX-1][0][2][0]+currSlice->all_mymv[mbAddrX-1][2][4][0]+
				currSlice->all_mymv[mbAddrX-1][0][4][0]+currSlice->all_mymv[mbAddrX-1][2][6][0])/4;
		pmvady=(currSlice->all_mymv[mbAddrX-1][0][2][1]+currSlice->all_mymv[mbAddrX-1][2][4][1]+
				currSlice->all_mymv[mbAddrX-1][0][4][1]+currSlice->all_mymv[mbAddrX-1][2][6][1])/4;
	
	}
	 if(currSlice->mymv_best[mbAddrX-1][0][0]==3){
	
		pmvadx=(currSlice->all_mymv[mbAddrX-1][0][4][0]+currSlice->all_mymv[mbAddrX-1][2][6][0]+
				currSlice->all_mymv[mbAddrX-1][0][2][0]+currSlice->all_mymv[mbAddrX-1][2][4][0])/4;
		pmvady=(currSlice->all_mymv[mbAddrX-1][0][4][1]+currSlice->all_mymv[mbAddrX-1][2][6][1]+
				currSlice->all_mymv[mbAddrX-1][0][2][1]+currSlice->all_mymv[mbAddrX-1][2][4][1])/4;
	
	}
	
	pmvadx=currSlice->all_mymv[mbAddrX-1][0][4][0];
	pmvady=currSlice->all_mymv[mbAddrX-1][0][4][1];
	
	}
	//end a
	
	if(block[1].available){
	if(currSlice->mymv_best[mbAddrX-22][0][3]==7){
		pmvb_b44x1[0][0] = (currSlice->all_mymv[mbAddrX-22][0][4][0]+currSlice->all_mymv[mbAddrX-22][0][5][0])/2;
		pmvb_b44y1[0][1] = (currSlice->all_mymv[mbAddrX-22][0][4][1]+currSlice->all_mymv[mbAddrX-22][0][5][1])/2;
		pmvb_b44x2[0][0] = (currSlice->all_mymv[mbAddrX-22][1][4][0]+currSlice->all_mymv[mbAddrX-22][1][5][0])/2;
		pmvb_b44y2[0][1] = (currSlice->all_mymv[mbAddrX-22][1][4][1]+currSlice->all_mymv[mbAddrX-22][1][5][1])/2;
		pmvb_c44x1[0][0] = (currSlice->all_mymv[mbAddrX-22][0][4][0]+currSlice->all_mymv[mbAddrX-22][1][4][0])/2;
		pmvb_c44y1[0][1] = (currSlice->all_mymv[mbAddrX-22][0][4][1]+currSlice->all_mymv[mbAddrX-22][1][4][1])/2;
		pmvb_c44x2[0][0] = (currSlice->all_mymv[mbAddrX-22][0][5][0]+currSlice->all_mymv[mbAddrX-22][1][5][0])/2;
		pmvb_c44y2[0][1] = (currSlice->all_mymv[mbAddrX-22][0][5][1]+currSlice->all_mymv[mbAddrX-22][1][5][1])/2;	
		
		pmvb_d44x1[0] = (pmvb_b44x1[0][0]+pmvb_c44x1[0][0]);
		pmvb_d44y1[0] = (pmvb_b44y1[0][1]+pmvb_c44y1[0][1]);
		pmvb_d44x2[0] = (pmvb_b44x2[0][0]+pmvb_c44x2[0][0]);
		pmvb_d44y2[0] = (pmvb_b44y2[0][1]+pmvb_c44y2[0][1]);
		
		pmvb44x[0][0] = (pmvb_d44x1[0]+pmvb_d44x2[0])/4;
		pmvb44y[0][1] = (pmvb_d44y1[0]+pmvb_d44y2[0])/4;
		}
		else if(currSlice->mymv_best[mbAddrX-22][0][2]==6 || currSlice->mymv_best[mbAddrX-22][0][1]==5){
		pmvb44x[0][0]=(currSlice->all_mymv[mbAddrX-22][0][4][0]+currSlice->all_mymv[mbAddrX-22][0][4][0]+
					   currSlice->all_mymv[mbAddrX-22][0][5][0]+currSlice->all_mymv[mbAddrX-22][-1][4][0])/4;
		pmvb44y[0][1]=(currSlice->all_mymv[mbAddrX-22][0][4][1]+currSlice->all_mymv[mbAddrX-22][0][4][1]+
					   currSlice->all_mymv[mbAddrX-22][0][5][1]+currSlice->all_mymv[mbAddrX-22][-1][4][1])/4;
		}
	if(currSlice->mymv_best[mbAddrX-22][1][3]==7){	
		pmvb_b44x1[1][0] = (currSlice->all_mymv[mbAddrX-22][0][2][0]+currSlice->all_mymv[mbAddrX-22][0][3][0])/2;
		pmvb_b44y1[1][1] = (currSlice->all_mymv[mbAddrX-22][0][2][1]+currSlice->all_mymv[mbAddrX-22][0][3][1])/2;
		pmvb_b44x2[1][0] = (currSlice->all_mymv[mbAddrX-22][1][2][0]+currSlice->all_mymv[mbAddrX-22][1][3][0])/2;
		pmvb_b44y2[1][1] = (currSlice->all_mymv[mbAddrX-22][1][2][1]+currSlice->all_mymv[mbAddrX-22][1][3][1])/2;
		pmvb_c44x1[1][0] = (currSlice->all_mymv[mbAddrX-22][0][2][0]+currSlice->all_mymv[mbAddrX-22][1][2][0])/2;
		pmvb_c44y1[1][1] = (currSlice->all_mymv[mbAddrX-22][0][2][1]+currSlice->all_mymv[mbAddrX-22][1][2][1])/2;
		pmvb_c44x2[1][0] = (currSlice->all_mymv[mbAddrX-22][0][3][0]+currSlice->all_mymv[mbAddrX-22][1][3][0])/2;
		pmvb_c44y2[1][1] = (currSlice->all_mymv[mbAddrX-22][0][3][1]+currSlice->all_mymv[mbAddrX-22][1][3][1])/2;
		
		pmvb_d44x1[1] = (pmvb_b44x1[1][0]+pmvb_c44x1[1][0]);
		pmvb_d44y1[1] = (pmvb_b44y1[1][1]+pmvb_c44y1[1][1]);
		pmvb_d44x2[1] = (pmvb_b44x2[1][0]+pmvb_c44x2[1][0]);
		pmvb_d44y2[1] = (pmvb_b44y2[1][1]+pmvb_c44y2[1][1]);

		pmvb44x[1][0] = (pmvb_d44x1[1]+pmvb_d44x2[1])/4;
		pmvb44y[1][1] = (pmvb_d44y1[1]+pmvb_d44y2[1])/4;
		}
		else if(currSlice->mymv_best[mbAddrX-22][0][2]==6 || currSlice->mymv_best[mbAddrX-22][0][1]==5){
		pmvb44x[1][0]=(currSlice->all_mymv[mbAddrX-22][0][4][0]+currSlice->all_mymv[mbAddrX-22][0][4][0]+
					   currSlice->all_mymv[mbAddrX-22][0][5][0]+currSlice->all_mymv[mbAddrX-22][-1][4][0])/4;
		pmvb44y[1][1]=(currSlice->all_mymv[mbAddrX-22][0][4][1]+currSlice->all_mymv[mbAddrX-22][0][4][1]+
					   currSlice->all_mymv[mbAddrX-22][0][5][1]+currSlice->all_mymv[mbAddrX-22][-1][4][1])/4;
		}
	if(currSlice->mymv_best[mbAddrX-22][2][3]==7){
		pmvb_b44x1[2][0] = (currSlice->all_mymv[mbAddrX-22][2][4][0]+currSlice->all_mymv[mbAddrX-22][2][5][0])/2;
		pmvb_b44y1[2][1] = (currSlice->all_mymv[mbAddrX-22][2][4][1]+currSlice->all_mymv[mbAddrX-22][2][5][1])/2;
		pmvb_b44x2[2][0] = (currSlice->all_mymv[mbAddrX-22][3][4][0]+currSlice->all_mymv[mbAddrX-22][3][5][0])/2;
		pmvb_b44y2[2][1] = (currSlice->all_mymv[mbAddrX-22][3][4][1]+currSlice->all_mymv[mbAddrX-22][3][5][1])/2;
		pmvb_c44x1[2][0] = (currSlice->all_mymv[mbAddrX-22][2][4][0]+currSlice->all_mymv[mbAddrX-22][3][4][0])/2;
		pmvb_c44y1[2][1] = (currSlice->all_mymv[mbAddrX-22][2][4][1]+currSlice->all_mymv[mbAddrX-22][3][4][1])/2;
		pmvb_c44x2[2][0] = (currSlice->all_mymv[mbAddrX-22][2][5][0]+currSlice->all_mymv[mbAddrX-22][3][5][0])/2;
		pmvb_c44y2[2][1] = (currSlice->all_mymv[mbAddrX-22][2][5][1]+currSlice->all_mymv[mbAddrX-22][3][5][1])/2;
		
		pmvb_d44x1[2] = (pmvb_b44x1[2][0]+pmvb_c44x1[2][0]);
		pmvb_d44y1[2] = (pmvb_b44y1[2][1]+pmvb_c44y1[2][1]);
		pmvb_d44x2[2] = (pmvb_b44x2[2][0]+pmvb_c44x2[2][0]);
		pmvb_d44y2[2] = (pmvb_b44y2[2][1]+pmvb_c44y2[2][1]);
		

		pmvb44x[2][0] = (pmvb_d44x1[2]+pmvb_d44x2[2])/4;
		pmvb44y[2][1] = (pmvb_d44y1[2]+pmvb_d44y2[2])/4;
		}
		else if(currSlice->mymv_best[mbAddrX-22][0][2]==6 || currSlice->mymv_best[mbAddrX-22][0][1]==5){
		pmvb44x[2][0]=(currSlice->all_mymv[mbAddrX-22][0][4][0]+currSlice->all_mymv[mbAddrX-22][0][4][0]+
					   currSlice->all_mymv[mbAddrX-22][0][5][0]+currSlice->all_mymv[mbAddrX-22][-1][4][0])/4;
		pmvb44y[2][1]=(currSlice->all_mymv[mbAddrX-22][0][4][1]+currSlice->all_mymv[mbAddrX-22][0][4][1]+
					   currSlice->all_mymv[mbAddrX-22][0][5][1]+currSlice->all_mymv[mbAddrX-22][-1][4][1])/4;
		}
	if(currSlice->mymv_best[mbAddrX-22][3][3]==7){
		pmvb_b44x1[3][0] = (currSlice->all_mymv[mbAddrX-22][2][6][0]+currSlice->all_mymv[mbAddrX-22][2][7][0])/2;
		pmvb_b44y1[3][1] = (currSlice->all_mymv[mbAddrX-22][2][6][1]+currSlice->all_mymv[mbAddrX-22][2][7][1])/2;
		pmvb_b44x2[3][0] = (currSlice->all_mymv[mbAddrX-22][3][6][0]+currSlice->all_mymv[mbAddrX-22][3][7][0])/2;
		pmvb_b44y2[3][1] = (currSlice->all_mymv[mbAddrX-22][3][6][1]+currSlice->all_mymv[mbAddrX-22][3][7][1])/2;
		pmvb_c44x1[3][0] = (currSlice->all_mymv[mbAddrX-22][2][6][0]+currSlice->all_mymv[mbAddrX-22][3][6][0])/2;
		pmvb_c44y1[3][1] = (currSlice->all_mymv[mbAddrX-22][2][6][1]+currSlice->all_mymv[mbAddrX-22][3][6][1])/2;
		pmvb_c44x2[3][0] = (currSlice->all_mymv[mbAddrX-22][2][7][0]+currSlice->all_mymv[mbAddrX-22][3][7][0])/2;
		pmvb_c44y2[3][1] = (currSlice->all_mymv[mbAddrX-22][2][7][1]+currSlice->all_mymv[mbAddrX-22][3][7][1])/2;
		
		pmvb_d44x1[3] = (pmvb_b44x1[3][0]+pmvb_c44x1[3][0]);
		pmvb_d44y1[3] = (pmvb_b44y1[3][1]+pmvb_c44y1[3][1]);
		pmvb_d44x2[3] = (pmvb_b44x2[3][0]+pmvb_c44x2[3][0]);
		pmvb_d44y2[3] = (pmvb_b44y2[3][1]+pmvb_c44y2[3][1]);
		
		pmvb44x[3][0] = (pmvb_d44x1[3]+pmvb_d44x2[3])/4;
		pmvb44y[3][1] = (pmvb_d44y1[3]+pmvb_d44y2[3])/4;
		}
		else if(currSlice->mymv_best[mbAddrX-22][0][2]==6 || currSlice->mymv_best[mbAddrX-22][0][1]==5){
		pmvb44x[3][0]=(currSlice->all_mymv[mbAddrX-22][0][4][0]+currSlice->all_mymv[mbAddrX-22][0][4][0]+
					   currSlice->all_mymv[mbAddrX-22][0][5][0]+currSlice->all_mymv[mbAddrX-22][-1][4][0])/4;
		pmvb44y[3][1]=(currSlice->all_mymv[mbAddrX-22][0][4][1]+currSlice->all_mymv[mbAddrX-22][0][4][1]+
					   currSlice->all_mymv[mbAddrX-22][0][5][1]+currSlice->all_mymv[mbAddrX-22][-1][4][1])/4;
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
	
	
	 if(currSlice->mymv_best[mbAddrX-22][0][0]==4){
		pmvbbx[0]=(currSlice->all_mymv[mbAddrX-22][0][4][0]+currSlice->all_mymv[mbAddrX-22][2][4][0])/2;
		pmvbby[0]=(currSlice->all_mymv[mbAddrX-22][0][4][1]+currSlice->all_mymv[mbAddrX-22][2][4][1])/2;
		pmvbbx[1]=(currSlice->all_mymv[mbAddrX-22][0][2][0]+currSlice->all_mymv[mbAddrX-22][2][6][0])/2;
		pmvbby[1]=(currSlice->all_mymv[mbAddrX-22][0][2][1]+currSlice->all_mymv[mbAddrX-22][2][6][1])/2;
		
		
		pmvbcx[0]=(currSlice->all_mymv[mbAddrX-22][0][4][0]+currSlice->all_mymv[mbAddrX-22][0][2][0])/2;
		pmvbcy[0]=(currSlice->all_mymv[mbAddrX-22][0][4][1]+currSlice->all_mymv[mbAddrX-22][0][2][1])/2;
		pmvbcx[1]=(currSlice->all_mymv[mbAddrX-22][2][4][0]+currSlice->all_mymv[mbAddrX-22][2][6][0])/2;
		pmvbcy[1]=(currSlice->all_mymv[mbAddrX-22][2][4][1]+currSlice->all_mymv[mbAddrX-22][2][6][1])/2;
	  
		pmvbdx=(pmvbbx[0]+pmvbcx[0]+pmvbbx[1]+pmvbcx[1])/4;
		pmvbdy=(pmvbby[0]+pmvbcy[0]+pmvbby[1]+pmvbcy[1])/4;
	
	}
	 if(currSlice->mymv_best[mbAddrX-22][0][0]==2){
	
		pmvbdx=(currSlice->all_mymv[mbAddrX-22][0][2][0]+currSlice->all_mymv[mbAddrX-22][2][4][0]+
		currSlice->all_mymv[mbAddrX-22][0][4][0]+currSlice->all_mymv[mbAddrX-22][2][6][0])/4;
		pmvbdy=(currSlice->all_mymv[mbAddrX-22][0][2][1]+currSlice->all_mymv[mbAddrX-22][2][4][1]+
		currSlice->all_mymv[mbAddrX-22][0][4][1]+currSlice->all_mymv[mbAddrX-22][2][6][1])/4;
	
	}
	 if(currSlice->mymv_best[mbAddrX-22][0][0]==3){
	
		pmvbdx=(currSlice->all_mymv[mbAddrX-22][0][4][0]+currSlice->all_mymv[mbAddrX-22][2][6][0]+
		currSlice->all_mymv[mbAddrX-22][0][2][0]+currSlice->all_mymv[mbAddrX-22][2][4][0])/4;
		pmvbdy=(currSlice->all_mymv[mbAddrX-22][0][4][1]+currSlice->all_mymv[mbAddrX-22][2][6][1]+
		currSlice->all_mymv[mbAddrX-22][0][2][1]+currSlice->all_mymv[mbAddrX-22][2][4][1])/4;
	
	}
	
	pmvbdx=currSlice->all_mymv[mbAddrX-22][0][4][0];
	pmvbdy=currSlice->all_mymv[mbAddrX-22][0][4][1];
	
	}
	//end b
	if(block[2].available){
	    if(currSlice->mymv_best[mbAddrX-21][0][3]==7){
		pmvc_b44x1[0][0] = (currSlice->all_mymv[mbAddrX-21][0][4][0]+currSlice->all_mymv[mbAddrX-21][0][5][0])/2;
		pmvc_b44y1[0][1] = (currSlice->all_mymv[mbAddrX-21][0][4][1]+currSlice->all_mymv[mbAddrX-21][0][5][1])/2;
		pmvc_b44x2[0][0] = (currSlice->all_mymv[mbAddrX-21][1][4][0]+currSlice->all_mymv[mbAddrX-21][1][5][0])/2;
		pmvc_b44y2[0][1] = (currSlice->all_mymv[mbAddrX-21][1][4][1]+currSlice->all_mymv[mbAddrX-21][1][5][1])/2;
		pmvc_c44x1[0][0] = (currSlice->all_mymv[mbAddrX-21][0][4][0]+currSlice->all_mymv[mbAddrX-21][1][4][0])/2;
		pmvc_c44y1[0][1] = (currSlice->all_mymv[mbAddrX-21][0][4][1]+currSlice->all_mymv[mbAddrX-21][1][4][1])/2;
		pmvc_c44x2[0][0] = (currSlice->all_mymv[mbAddrX-21][0][5][0]+currSlice->all_mymv[mbAddrX-21][1][5][0])/2;
		pmvc_c44y2[0][1] = (currSlice->all_mymv[mbAddrX-21][0][5][1]+currSlice->all_mymv[mbAddrX-21][1][5][1])/2;	
		
		pmvc_d44x1[0] = (pmvc_b44x1[0][0]+pmvc_c44x1[0][0]);
		pmvc_d44y1[0] = (pmvc_b44y1[0][1]+pmvc_c44y1[0][1]);
		pmvc_d44x2[0] = (pmvc_b44x2[0][0]+pmvc_c44x2[0][0]);
		pmvc_d44y2[0] = (pmvc_b44y2[0][1]+pmvc_c44y2[0][1]);

		pmvc44x[0][0] = (pmvc_d44x1[0]+pmvc_d44x2[0])/4;
		pmvc44y[0][1] = (pmvc_d44y1[0]+pmvc_d44y2[0])/4;
		}
		else if(currSlice->mymv_best[mbAddrX-21][0][2]==6 || currSlice->mymv_best[mbAddrX-21][0][1]==5){
		pmvb44x[0][0]=(currSlice->all_mymv[mbAddrX-21][0][4][0]+currSlice->all_mymv[mbAddrX-21][0][4][0]+
					   currSlice->all_mymv[mbAddrX-21][0][5][0]+currSlice->all_mymv[mbAddrX-21][-1][4][0])/4;
		pmvb44y[0][1]=(currSlice->all_mymv[mbAddrX-21][0][4][1]+currSlice->all_mymv[mbAddrX-21][0][4][1]+
					   currSlice->all_mymv[mbAddrX-21][0][5][1]+currSlice->all_mymv[mbAddrX-21][-1][4][1])/4;
		}
		if(currSlice->mymv_best[mbAddrX-21][1][3]==7){
		pmvc_b44x1[1][0] = (currSlice->all_mymv[mbAddrX-21][0][2][0]+currSlice->all_mymv[mbAddrX-21][0][3][0])/2;
		pmvc_b44y1[1][1] = (currSlice->all_mymv[mbAddrX-21][0][2][1]+currSlice->all_mymv[mbAddrX-21][0][3][1])/2;
		pmvc_b44x2[1][0] = (currSlice->all_mymv[mbAddrX-21][1][2][0]+currSlice->all_mymv[mbAddrX-21][1][3][0])/2;
		pmvc_b44y2[1][1] = (currSlice->all_mymv[mbAddrX-21][1][2][1]+currSlice->all_mymv[mbAddrX-21][1][3][1])/2;
		pmvc_c44x1[1][0] = (currSlice->all_mymv[mbAddrX-21][0][2][0]+currSlice->all_mymv[mbAddrX-21][1][2][0])/2;
		pmvc_c44y1[1][1] = (currSlice->all_mymv[mbAddrX-21][0][2][1]+currSlice->all_mymv[mbAddrX-21][1][2][1])/2;
		pmvc_c44x2[1][0] = (currSlice->all_mymv[mbAddrX-21][0][3][0]+currSlice->all_mymv[mbAddrX-21][1][3][0])/2;
		pmvc_c44y2[1][1] = (currSlice->all_mymv[mbAddrX-21][0][3][1]+currSlice->all_mymv[mbAddrX-21][1][3][1])/2;
		
		pmvc_d44x1[1] = (pmvc_b44x1[1][0]+pmvc_c44x1[1][0]);
		pmvc_d44y1[1] = (pmvc_b44y1[1][1]+pmvc_c44y1[1][1]);
		pmvc_d44x2[1] = (pmvc_b44x2[1][0]+pmvc_c44x2[1][0]);
		pmvc_d44y2[1] = (pmvc_b44y2[1][1]+pmvc_c44y2[1][1]);

		pmvc44x[1][0] = (pmvc_d44x1[1]+pmvc_d44x2[1])/4;
		pmvc44y[1][1] = (pmvc_d44y1[1]+pmvc_d44y2[1])/4;
		}
		else if(currSlice->mymv_best[mbAddrX-21][0][2]==6 || currSlice->mymv_best[mbAddrX-21][0][1]==5){
		pmvb44x[1][0]=(currSlice->all_mymv[mbAddrX-21][0][4][0]+currSlice->all_mymv[mbAddrX-21][0][4][0]+
					   currSlice->all_mymv[mbAddrX-21][0][5][0]+currSlice->all_mymv[mbAddrX-21][-1][4][0])/4;
		pmvb44y[1][1]=(currSlice->all_mymv[mbAddrX-21][0][4][1]+currSlice->all_mymv[mbAddrX-21][0][4][1]+
					   currSlice->all_mymv[mbAddrX-21][0][5][1]+currSlice->all_mymv[mbAddrX-21][-1][4][1])/4;
		}
		if(currSlice->mymv_best[mbAddrX-21][2][3]==7){
		pmvc_b44x1[2][0] = (currSlice->all_mymv[mbAddrX-21][2][4][0]+currSlice->all_mymv[mbAddrX-21][2][5][0])/2;
		pmvc_b44y1[2][1] = (currSlice->all_mymv[mbAddrX-21][2][4][1]+currSlice->all_mymv[mbAddrX-21][2][5][1])/2;
		pmvc_b44x2[2][0] = (currSlice->all_mymv[mbAddrX-21][3][4][0]+currSlice->all_mymv[mbAddrX-21][3][5][0])/2;
		pmvc_b44y2[2][1] = (currSlice->all_mymv[mbAddrX-21][3][4][1]+currSlice->all_mymv[mbAddrX-21][3][5][1])/2;
		pmvc_c44x1[2][0] = (currSlice->all_mymv[mbAddrX-21][2][4][0]+currSlice->all_mymv[mbAddrX-21][3][4][0])/2;
		pmvc_c44y1[2][1] = (currSlice->all_mymv[mbAddrX-21][2][4][1]+currSlice->all_mymv[mbAddrX-21][3][4][1])/2;
		pmvc_c44x2[2][0] = (currSlice->all_mymv[mbAddrX-21][2][5][0]+currSlice->all_mymv[mbAddrX-21][3][5][0])/2;
		pmvc_c44y2[2][1] = (currSlice->all_mymv[mbAddrX-21][2][5][1]+currSlice->all_mymv[mbAddrX-21][3][5][1])/2;
		
		pmvc_d44x1[2] = (pmvc_b44x1[2][0]+pmvc_c44x1[2][0]);
		pmvc_d44y1[2] = (pmvc_b44y1[2][1]+pmvc_c44y1[2][1]);
		pmvc_d44x2[2] = (pmvc_b44x2[2][0]+pmvc_c44x2[2][0]);
		pmvc_d44y2[2] = (pmvc_b44y2[2][1]+pmvc_c44y2[2][1]);

		pmvc44x[2][0] = (pmvc_d44x1[2]+pmvc_d44x2[2])/4;
		pmvc44y[2][1] = (pmvc_d44y1[2]+pmvc_d44y2[2])/4;
		}
		else if(currSlice->mymv_best[mbAddrX-21][0][2]==6 || currSlice->mymv_best[mbAddrX-21][0][1]==5){
		pmvb44x[2][0]=(currSlice->all_mymv[mbAddrX-21][0][4][0]+currSlice->all_mymv[mbAddrX-21][0][4][0]+
					   currSlice->all_mymv[mbAddrX-21][0][5][0]+currSlice->all_mymv[mbAddrX-21][-1][4][0])/4;
		pmvb44y[2][1]=(currSlice->all_mymv[mbAddrX-21][0][4][1]+currSlice->all_mymv[mbAddrX-21][0][4][1]+
					   currSlice->all_mymv[mbAddrX-21][0][5][1]+currSlice->all_mymv[mbAddrX-21][-1][4][1])/4;
		}
		if(currSlice->mymv_best[mbAddrX-21][3][3]==7){
		pmvc_b44x1[3][0] = (currSlice->all_mymv[mbAddrX-21][2][6][0]+currSlice->all_mymv[mbAddrX-21][2][7][0])/2;
		pmvc_b44y1[3][1] = (currSlice->all_mymv[mbAddrX-21][2][6][1]+currSlice->all_mymv[mbAddrX-21][2][7][1])/2;
		pmvc_b44x2[3][0] = (currSlice->all_mymv[mbAddrX-21][3][6][0]+currSlice->all_mymv[mbAddrX-21][3][7][0])/2;
		pmvc_b44y2[3][1] = (currSlice->all_mymv[mbAddrX-21][3][6][1]+currSlice->all_mymv[mbAddrX-21][3][7][1])/2;
		pmvc_c44x1[3][0] = (currSlice->all_mymv[mbAddrX-21][2][6][0]+currSlice->all_mymv[mbAddrX-21][3][6][0])/2;
		pmvc_c44y1[3][1] = (currSlice->all_mymv[mbAddrX-21][2][6][1]+currSlice->all_mymv[mbAddrX-21][3][6][1])/2;
		pmvc_c44x2[3][0] = (currSlice->all_mymv[mbAddrX-21][2][7][0]+currSlice->all_mymv[mbAddrX-21][3][7][0])/2;
		pmvc_c44y2[3][1] = (currSlice->all_mymv[mbAddrX-21][2][7][1]+currSlice->all_mymv[mbAddrX-21][3][7][1])/2;
		
		pmvc_d44x1[3] = (pmvc_b44x1[3][0]+pmvc_c44x1[3][0]);
		pmvc_d44y1[3] = (pmvc_b44y1[3][1]+pmvc_c44y1[3][1]);
		pmvc_d44x2[3] = (pmvc_b44x2[3][0]+pmvc_c44x2[3][0]);
		pmvc_d44y2[3] = (pmvc_b44y2[3][1]+pmvc_c44y2[3][1]);

		pmvc44x[3][0] = (pmvc_d44x1[3]+pmvc_d44x2[3])/4;
		pmvc44y[3][1] = (pmvc_d44y1[3]+pmvc_d44y2[3])/4;
		}
		else if(currSlice->mymv_best[mbAddrX-21][0][2]==6 || currSlice->mymv_best[mbAddrX-21][0][1]==5){
		pmvb44x[3][0]=(currSlice->all_mymv[mbAddrX-21][0][4][0]+currSlice->all_mymv[mbAddrX-21][0][4][0]+
					   currSlice->all_mymv[mbAddrX-21][0][5][0]+currSlice->all_mymv[mbAddrX-21][-1][4][0])/4;
		pmvb44y[3][1]=(currSlice->all_mymv[mbAddrX-21][0][4][1]+currSlice->all_mymv[mbAddrX-21][0][4][1]+
					   currSlice->all_mymv[mbAddrX-21][0][5][1]+currSlice->all_mymv[mbAddrX-21][-1][4][1])/4;
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
	
	if(currSlice->mymv_best[mbAddrX-21][0][0]==4){
		pmvcbx[0]=(currSlice->all_mymv[mbAddrX-21][0][4][0]+currSlice->all_mymv[mbAddrX-21][2][4][0])/2;
		pmvcby[0]=(currSlice->all_mymv[mbAddrX-21][0][4][1]+currSlice->all_mymv[mbAddrX-21][2][4][1])/2;
		pmvcbx[1]=(currSlice->all_mymv[mbAddrX-21][0][2][0]+currSlice->all_mymv[mbAddrX-21][2][6][0])/2;
		pmvcby[1]=(currSlice->all_mymv[mbAddrX-21][0][2][1]+currSlice->all_mymv[mbAddrX-21][2][6][1])/2;
		
		
		pmvccx[0]=(currSlice->all_mymv[mbAddrX-21][0][4][0]+currSlice->all_mymv[mbAddrX-21][0][2][0])/2;
		pmvccy[0]=(currSlice->all_mymv[mbAddrX-21][0][4][1]+currSlice->all_mymv[mbAddrX-21][0][2][1])/2;
		pmvccx[1]=(currSlice->all_mymv[mbAddrX-21][2][4][0]+currSlice->all_mymv[mbAddrX-21][2][6][0])/2;
		pmvccy[1]=(currSlice->all_mymv[mbAddrX-21][2][4][1]+currSlice->all_mymv[mbAddrX-21][2][6][1])/2;
	  
		pmvcdx=(pmvcbx[0]+pmvccx[0]+pmvcbx[1]+pmvccx[1])/4;
		pmvcdy=(pmvcby[0]+pmvccy[0]+pmvcby[1]+pmvccy[1])/4;
	
	}
	if(currSlice->mymv_best[mbAddrX-21][0][0]==2){
	
		pmvcdx=(currSlice->all_mymv[mbAddrX-21][0][2][0]+currSlice->all_mymv[mbAddrX-21][2][4][0]+
				currSlice->all_mymv[mbAddrX-21][0][4][0]+currSlice->all_mymv[mbAddrX-21][2][6][0])/4;
		
		pmvcdy=(currSlice->all_mymv[mbAddrX-21][0][2][1]+currSlice->all_mymv[mbAddrX-21][2][4][1]+
				currSlice->all_mymv[mbAddrX-21][0][4][1]+currSlice->all_mymv[mbAddrX-21][2][6][1])/4;
	
	}
	if(currSlice->mymv_best[mbAddrX-21][0][0]==3){
	
		pmvcdx=(currSlice->all_mymv[mbAddrX-21][0][4][0]+currSlice->all_mymv[mbAddrX-21][2][6][0]+
				currSlice->all_mymv[mbAddrX-21][0][2][0]+currSlice->all_mymv[mbAddrX-21][2][4][0])/4;
		
		pmvcdy=(currSlice->all_mymv[mbAddrX-21][0][4][1]+currSlice->all_mymv[mbAddrX-21][2][6][1]+
				currSlice->all_mymv[mbAddrX-21][0][2][1]+currSlice->all_mymv[mbAddrX-21][2][4][1])/4;
	
	}
	
	pmvcdx=currSlice->all_mymv[mbAddrX-21][0][4][0];
	pmvcdy=currSlice->all_mymv[mbAddrX-21][0][4][1];
	
}
	
	if(currMB->mb_x==0){
		if(block[1].available)  L=abs(pmvbdx)+abs(pmvbdy); 
	//	pmv[0] = (short) pmvbdx;
	//	pmv[1] = (short) pmvbdy;
		} //boundary checked
	else if(currMB->mb_y==0) {
		if(block[0].available)	L=abs(pmvadx)+abs(pmvady); 
	//	pmv[0] = (short) pmvadx;
	//	pmv[1] = (short) pmvady;
		}
	else if(currMB->mb_x==21) {
		if(block[0].available)	L=abs(pmvadx)+abs(pmvady); 
		//pmv[0] = (short) pmvadx;
		//pmv[1] = (short) pmvady;
		}
		
	else
	{
	if(block[0].available)	mva = abs(pmvadx)+abs(pmvady);
	if(block[1].available)	mvb = abs(pmvbdx)+abs(pmvbdy); 
	if(block[2].available)	mvc = abs(pmvcdx)+abs(pmvcdy); 
		
	if(mva>=mvb && mva>=mvc) 
	{
		L=mva;
		//pmv[0] = (short) pmvadx;
		//pmv[1] = (short) pmvady;
		}
	if(mvb>=mvc && mvb>=mva) 
	{
		L=mvb;
		//pmv[0] = (short) pmvbdx;
		//pmv[1] = (short) pmvbdy;
		}
	if(mvc>=mva && mvc>=mvb)
	{
		L=mvc;
		//pmv[0] = (short) pmvcdx;
		//pmv[1] = (short) pmvcdy;
	}
	
	}
	//printf("pmvadx=%d pmvady=%d\n",pmvadx,pmvady);
	  // 	printf("pmvbdx=%d pmvbdy=%d\n",pmvbdx,pmvbdy);
	//	printf("pmvcdx=%d pmvcdy=%d\n",pmvcdx,pmvcdy);
	//currMB->pmva =mva;
	//currMB->pmvb =mvb;
	//currMB->pmvc =mvc;
	currMB->L[mbAddrX] =L;
	//if(currMB->L[mbAddrX] ==2)//system("pause");
	//printf ("currMB->L[mbAddr]=%d=( %d )\n",mbAddrX,currMB->L[mbAddrX]);
}


}
*/

void InitMotionVectorPrediction(Macroblock *currMB, int MbaffFrameFlag)
{  
	
  if (MbaffFrameFlag)
    currMB->GetMVPredictor = GetMotionVectorPredictorMBAFF;
	
  else
    currMB->GetMVPredictor = GetMotionVectorPredictorNormal;
	//currMB->GetMVPredictorL = GetMotionVectorPredictorL;
  
}
