/*!
 ***************************************************************************
 * \file md_high.c
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
#include "intrarefresh.h"
#include "image.h"
#include "ratectl.h"
#include "mode_decision.h"
#include "fmo.h"
#include "me_umhex.h"
#include "me_umhexsmp.h"
#include "macroblock.h"
#include "md_common.h"
#include "conformance.h"
#include "vlc.h"
#include "rdopt.h"
#include "mv_search.h"

/*!
*************************************************************************************
* \brief
*    Mode Decision for a macroblock
*************************************************************************************
*/
void encode_one_macroblock_high (Macroblock *currMB)
{
  Slice *currSlice = currMB->p_slice;
  ImageParameters *p_Img = currMB->p_Img;
  InputParameters *p_Inp = currMB->p_Inp;
  PicMotionParams *motion = &p_Img->enc_picture->motion;
  RDOPTStructure  *p_RDO = currSlice->p_RDO;

  int         max_index = 9;
  int         block, index, mode, i, j;
  RD_PARAMS   enc_mb;
  int         bmcost[5] = {INT_MAX};
  int         cost=0;
  int         min_cost = INT_MAX;
  int         intra1 = 0;
  int         mb_available[3];

  short       bslice      = (short) (currSlice->slice_type == B_SLICE);
  short       pslice      = (short) ((currSlice->slice_type == P_SLICE) || (currSlice->slice_type == SP_SLICE));
  short       intra       = (short) ((currSlice->slice_type == I_SLICE) || (pslice && currMB->mb_y == p_Img->mb_y_upd && p_Img->mb_y_upd != p_Img->mb_y_intra));
  int         lambda_mf[3];
 
  imgpel    **mb_pred  = currSlice->mb_pred[0];
  Block8x8Info *b8x8info = p_Img->b8x8info;

  char        chroma_pred_mode_range[2];
  short       inter_skip = 0;
  BestMode    md_best;
  Info8x8     best;
  //printf ("currMBNum=%d\n",currMB->mbAddrX);
    currMB->Tlow_flag =0;
	currMB->Thigh_flag =0;
	currMB->mode1_flag = 0;
	currMB->mode2_flag = 0;
	currMB->mode3_flag = 0;
	currMB->mode4_flag = 0;
  //printf ("currMBQP=%d\n",currMB->qp);
	
 // system("pause");
  init_md_best(&md_best);

  // Init best (need to create simple function)
  best.pdir = 0;
  best.bipred = 0;
  best.ref[LIST_0] = 0;
  best.ref[LIST_1] = -1;

  intra |= RandomIntra (p_Img, currMB->mbAddrX);    // Forced Pseudo-Random Intra

  //===== Setup Macroblock encoding parameters =====
  init_enc_mb_params(currMB, &enc_mb, intra);
  if (p_Inp->AdaptiveRounding)
  {
    reset_adaptive_rounding(p_Img);
  }

  if (currSlice->MbaffFrameFlag)
  {
    reset_mb_nz_coeff(p_Img, currMB->mbAddrX);
  }

  //=====   S T O R E   C O D I N G   S T A T E   =====
  //---------------------------------------------------
  currSlice->store_coding_state (currMB, currSlice->p_RDO->cs_cm);

  if (!intra)
  {
    //===== set skip/direct motion vectors =====
    if (enc_mb.valid[0])
    {
      if (bslice)
        currSlice->Get_Direct_Motion_Vectors (currMB);
      else 
        FindSkipModeMotionVector (currMB);
    }
    if (p_Inp->CtxAdptLagrangeMult == 1)
    {
      get_initial_mb16x16_cost(currMB);
    }

    //===== MOTION ESTIMATION FOR 16x16, 16x8, 8x16 BLOCKS =====
    for (mode = 1; mode < 4; mode++)
    {
      best.mode = (char) mode;
      best.bipred = 0;
      b8x8info->best[mode][0].bipred = 0;
	 // if(currMB->mbAddrX==393){
	 // system("pause");
	  //printf("mode=%d\n",mode);}
      if (enc_mb.valid[mode])
      {
        for (cost=0, block=0; block<(mode==1?1:2); block++)
        {
          update_lambda_costs(currMB, &enc_mb, lambda_mf);
          PartitionMotionSearch (currMB, mode, block, lambda_mf);

          //--- set 4x4 block indices (for getting MV) ---
          j = (block==1 && mode==2 ? 2 : 0);
          i = (block==1 && mode==3 ? 2 : 0);

          //--- get cost and reference frame for List 0 prediction ---
          bmcost[LIST_0] = INT_MAX;
          list_prediction_cost(currMB, LIST_0, block, mode, &enc_mb, bmcost, best.ref);

          if (bslice)
          {
            //--- get cost and reference frame for List 1 prediction ---
            bmcost[LIST_1] = INT_MAX;
            list_prediction_cost(currMB, LIST_1, block, mode, &enc_mb, bmcost, best.ref);

            // Compute bipredictive cost between best list 0 and best list 1 references
            list_prediction_cost(currMB, BI_PRED, block, mode, &enc_mb, bmcost, best.ref);

            // currently Bi predictive ME is only supported for modes 1, 2, 3 and ref 0
            if (is_bipred_enabled(p_Inp, mode))
            {
              get_bipred_cost(currMB, mode, block, i, j, &best, &enc_mb, bmcost);
            }
            else
            {
              bmcost[BI_PRED_L0] = INT_MAX;
              bmcost[BI_PRED_L1] = INT_MAX;
            }

            // Determine prediction list based on mode cost
            determine_prediction_list(bmcost, &best, &cost);
          }
          else // if (bslice)
          {
            best.pdir = 0;
            cost      += bmcost[LIST_0];
          }

          assign_enc_picture_params(currMB, mode, &best, 2 * block);

          //----- set reference frame and direction parameters -----
          set_block8x8_info(b8x8info, mode, block, &best);

          //--- set reference frames and motion vectors ---
          if (mode>1 && block == 0)
            currSlice->set_ref_and_motion_vectors (currMB, motion, &best, block);
        } // for (block=0; block<(mode==1?1:2); block++)

        if (cost < min_cost)
        {
          md_best.mode = (byte) mode;
          md_best.cost = cost;
          currMB->best_mode = (short) mode;
		  //printf("===best_frame=%d===\n",mode);
		  //system("pause");
          min_cost  = cost;
          if (p_Inp->CtxAdptLagrangeMult == 1)
          {
            adjust_mb16x16_cost(currMB, cost);
          }
        }
      } // if (enc_mb.valid[mode])
    } // for (mode=1; mode<4; mode++)

    if (enc_mb.valid[P8x8])
    {      
      currMB->valid_8x8 = FALSE;

      if (p_Inp->Transform8x8Mode)
      {
        ResetRD8x8Data(p_Img, p_RDO->tr8x8);
        currMB->luma_transform_size_8x8_flag = TRUE; //switch to 8x8 transform size
        //===========================================================
        // Check 8x8 partition with transform size 8x8
        //===========================================================
        //=====  LOOP OVER 8x8 SUB-PARTITIONS  (Motion Estimation & Mode Decision) =====
        for (block = 0; block < 4; block++)
        {
          submacroblock_mode_decision(currMB, &enc_mb, p_RDO->tr8x8, p_RDO->cofAC8x8ts[block], block, &cost);

          set_subblock8x8_info(b8x8info, P8x8, block, p_RDO->tr8x8);
        }        
      }// if (p_Inp->Transform8x8Mode)


      if (p_Inp->Transform8x8Mode != 2)
      {
        currMB->luma_transform_size_8x8_flag = FALSE; //switch to 8x8 transform size
        ResetRD8x8Data(p_Img, p_RDO->tr4x4);
        //=================================================================
        // Check 8x8, 8x4, 4x8 and 4x4 partitions with transform size 4x4
        //=================================================================
        //=====  LOOP OVER 8x8 SUB-PARTITIONS  (Motion Estimation & Mode Decision) =====
        for (block = 0; block < 4; block++)
        {
          submacroblock_mode_decision(currMB, &enc_mb, p_RDO->tr4x4, p_RDO->coefAC8x8[block], block, &cost);

           set_subblock8x8_info(b8x8info, P8x8, block, p_RDO->tr4x4);
        }
      }// if (p_Inp->Transform8x8Mode != 2)

      if (p_Inp->RCEnable)
        rc_store_diff(currSlice->diffy, &p_Img->pCurImg[currMB->opix_y], currMB->pix_x, mb_pred);

      p_Img->giRDOpt_B8OnlyFlag = FALSE;
    }
  }
  else // if (!intra)
  {
    min_cost = INT_MAX;
  }

  // Set Chroma mode
  SetChromaPredMode(currMB, enc_mb, mb_available, chroma_pred_mode_range);

  //========= C H O O S E   B E S T   M A C R O B L O C K   M O D E =========
  //-------------------------------------------------------------------------

  for (currMB->c_ipred_mode = chroma_pred_mode_range[0]; currMB->c_ipred_mode<=chroma_pred_mode_range[1]; currMB->c_ipred_mode++)
  {
    // bypass if c_ipred_mode is not allowed
    if ( (p_Img->yuv_format != YUV400) &&
      (  ((!intra || !p_Inp->IntraDisableInterOnly) && p_Inp->ChromaIntraDisable == 1 && currMB->c_ipred_mode!=DC_PRED_8) 
      || (currMB->c_ipred_mode == VERT_PRED_8 && !mb_available[0]) 
      || (currMB->c_ipred_mode == HOR_PRED_8  && !mb_available[1]) 
      || (currMB->c_ipred_mode == PLANE_8     && (!mb_available[1] || !mb_available[0] || !mb_available[2]))))
      continue;        

    //===== GET BEST MACROBLOCK MODE =====
    for (index=0; index < max_index; index++)
    {
      mode = mb_mode_table[index];
      if (enc_mb.valid[mode])
      {
        if (p_Img->yuv_format != YUV400)
        {           
          currMB->i16mode = 0; 
        }

        // Skip intra modes in inter slices if best mode is inter <P8x8 with cbp equal to 0    
        if (currSlice->P444_joined)
        {
          if (p_Inp->SkipIntraInInterSlices && !intra && mode >= I16MB 
            && currMB->best_mode <=3 && currMB->best_cbp == 0 && currSlice->cmp_cbp[1] == 0 && currSlice->cmp_cbp[2] == 0 && (currMB->min_rdcost < enc_mb.lambda_md * 5.0))
            continue;
        }
        else
        {
          if (p_Inp->SkipIntraInInterSlices)
          {
            if (!intra && mode >= I4MB)
            {
              if (currMB->best_mode <=3 && currMB->best_cbp == 0 && (currMB->min_rdcost < enc_mb.lambda_md * 5.0))
              {
                continue;
              }
              else if (currMB->best_mode == 0 && (currMB->min_rdcost < enc_mb.lambda_md * 6.0))
              {
                continue;
              }
            }
          }

        }

        compute_mode_RD_cost(currMB, &enc_mb, (short) mode, &inter_skip);
        if(currMB->Tlow_flag) break;
		else if(currMB->Thigh_flag  && index==6) break;
		else if(currMB->Thigh_flag) index=4;
		else if(currMB->mode1_flag && index==1) break;
		else if(currMB->mode2_flag && currMB->mode3_flag)index=1;
		else if(currMB->mode2_flag && currMB->mode3_flag && index==3) break;
		else if(currMB->mode4_flag && index==4) break;
		else if(currMB->mode4_flag) index=3;
		//currMB->Tlow_flag  =0;
		//currMB->Thigh_flag = 0;
		//currMB->mode1_flag = 0;
		//currMB->mode2_flag = 0;
		//currMB->mode3_flag = 0;
		//currMB->mode4_flag = 0;
		
      }
    }// for (index=0; index<max_index; index++)
  }// for (currMB->c_ipred_mode=DC_PRED_8; currMB->c_ipred_mode<=chroma_pred_mode_range[1]; currMB->c_ipred_mode++)                     

  restore_nz_coeff(currMB);

  intra1 = IS_INTRA(currMB);

  //=====  S E T   F I N A L   M A C R O B L O C K   P A R A M E T E R S ======
  //---------------------------------------------------------------------------
  update_qp_cbp_tmp(currMB, p_RDO->cbp);
  currSlice->set_stored_mb_parameters (currMB);

  // Rate control
  if(p_Inp->RCEnable && p_Inp->RCUpdateMode <= MAX_RC_MODE)
    rc_store_mad(currMB);


  //===== Decide if this MB will restrict the reference frames =====
  if (p_Inp->RestrictRef)
    update_refresh_map(currMB, intra, intra1);
	
}

