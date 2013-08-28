/*!
 *************************************************************************************
 * \file macroblock_P444.c
 *
 * \brief
 *    Process one macroblock functions (P444 mode)
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Alexis Michael Tourapis         <alexismt@ieee.org>
 *************************************************************************************
 */

#include "contributors.h"
#include "global.h"
#include "mc_prediction.h"
#include "md_common.h"
#include "rdopt.h"
#include "macroblock.h"


/*!
 ************************************************************************
 * \brief
 *    Residual Coding of an 8x8 Luma block (not for intra)
 *
 * \return
 *    coefficient cost
 ************************************************************************
 */
int luma_residual_coding_p444_16x16 (Macroblock* currMB,  //!< Current Macroblock to be coded
                                     int   block8x8,     //!< block number of 8x8 block
                                     short p_dir,        //!< prediction direction
                                     int   list_mode[2], //!< list0 prediction mode (1-7, 0=DIRECT)
                                     char  *ref_idx)     //!< reference pictures for each list
{

  int   *cbp     = &(currMB->cbp);
  int64 *cbp_blk = &(currMB->cbp_blk);
  Slice *currSlice = currMB->p_slice;
  ImageParameters *p_Img = currMB->p_Img;  
  int    i, j, nonzero = 0, cbp_blk_mask;
  int    coeff_cost = 0;
  int    mb_y       = (block8x8 >> 1) << 3;
  int    mb_x       = (block8x8 & 0x01) << 3;
  int    cbp_mask   = 1 << block8x8;

  int    skipped    = (list_mode[0] == 0 && list_mode[1] == 0 && (currSlice->slice_type != B_SLICE));
  ColorPlane uv;

  char   bipred_me = currMB->b8x8[block8x8].bipred;
  currSlice->coeff_cost_cr[1] = currSlice->coeff_cost_cr[2] = 0;

  //memset( currSlice->cofAC[block8x8][0][0], 0, 4 * 2 * 65 * sizeof(int));

  //===== luma prediction ======
  luma_prediction (currMB, mb_x, mb_y, 8, 8, p_dir, list_mode, ref_idx, bipred_me); 

  //===== compute prediction residual ======            
  compute_residue (&p_Img->pCurImg[currMB->opix_y + mb_y], &currSlice->mb_pred[0][mb_y], &currSlice->mb_ores[0][mb_y], mb_x, currMB->pix_x + mb_x, 8, 8);

  for (uv = PLANE_U; uv <= PLANE_V; ++uv)
  {
    select_plane(p_Img, uv);
    chroma_prediction (currMB, uv - 1, mb_x, mb_y, 8, 8, p_dir, list_mode[0], list_mode[1], ref_idx[0], ref_idx[1], bipred_me);

    //===== compute prediction residual ======            
    compute_residue(&p_Img->pImgOrg[uv][currMB->opix_y + mb_y], &currSlice->mb_pred[uv][mb_y], &currSlice->mb_ores[uv][mb_y], mb_x, currMB->pix_x + mb_x, 8, 8);
  }
  select_plane(p_Img, PLANE_Y);

  //===== loop over 4x4 blocks =====
  if(!currMB->luma_transform_size_8x8_flag)
  {
    //===== DCT, Quantization, inverse Quantization, IDCT, Reconstruction =====
    if (!skipped && ( (currSlice->NoResidueDirect != 1) || (currMB->qp_scaled[0] == 0 && p_Img->lossless_qpprime_flag == 1) ))
    {
      int block_y, block_x;

      for (block_y=mb_y; block_y<mb_y + 8; block_y += 4)
      {
        for (block_x=mb_x; block_x<mb_x + 8; block_x += 4)
        {          
          //===== DCT, Quantization, inverse Quantization, IDCT, Reconstruction =====
          nonzero = currMB->trans_4x4 (currMB, PLANE_Y, block_x, block_y, &coeff_cost, 0);

          if (nonzero)
          {
            cbp_blk_mask = (block_x >> 2) + block_y;      
            (*cbp_blk) |= (int64) 1 << cbp_blk_mask;  // one bit for every 4x4 block
            (*cbp)     |= cbp_mask;           // one bit for the 4x4 blocks of an 8x8 block
          }

          if (currSlice->slice_type != SP_SLICE)  
          {
            for (uv = PLANE_U; uv <= PLANE_V; ++uv)
            {
              select_plane(p_Img, (ColorPlane) uv);
              nonzero = currMB->trans_4x4( currMB, (ColorPlane) uv, block_x, block_y, &currSlice->coeff_cost_cr[uv], 0);
              if (nonzero)
              {
                cbp_blk_mask = (block_x >> 2) + block_y;
                (currSlice->cur_cbp_blk[uv]) |= (int64) 1 << cbp_blk_mask;  // one bit for every 4x4 block
                (currSlice->cmp_cbp[uv]) |= cbp_mask;           // one bit for the 4x4 blocks of an 8x8 block
              }
            }
            select_plane(p_Img, PLANE_Y);
          }
          else
          {
            assert(currSlice->slice_type == SP_SLICE);   //SP_SLICE not implemented for FREXT_AD444
          }
        }
      }
    }
  }
  else
  {
    if (currSlice->NoResidueDirect != 1 && !skipped)
    {
      if (currSlice->slice_type != SP_SLICE)
        nonzero = currMB->trans_8x8 (currMB, PLANE_Y, block8x8, &coeff_cost, 0);

      if (nonzero)
      {
        (*cbp_blk) |= 51 << (4*block8x8 - 2*(block8x8 & 0x01)); // corresponds to 110011, as if all four 4x4 blocks contain coeff, shifted to block position
        (*cbp)     |= cbp_mask;                               // one bit for the 4x4 blocks of an 8x8 block
      }

      if (currSlice->slice_type != SP_SLICE)
      {
        for (uv = PLANE_U; uv <= PLANE_V; ++uv)
        {
          select_plane(p_Img, (ColorPlane) uv);
          nonzero = currMB->trans_8x8( currMB, (ColorPlane) uv, block8x8, &currSlice->coeff_cost_cr[uv], 0);
          if (nonzero)
          {
            (currSlice->cur_cbp_blk[uv]) |= 51 << (4*block8x8-2*(block8x8 & 0x01)); // corresponds to 110011, as if all four 4x4 blocks contain coeff, shifted to block position
            (currSlice->cmp_cbp[uv])     |= cbp_mask;           // one bit for the 4x4 blocks of an 8x8 block
          }
        }
        select_plane(p_Img, PLANE_Y);
      }
    }
  }

  /*
  The purpose of the action below is to prevent that single or 'expensive' coefficients are coded.
  With 4x4 transform there is larger chance that a single coefficient in a 8x8 or 16x16 block may be nonzero.
  A single small (level=1) coefficient in a 8x8 block will cost: 3 or more bits for the coefficient,
  4 bits for EOBs for the 4x4 blocks,possibly also more bits for CBP.  Hence the total 'cost' of that single
  coefficient will typically be 10-12 bits which in a RD consideration is too much to justify the distortion improvement.
  The action below is to watch such 'single' coefficients and set the reconstructed block equal to the prediction according
  to a given criterium.  The action is taken only for inter luma blocks.

  Notice that this is a pure encoder issue and hence does not have any implication on the standard.
  coeff_cost is a parameter set in dct_4x4() and accumulated for each 8x8 block.  If level=1 for a coefficient,
  coeff_cost is increased by a number depending on RUN for that coefficient.The numbers are (see also dct_4x4()): 3,2,2,1,1,1,0,0,...
  when RUN equals 0,1,2,3,4,5,6, etc.
  If level >1 coeff_cost is increased by 9 (or any number above 3). The threshold is set to 3. This means for example:
  1: If there is one coefficient with (RUN,level)=(0,1) in a 8x8 block this coefficient is discarded.
  2: If there are two coefficients with (RUN,level)=(1,1) and (4,1) the coefficients are also discarded
  sum_cnt_nonz[0] is the accumulation of coeff_cost over a whole macro block.  If sum_cnt_nonz[0] is 5 or less for the whole MB,
  all nonzero coefficients are discarded for the MB and the reconstructed block is set equal to the prediction.
  */

  if (currSlice->NoResidueDirect != 1 && !skipped && coeff_cost <= _LUMA_COEFF_COST_ &&
    ((currMB->qp_scaled[0])!=0 || p_Img->lossless_qpprime_flag==0)&&
    !(currSlice->slice_type == SP_SLICE && (p_Img->si_frame_indicator == TRUE || p_Img->sp2_frame_indicator == TRUE )))// last set of conditions
    // cannot skip when perfect reconstruction is as in switching pictures or SI pictures
  {
    coeff_cost  = 0;
    (*cbp)     &=  (63 - cbp_mask);
    (*cbp_blk) &= ~(51 << (4 * block8x8 - 2 * (block8x8 & 0x01)));

    memset( currSlice->cofAC[block8x8][0][0], 0, 4 * 2 * 65 * sizeof(int));

    copy_image_data_8x8(&p_Img->enc_picture->imgY[currMB->pix_y + mb_y], &currSlice->mb_pred[0][mb_y], currMB->pix_x + mb_x, mb_x);

    if (currSlice->slice_type == SP_SLICE)
    {
      for (i = mb_x; i < mb_x + BLOCK_SIZE_8x8; i += BLOCK_SIZE)
        for (j = mb_y; j < mb_y + BLOCK_SIZE_8x8; j += BLOCK_SIZE)
          copyblock_sp(currMB, PLANE_Y, i, j);
    }
  }

  for (uv = PLANE_U; uv <= PLANE_V; ++uv)
  {
    if (currSlice->NoResidueDirect != 1 && !skipped && currSlice->coeff_cost_cr[uv] <= _LUMA_COEFF_COST_ &&
      (currMB->qp_scaled[uv]!=0 || p_Img->lossless_qpprime_flag==0))// last set of conditions
    {
      currSlice->coeff_cost_cr[uv] = 0;
      currSlice->cmp_cbp[uv] &= (63 - cbp_mask);
      currSlice->cur_cbp_blk[uv] &= ~(51 << (4*block8x8-2*(block8x8 & 0x01)));

      memset( currSlice->cofAC[block8x8 + 4 * uv][0][0], 0, 4 * 2 * 65 * sizeof(int));

      copy_image_data_8x8(&p_Img->enc_picture->imgUV[uv - 1][currMB->pix_y + mb_y], &currSlice->mb_pred[uv][mb_y], currMB->pix_x + mb_x, mb_x);
    }
  }

  return coeff_cost;
}


/*!
 ************************************************************************
 * \brief
 *    Residual Coding of an 8x8 Luma block (not for intra)
 *
 * \return
 *    coefficient cost
 ************************************************************************
 */
int luma_residual_coding_p444_8x8 (Macroblock* currMB,  //!< Current Macroblock to be coded
                                   int   *cbp,         //!< Output: cbp (updated according to processed 8x8 luminance block)
                                   int64 *cbp_blk,     //!< Output: block cbp (updated according to processed 8x8 luminance block)
                                   int   block8x8,     //!< block number of 8x8 block
                                   short p_dir,        //!< prediction direction
                                   int   list_mode[2], //!< list prediction mode (1-7, 0=DIRECT)
                                   char  *ref_idx)     //!< reference pictures for each list
{
  Slice *currSlice = currMB->p_slice;
  ImageParameters *p_Img = currSlice->p_Img;  
  int    block_y, block_x, pic_pix_x, i, j, nonzero = 0, cbp_blk_mask;
  int    coeff_cost = 0;
  int    mb_y       = (block8x8 >> 1) << 3;
  int    mb_x       = (block8x8 & 0x01) << 3;
  int    cbp_mask   = 1 << block8x8;
  int    bxx, byy;                   // indexing curr_blk
  int    skipped    = (list_mode[0] == 0 && list_mode[1] == 0 && (currSlice->slice_type != B_SLICE));
  ColorPlane uv;

  char   bipred_me = currMB->b8x8[block8x8].bipred;
  currSlice->coeff_cost_cr[1] = currSlice->coeff_cost_cr[2] = 0;

  //memset( currSlice->cofAC[block8x8][0][0], 0, 4 * 2 * 65 * sizeof(int));

  //===== loop over 4x4 blocks =====
  if(!currMB->luma_transform_size_8x8_flag)
  {
    if (((p_dir == 0 || p_dir == 2 )&& list_mode[0] < 5) || ((p_dir == 1 || p_dir == 2 ) && list_mode[1] < 5))
    {
      luma_prediction (currMB, mb_x, mb_y, 8, 8, p_dir, list_mode, ref_idx, bipred_me); 

      //===== compute prediction residual ======            
      compute_residue (&p_Img->pCurImg[currMB->opix_y + mb_y], &currSlice->mb_pred[0][mb_y], &currSlice->mb_ores[0][mb_y], mb_x, currMB->pix_x + mb_x, 8, 8);
    }

    for (byy=0, block_y=mb_y; block_y<mb_y+8; byy+=4, block_y+=4)
    {
      for (bxx=0, block_x=mb_x; block_x<mb_x+8; bxx+=4, block_x+=4)
      {
        pic_pix_x = currMB->pix_x + block_x;
        cbp_blk_mask = (block_x >> 2) + block_y;

        //===== prediction of 4x4 block =====
        if (!(((p_dir == 0 || p_dir == 2 )&& list_mode[0] < 5) || ((p_dir == 1 || p_dir == 2 ) && list_mode[1] < 5)))
        {
          luma_prediction (currMB, block_x, block_y, 4, 4, p_dir, list_mode, ref_idx, bipred_me);

          //===== compute prediction residual ======            
          compute_residue(&p_Img->pCurImg[currMB->opix_y + block_y], &currSlice->mb_pred[0][block_y], &currSlice->mb_ores[0][block_y], block_x, pic_pix_x, 4, 4);
        }

        for (uv = PLANE_U; uv <= PLANE_V; ++uv)
        {
          select_plane(p_Img, uv);
          chroma_prediction (currMB, uv - 1, block_x, block_y, 4, 4, p_dir, list_mode[0], list_mode[1], ref_idx[0], ref_idx[1], bipred_me);

          //===== compute prediction residual ======            
          compute_residue(&p_Img->pImgOrg[uv][currMB->opix_y + block_y], &currSlice->mb_pred[uv][block_y], &currSlice->mb_ores[uv][block_y], block_x, pic_pix_x, 4, 4);
        }
        select_plane(p_Img, PLANE_Y);

        //===== DCT, Quantization, inverse Quantization, IDCT, Reconstruction =====
        if (!skipped && ( (currSlice->NoResidueDirect != 1) || (currMB->qp_scaled[0] == 0 && p_Img->lossless_qpprime_flag == 1) ))
        {
          //===== DCT, Quantization, inverse Quantization, IDCT, Reconstruction =====
          nonzero = currMB->trans_4x4 (currMB, PLANE_Y, block_x, block_y, &coeff_cost, 0);

          if (nonzero)
          {
            (*cbp_blk) |= (int64)1 << cbp_blk_mask;  // one bit for every 4x4 block
            (*cbp)     |= cbp_mask;           // one bit for the 4x4 blocks of an 8x8 block
          }

          if (currSlice->slice_type != SP_SLICE)  
          {
            for (uv = PLANE_U; uv <= PLANE_V; ++uv)
            {
              select_plane(p_Img, (ColorPlane) uv);
              nonzero = currMB->trans_4x4( currMB, (ColorPlane) uv, block_x, block_y, &currSlice->coeff_cost_cr[uv], 0);
              if (nonzero)
              {
                (currSlice->cur_cbp_blk[uv]) |= (int64) 1 << cbp_blk_mask;  // one bit for every 4x4 block
                (currSlice->cmp_cbp[uv]) |= cbp_mask;           // one bit for the 4x4 blocks of an 8x8 block
              }
            }
            select_plane(p_Img, PLANE_Y);
          }
          else
          {
            assert(currSlice->slice_type == SP_SLICE);   //SP_SLICE not implementd for FREXT_AD444
          }
        }
      }
    }
  }
  else
  {
    block_y = mb_y;
    block_x = mb_x;

    pic_pix_x = currMB->pix_x + block_x;

    cbp_blk_mask = (block_x>>2) + block_y;

    //===== prediction of 4x4 block =====
    luma_prediction (currMB, block_x, block_y, 8, 8, p_dir, list_mode, ref_idx, bipred_me);

    //===== compute prediction residual ======            
    compute_residue (&p_Img->pCurImg[currMB->opix_y + block_y], &currSlice->mb_pred[0][block_y], &currSlice->mb_ores[0][block_y], block_x, pic_pix_x, 8, 8);

    for (uv = PLANE_U; uv <= PLANE_V; ++uv)
    {
      select_plane(p_Img, (ColorPlane) uv);
      chroma_prediction (currMB, uv - 1, block_x, block_y, 8, 8, p_dir, list_mode[0], list_mode[1], ref_idx[0], ref_idx[1], bipred_me);

      //===== compute prediction residual ======            
      compute_residue (&p_Img->pImgOrg[uv][currMB->opix_y + block_y], &currSlice->mb_pred[uv][block_y], &currSlice->mb_ores[uv][block_y], block_x, pic_pix_x, 8, 8);
    }
    select_plane(p_Img, PLANE_Y);

    if (currSlice->NoResidueDirect != 1 && !skipped)
    {
      if (currSlice->slice_type != SP_SLICE)
        nonzero = currMB->trans_8x8 (currMB, PLANE_Y, block8x8, &coeff_cost, 0);

      if (nonzero)
      {
        (*cbp_blk) |= 51 << (4*block8x8 - 2*(block8x8 & 0x01)); // corresponds to 110011, as if all four 4x4 blocks contain coeff, shifted to block position
        (*cbp)     |= cbp_mask;                               // one bit for the 4x4 blocks of an 8x8 block
      }

      if (currSlice->slice_type != SP_SLICE)
      {
        for (uv = PLANE_U; uv <= PLANE_V; ++uv)
        {
          select_plane(p_Img, (ColorPlane) uv);
          nonzero = currMB->trans_8x8( currMB, (ColorPlane) uv, block8x8, &currSlice->coeff_cost_cr[uv], 0);
          if (nonzero)
          {
            (currSlice->cur_cbp_blk[uv]) |= 51 << (4*block8x8-2*(block8x8 & 0x01)); // corresponds to 110011, as if all four 4x4 blocks contain coeff, shifted to block position
            (currSlice->cmp_cbp[uv])     |= cbp_mask;           // one bit for the 4x4 blocks of an 8x8 block
          }
        }
        select_plane(p_Img, PLANE_Y);
      }        
    }
  }

  /*
  The purpose of the action below is to prevent that single or 'expensive' coefficients are coded.
  With 4x4 transform there is larger chance that a single coefficient in a 8x8 or 16x16 block may be nonzero.
  A single small (level=1) coefficient in a 8x8 block will cost: 3 or more bits for the coefficient,
  4 bits for EOBs for the 4x4 blocks,possibly also more bits for CBP.  Hence the total 'cost' of that single
  coefficient will typically be 10-12 bits which in a RD consideration is too much to justify the distortion improvement.
  The action below is to watch such 'single' coefficients and set the reconstructed block equal to the prediction according
  to a given criterium.  The action is taken only for inter luma blocks.

  Notice that this is a pure encoder issue and hence does not have any implication on the standard.
  coeff_cost is a parameter set in dct_4x4() and accumulated for each 8x8 block.  If level=1 for a coefficient,
  coeff_cost is increased by a number depending on RUN for that coefficient.The numbers are (see also dct_4x4()): 3,2,2,1,1,1,0,0,...
  when RUN equals 0,1,2,3,4,5,6, etc.
  If level >1 coeff_cost is increased by 9 (or any number above 3). The threshold is set to 3. This means for example:
  1: If there is one coefficient with (RUN,level)=(0,1) in a 8x8 block this coefficient is discarded.
  2: If there are two coefficients with (RUN,level)=(1,1) and (4,1) the coefficients are also discarded
  sum_cnt_nonz[0] is the accumulation of coeff_cost over a whole macro block.  If sum_cnt_nonz[0] is 5 or less for the whole MB,
  all nonzero coefficients are discarded for the MB and the reconstructed block is set equal to the prediction.
  */

  if (currSlice->NoResidueDirect != 1 && !skipped && coeff_cost <= _LUMA_COEFF_COST_ &&
    ((currMB->qp_scaled[0])!=0 || p_Img->lossless_qpprime_flag==0)&&
    !(currSlice->slice_type == SP_SLICE && (p_Img->si_frame_indicator == TRUE || p_Img->sp2_frame_indicator == TRUE )))// last set of conditions
    // cannot skip when perfect reconstruction is as in switching pictures or SI pictures
  {
    coeff_cost  = 0;
    (*cbp)     &=  (63 - cbp_mask);
    (*cbp_blk) &= ~(51 << (4 * block8x8 - 2 * (block8x8 & 0x01)));

    memset( currSlice->cofAC[block8x8][0][0], 0, 4 * 2 * 65 * sizeof(int));

    copy_image_data_8x8(&p_Img->enc_picture->imgY[currMB->pix_y + mb_y], &currSlice->mb_pred[0][mb_y], currMB->pix_x + mb_x, mb_x);

    if (currSlice->slice_type == SP_SLICE)
    {
      for (i=mb_x; i < mb_x + BLOCK_SIZE_8x8; i+=BLOCK_SIZE)
        for (j=mb_y; j < mb_y + BLOCK_SIZE_8x8; j+=BLOCK_SIZE)
          copyblock_sp(currMB, PLANE_Y, i, j);
    }
  }

  for (uv = PLANE_U; uv <= PLANE_V; ++uv)
  {
    if (currSlice->NoResidueDirect != 1 && !skipped && currSlice->coeff_cost_cr[uv] <= _LUMA_COEFF_COST_ &&
      (currMB->qp_scaled[uv]!=0 || p_Img->lossless_qpprime_flag==0))// last set of conditions
    {
      currSlice->coeff_cost_cr[uv] = 0;
      currSlice->cmp_cbp[uv] &= (63 - cbp_mask);
      currSlice->cur_cbp_blk[uv] &= ~(51 << (4*block8x8-2*(block8x8 & 0x01)));

      memset( currSlice->cofAC[block8x8 + 4 * uv][0][0], 0, 4 * 2 * 65 * sizeof(int));

      copy_image_data_8x8(&p_Img->enc_picture->imgUV[uv - 1][currMB->pix_y + mb_y], &currSlice->mb_pred[uv][mb_y], currMB->pix_x + mb_x, mb_x);
    }
  }

  return coeff_cost;
}


/*!
 ************************************************************************
 * \brief
 *    Residual Coding of a Luma macroblock (not for intra)
 ************************************************************************
 */
void luma_residual_coding_p444 (Macroblock *currMB)
{
  Slice *currSlice = currMB->p_slice;
  ImageParameters *p_Img = currMB->p_Img;

  int uv, i,j,block8x8,b8_x,b8_y;
  int list_mode[2];
  char list_ref_idx[2];
  short p_dir;
  int sum_cnt_nonz[3] = {0 ,0, 0};
  int is_skip = currSlice->slice_type == P_SLICE && currMB->mb_type == PSKIP;   

  currMB->cbp     = 0;
  currMB->cbp_blk = 0;
  currSlice->cmp_cbp[1] = currSlice->cmp_cbp[2] = 0;
  currSlice->cur_cbp_blk[1] = currSlice->cur_cbp_blk[2] = 0;

  if (is_skip || currMB->mb_type == 1)
  {

    for (block8x8=0; block8x8<4; ++block8x8)
    {    
      currSlice->set_modes_and_reframe (currMB, block8x8, &p_dir, list_mode, list_ref_idx);

      sum_cnt_nonz[0] += luma_residual_coding_p444_16x16 (currMB, block8x8, p_dir, list_mode, list_ref_idx);
      sum_cnt_nonz[1] += currSlice->coeff_cost_cr[1];
      sum_cnt_nonz[2] += currSlice->coeff_cost_cr[2];
    }
  }
  else
  {
    for (block8x8=0; block8x8<4; ++block8x8)
    {    
      currSlice->set_modes_and_reframe (currMB, block8x8, &p_dir, list_mode, list_ref_idx);

      sum_cnt_nonz[0] += luma_residual_coding_p444_8x8 (currMB, &(currMB->cbp), &(currMB->cbp_blk), block8x8, p_dir, list_mode, list_ref_idx);

      if(p_Img->P444_joined) 
      {
        sum_cnt_nonz[1] += currSlice->coeff_cost_cr[1];
        sum_cnt_nonz[2] += currSlice->coeff_cost_cr[2];
      }
    }
  }

  if ((is_skip || 
    (sum_cnt_nonz[0] <= _LUMA_MB_COEFF_COST_ && ((currMB->qp_scaled[0])!=0 || p_Img->lossless_qpprime_flag==0))) &&
    !(currSlice->slice_type == SP_SLICE && (p_Img->si_frame_indicator == TRUE || p_Img->sp2_frame_indicator == TRUE)))// modif ES added last set of conditions
    //cannot skip if SI or switching SP frame perfect reconstruction is needed
  {
    currMB->cbp     &= 0xfffff0 ;
    currMB->cbp_blk &= 0xff0000 ;

    copy_image_data_16x16(&p_Img->enc_picture->imgY[currMB->pix_y], currSlice->mb_pred[0], currMB->pix_x, 0);

    memset( currSlice->cofAC[0][0][0], 0, 2080 * sizeof(int)); // 4 * 4 * 2 * 65

    if (currSlice->slice_type == SP_SLICE)
    {
      for(block8x8=0;block8x8<4; ++block8x8)
      {
        b8_x=(block8x8&1)<<3;
        b8_y=(block8x8&2)<<2;
        for (i = b8_x; i < b8_x + BLOCK_SIZE_8x8; i += 4)
          for (j = b8_y; j < b8_y + BLOCK_SIZE_8x8;j += 4)
            copyblock_sp(currMB, PLANE_Y, i, j);
      }
    }
  }

  for (uv = PLANE_U; uv <= PLANE_V; ++uv)
  {
    if(is_skip || (sum_cnt_nonz[uv] <= _LUMA_MB_COEFF_COST_ &&
      ((currMB->qp_scaled[uv])!=0 ||p_Img->lossless_qpprime_flag==0)))
    {
      currSlice->cmp_cbp[uv] &= 0xfffff0 ;
      currSlice->cur_cbp_blk[uv] &= 0xff0000 ;

      copy_image_data_16x16(&p_Img->enc_picture->p_img[uv][currMB->pix_y], currSlice->mb_pred[uv], currMB->pix_x, 0);

      memset( currSlice->cofAC[4 * uv][0][0], 0, 2080 * sizeof(int)); // 4 * 4 * 2 * 65
    }
    currMB->cbp |= currSlice->cmp_cbp[uv];
  }
}



