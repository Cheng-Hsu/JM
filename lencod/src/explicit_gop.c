/*!
 *************************************************************************************
 * \file explicit_gop.c
 *
 * \brief
 *    Code for explicit gop support and hierarchical coding.
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Alexis Michael Tourapis                     <alexismt@ieee.org>
 *************************************************************************************
 */

#include "contributors.h"

#include <ctype.h>
#include <limits.h>
#include "global.h"

#include "explicit_gop.h"
#include "image.h"
#include "nalucommon.h"
#include "report.h"

/*!
************************************************************************
* \brief
*    Generation of hierarchical GOP
************************************************************************
*/
void create_hierarchy(ImageParameters *p_Img, InputParameters *p_Inp)
{
  int i, j;
  int centerB = (p_Inp->NumberBFrames >> 1);
  GOP_DATA tmp;

  if (p_Inp->HierarchicalCoding == 1)
  {
    for (i=0;i<p_Inp->NumberBFrames;i++)
    {
      if (i < centerB)
      {
        p_Img->gop_structure[i].slice_type = ( p_Inp->PReplaceBSlice ) ? P_SLICE : B_SLICE;
        p_Img->gop_structure[i].display_no = i * 2 + 1;
        p_Img->gop_structure[i].hierarchy_layer = 1;
        p_Img->gop_structure[i].reference_idc = NALU_PRIORITY_LOW;
        p_Img->gop_structure[i].slice_qp = imax(0, (p_Inp->qp[0][B_SLICE] + (p_Inp->HierarchyLevelQPEnable ? -1: p_Inp->qpBRSOffset[0])));

      }
      else
      {
        p_Img->gop_structure[i].slice_type = ( p_Inp->PReplaceBSlice ) ? P_SLICE : B_SLICE;
        p_Img->gop_structure[i].display_no = (i - centerB) * 2;
        p_Img->gop_structure[i].hierarchy_layer = 0;
        p_Img->gop_structure[i].reference_idc = NALU_PRIORITY_DISPOSABLE;
        p_Img->gop_structure[i].slice_qp = p_Inp->qp[0][B_SLICE];
      }
    }
    p_Img->GopLevels = 2;
  }
  else
  {
    int GOPlevels = 1;
    int Bframes = p_Inp->NumberBFrames;
    int *curGOPLevelfrm,*curGOPLeveldist ;
    int curlevel = GOPlevels ;
    int i;

    while (((Bframes + 1 ) >> GOPlevels) > 1)
    {
      GOPlevels ++;
    }

    curlevel = GOPlevels;
    p_Img->GopLevels = GOPlevels;
    if (NULL == (curGOPLevelfrm = (int*)malloc(GOPlevels * sizeof(int)))) no_mem_exit("create_hierarchy:curGOPLevelfrm");
    if (NULL == (curGOPLeveldist= (int*)malloc(GOPlevels * sizeof(int)))) no_mem_exit("create_hierarchy:curGOPLeveldist");

    for (i=0; i <p_Inp->NumberBFrames; i++)
    {
      p_Img->gop_structure[i].display_no = i;
      p_Img->gop_structure[i].slice_type = ( p_Inp->PReplaceBSlice ) ? P_SLICE : B_SLICE;
      p_Img->gop_structure[i].hierarchy_layer = 0;
      p_Img->gop_structure[i].reference_idc = NALU_PRIORITY_DISPOSABLE;
      p_Img->gop_structure[i].slice_qp = p_Inp->qp[0][B_SLICE];
    }

    for (j = 1; j < GOPlevels; j++)
    {
      for (i = (1 << j) - 1; i < Bframes + 1 - (1 << j); i += (1 << j)) 
      {
        p_Img->gop_structure[i].hierarchy_layer  = j;
        p_Img->gop_structure[i].reference_idc  = NALU_PRIORITY_LOW;
        p_Img->gop_structure[i].slice_qp = imax(0, p_Inp->qp[0][B_SLICE] + (p_Inp->HierarchyLevelQPEnable ? -j: p_Inp->qpBRSOffset[0]));
        //KHHan, for inter lossless code(referenced B picture)
        //if(!p_Inp->lossless_qpprime_y_zero_flag)
        //  p_Img->gop_structure[i].slice_qp = imax(0, p_Inp->qp[0][B_SLICE] + (p_Inp->HierarchyLevelQPEnable ? -j: p_Inp->qpBRSOffset[0]));
        //else
        //  p_Img->gop_structure[i].slice_qp = p_Inp->qp[0][B_SLICE];
      }
    }

    for (i = 1; i < Bframes; i++)
    {
      j = i;

      while (j > 0 && p_Img->gop_structure[j].hierarchy_layer > p_Img->gop_structure[j-1].hierarchy_layer)
      {
        tmp = p_Img->gop_structure[j-1];
        p_Img->gop_structure[j-1] = p_Img->gop_structure[j];
        p_Img->gop_structure[j] = tmp;
        j--;
      }
    }
  }
}


/*!
************************************************************************
* \brief
*    Initialization of GOP structure.
*
************************************************************************
*/
void init_gop_structure(ImageParameters *p_Img, InputParameters *p_Inp)
{
  int max_gopsize = p_Inp->NumberBFrames;

  p_Img->gop_structure = calloc(imax(10,max_gopsize), sizeof (GOP_DATA)); // +1 for reordering
  if (NULL==p_Img->gop_structure)
    no_mem_exit("init_gop_structure: gop_structure");
}


/*!
************************************************************************
* \brief
*    Clear GOP structure
************************************************************************
*/
void clear_gop_structure(ImageParameters *p_Img)
{
  if (p_Img->gop_structure)
    free(p_Img->gop_structure);
}


/*!
************************************************************************
* \brief
*    Interpret GOP struct from input parameters
************************************************************************
*/
void interpret_gop_structure(ImageParameters *p_Img, InputParameters *p_Inp)
{
  int nLength = strlen(p_Inp->ExplicitHierarchyFormat);
  int i =0, k, dqp, display_no;
  int slice_read =0, order_read = 0, stored_read = 0, qp_read =0;
  int coded_frame = 0;

  if (nLength > 0)
  {

    for (i = 0; i < nLength ; i++)
    {
      //! First lets read slice type
      if (slice_read == 0)
      {
        switch (p_Inp->ExplicitHierarchyFormat[i])
        {
        case 'P':
        case 'p':
          p_Img->gop_structure[coded_frame].slice_type = P_SLICE;
          break;
        case 'B':
        case 'b':
          p_Img->gop_structure[coded_frame].slice_type = B_SLICE;
          break;
        case 'I':
        case 'i':
          p_Img->gop_structure[coded_frame].slice_type = I_SLICE;
          break;
        default:
          snprintf(errortext, ET_SIZE, "Slice Type invalid in ExplicitHierarchyFormat param. Please check configuration file.");
          error (errortext, 400);
          break;
        }
        slice_read = 1;
      }
      else
      {
        //! Next is Display Order
        if (order_read == 0)
        {
          if (isdigit((int)(*(p_Inp->ExplicitHierarchyFormat+i))))
          {
            sscanf(p_Inp->ExplicitHierarchyFormat+i,"%d",&display_no);
            p_Img->gop_structure[coded_frame].display_no = display_no;
            order_read = 1;
            if (display_no < 0 || display_no >= p_Inp->NumberBFrames)
            {
              snprintf(errortext, ET_SIZE, "Invalid Frame Order value. Frame position needs to be in [0,%d] range.",p_Inp->NumberBFrames - 1);
              error (errortext, 400);
            }
            for (k=0;k<coded_frame;k++)
            {
              if (p_Img->gop_structure[k].display_no == display_no)
              {
                snprintf(errortext, ET_SIZE, "Frame Order value %d in frame %d already used for enhancement frame %d.",display_no,coded_frame,k);
                error (errortext, 400);
              }
            }
          }
          else
          {
            snprintf(errortext, ET_SIZE, "Slice Type needs to be followed by Display Order. Please check configuration file.");
            error (errortext, 400);
          }
        }
        else if (order_read == 1)
        {
          if (stored_read == 0 && !(isdigit((int)(*(p_Inp->ExplicitHierarchyFormat+i)))))
          {
            switch (p_Inp->ExplicitHierarchyFormat[i])
            {
            case 'E':
            case 'e':
              p_Img->gop_structure[coded_frame].reference_idc = NALU_PRIORITY_DISPOSABLE;
              p_Img->gop_structure[coded_frame].hierarchy_layer = 0;
              break;
            case 'R':
            case 'r':
              p_Img->gop_structure[coded_frame].reference_idc= NALU_PRIORITY_LOW;
              p_Img->gop_structure[coded_frame].hierarchy_layer = 1;
              p_Img->GopLevels = 2;
              break;
            default:
              snprintf(errortext, ET_SIZE, "Reference_IDC invalid in ExplicitHierarchyFormat param. Please check configuration file.");
              error (errortext, 400);
              break;
            }
            stored_read = 1;
          }
          else if (stored_read == 1 && qp_read == 0)
          {
            if (isdigit((int)(*(p_Inp->ExplicitHierarchyFormat+i))))
            {
              sscanf(p_Inp->ExplicitHierarchyFormat+i,"%d",&dqp);

              p_Img->gop_structure[coded_frame].slice_qp = p_Inp->qp[0][ p_Img->gop_structure[coded_frame].slice_type ];
              p_Img->gop_structure[coded_frame].slice_qp = iClip3(-p_Img->bitdepth_luma_qp_scale, 51,p_Img->gop_structure[coded_frame].slice_qp + dqp);
              qp_read = 1;
            }
            else
            {
              snprintf(errortext, ET_SIZE, "Reference_IDC needs to be followed by QP. Please check configuration file.");
              error (errortext, 400);
            }
          }
          else if (stored_read == 1 && qp_read == 1 && !(isdigit((int)(*(p_Inp->ExplicitHierarchyFormat+i)))) && (i < nLength - 3))
          {
            stored_read =0;
            qp_read=0;
            order_read=0;
            slice_read=0;
            i--;
            coded_frame ++;
            if (coded_frame >= p_Inp->NumberBFrames )
            {
              snprintf(errortext, ET_SIZE, "Total number of frames in Enhancement GOP need to be fewer or equal to NumberBFrames parameter.");
              error (errortext, 400);
            }
          }
        }
      }
    }
  }
  else
  {
    snprintf(errortext, ET_SIZE, "ExplicitHierarchyFormat is empty. Please check configuration file.");
    error (errortext, 400);
  }

  p_Inp->NumberBFrames = coded_frame + 1;
}
