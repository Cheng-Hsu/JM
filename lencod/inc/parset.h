
/*!
 **************************************************************************************
 * \file
 *    parset.h
 * \brief
 *    Picture and Sequence Parameter Sets, encoder operations
 *
 * \date 25 November 2002
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Stephan Wenger        <stewe@cs.tu-berlin.de>
 ***************************************************************************************
 */


#ifndef _PARSET_H_
#define _PARSET_H_

#include "parsetcommon.h"
#include "nalu.h"
#include "sei.h"

extern void GenerateParameterSets (ImageParameters *p_Img, InputParameters *p_Inp);
extern void FreeParameterSets     (ImageParameters *p_Img);

extern NALU_t *GenerateSeq_parameter_set_NALU (ImageParameters *p_Img);
extern NALU_t *GeneratePic_parameter_set_NALU (ImageParameters *p_Img, InputParameters *p_Inp, int);
extern NALU_t *GenerateSEImessage_NALU(InputParameters *p_Inp);

// The following are local helpers, but may come handy in the future, hence public
extern void GenerateSequenceParameterSet(seq_parameter_set_rbsp_t *sps, ImageParameters *p_Img, InputParameters *p_Inp, int SPS_id);
extern void GeneratePictureParameterSet( pic_parameter_set_rbsp_t *pps, seq_parameter_set_rbsp_t *sps, 
                                 ImageParameters *p_Img,
                                 InputParameters *p_Inp, int PPS_id,
                                 int WeightedPrediction, int WeightedBiprediction,
                                 int cb_qp_index_offset, int cr_qp_index_offset);

extern int  Scaling_List(short *scalingListinput, short *scalingList, int sizeOfScalingList, short *UseDefaultScalingMatrix, Bitstream *bitstream); 
extern int  GenerateSeq_parameter_set_rbsp (ImageParameters *p_Img, seq_parameter_set_rbsp_t *sps, byte *buf);
extern int  GeneratePic_parameter_set_rbsp (ImageParameters *p_Img, InputParameters *p_Inp, pic_parameter_set_rbsp_t *pps, byte *buf);
extern int  GenerateSEImessage_rbsp (InputParameters *p_Inp, int id, byte *buf);
extern void FreeSPS (seq_parameter_set_rbsp_t *sps);
extern void FreePPS (pic_parameter_set_rbsp_t *pps);

extern int  WriteHRDParameters(seq_parameter_set_rbsp_t *sps, Bitstream *bitstream);
extern void GenerateVUIParameters(seq_parameter_set_rbsp_t *sps, InputParameters *p_Inp);

extern pic_parameter_set_rbsp_t *AllocPPS (void);
extern seq_parameter_set_rbsp_t *AllocSPS (void);


#endif