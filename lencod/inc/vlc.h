
/*!
 *************************************************************************************
 * \file vlc.h
 *
 * \brief
 *    Prototypes for VLC coding funtions
 * \author
 *     Karsten Suehring
 *************************************************************************************
 */

#ifndef _VLC_H_
#define _VLC_H_

#include "enc_statistics.h"

extern Boolean u_1  (char *tracestring, int value, Bitstream *bitstream);
extern int se_v (char *tracestring, int value, Bitstream *bitstream);
extern int ue_v (char *tracestring, int value, Bitstream *bitstream);
extern int u_v  (int n, char *tracestring, int value, Bitstream *bitstream);

extern void levrun_linfo_c2x2(int level,int run,int *len,int *info);
extern void levrun_linfo_inter(int level,int run,int *len,int *info);

extern void writeSE_Fix                  (SyntaxElement *se, Bitstream *bitstream);
extern void writeSE_UVLC                 (SyntaxElement *se, DataPartition *dp);
extern void writeSE_SVLC                 (SyntaxElement *se, DataPartition *dp);
extern void writeSE_Flag                 (SyntaxElement *se, DataPartition *dp);
extern void writeSE_invFlag              (SyntaxElement *se, DataPartition *dp);
extern void writeSE_Dummy                (SyntaxElement *se, DataPartition *dp);

extern void  writeCBP_VLC                (Macroblock* currMB, SyntaxElement *se, DataPartition *dp);
extern void  writeIntraPredMode_CAVLC    (SyntaxElement *se, DataPartition *dp);
extern int   writeSyntaxElement2Buf_UVLC (SyntaxElement *se, Bitstream* this_streamBuffer );
extern void  writeUVLC2buffer            (SyntaxElement *se, Bitstream *currStream);
extern void  writeVlcByteAlign           (ImageParameters *p_Img, Bitstream* currStream, StatParameters *cur_stats);
extern int   writeSyntaxElement2Buf_Fixed(SyntaxElement *se, Bitstream* this_streamBuffer );
extern int   symbol2uvlc                 (SyntaxElement *se);
extern void  ue_linfo       (int n, int dummy, int *len,int *info);
extern void  se_linfo       (int mvd, int dummy, int *len,int *info);
extern void  cbp_linfo_intra_normal(int cbp, int dummy, int *len,int *info);
extern void  cbp_linfo_intra_other (int cbp, int dummy, int *len,int *info);
extern void  cbp_linfo_inter_normal(int cbp, int dummy, int *len,int *info);
extern void  cbp_linfo_inter_other (int cbp, int dummy, int *len,int *info);

extern int   writeSyntaxElement_VLC(SyntaxElement *se, DataPartition *this_dataPart);
extern int   writeSyntaxElement_TotalZeros(SyntaxElement *se, DataPartition *this_dataPart);
extern int   writeSyntaxElement_TotalZerosChromaDC(ImageParameters *p_Img, SyntaxElement *se, DataPartition *this_dataPart);
extern int   writeSyntaxElement_Run(SyntaxElement *se, DataPartition *this_dataPart);
extern int   writeSyntaxElement_NumCoeffTrailingOnes(SyntaxElement *se, DataPartition *this_dataPart);
extern int   writeSyntaxElement_NumCoeffTrailingOnesChromaDC(ImageParameters *p_Img, SyntaxElement *se, DataPartition *this_dataPart);
extern int   writeSyntaxElement_Level_VLC1(SyntaxElement *se, DataPartition *this_dataPart, int profile_idc);
extern int   writeSyntaxElement_Level_VLCN(SyntaxElement *se, int vlc, DataPartition *this_dataPart, int profile_idc);

extern void writeUVLC_CAVLC (Macroblock *currMB, SyntaxElement *se, DataPartition *dp);
extern void writeSVLC_CAVLC (Macroblock *currMB, SyntaxElement *se, DataPartition *dp);
extern void writeFlag_CAVLC (Macroblock *currMB, SyntaxElement *se, DataPartition *dp);

extern void reset_mb_nz_coeff(ImageParameters *p_Img, int mb_number); 

#endif

