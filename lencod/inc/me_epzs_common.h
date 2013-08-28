
/*!
************************************************************************
* \file
*     me_epzs_common.h
*
* \author
*    Alexis Michael Tourapis        <alexis.tourapis@dolby.com>
*
* \date
*    11. August 2006
*
* \brief
*    Headerfile for common functions for the EPZS motion estimation
**************************************************************************
*/


#ifndef _ME_EPZS_COMMON_H_
#define _ME_EPZS_COMMON_H_


// Structure definitions
typedef struct
{
  int         mb_adaptive_frame_field_flag;
  int         size_x, size_y;

  // Frame
  MotionVector ***frame;  //!< motion vector       [list][subblock_x][subblock_y]
  // Top field
  MotionVector ***top;    //!< motion vector       [list][subblock_x][subblock_y]
  // Bottom field
  MotionVector ***bot;    //!< motion vector       [list][subblock_x][subblock_y]
} EPZSColocParams;

typedef struct
{
  MotionVector motion;
  int start_nmbr;
  int next_points;
}
SPoint;

struct epzs_struct
{
  int    searchPoints;
  SPoint *point;
  int    stopSearch;
  int    nextLast;
  struct epzs_struct *nextpattern;
};

typedef struct epzs_struct EPZSStructure;

typedef enum
{
  SDIAMOND  = 0,
  SQUARE    = 1,
  EDIAMOND  = 2,
  LDIAMOND  = 3,
  SBDIAMOND = 4
} EPZSPatterns;

struct epzs_params {
  ImageParameters *p_Img;
  uint16 BlkCount;
  int   searcharray;

  int medthres[8];
  int maxthres[8];
  int minthres[8];
  int subthres[8];

  int mv_scale       [6][MAX_REFERENCE_PICTURES][MAX_REFERENCE_PICTURES];
  int mv_scale_update[6][MAX_REFERENCE_PICTURES][MAX_REFERENCE_PICTURES];

  uint16 **EPZSMap;  //!< Memory Map definition

#if EPZSREF
  MotionVector *****p_motion;  //!< Array for storing Motion Vectors
#else
  MotionVector ****p_motion;  //!< Array for storing Motion Vectors
#endif

  int ***distortion;  //!< Array for storing SAD Values
  int ***bi_distortion;  //!< Array for storing SAD Values
  //
  EPZSStructure *searchPattern;
  EPZSStructure *searchPatternD;
  EPZSStructure *predictor;
  // Window predictors
  EPZSStructure *window_predictor;
  EPZSStructure *window_predictor_ext;
  // Colocated helper
  EPZSColocParams *p_colocated;
};

typedef struct epzs_params EPZSParameters;


extern void  EPZSBlockTypePredictorsMB (Slice *currSlice, MEBlock *mv_block, SPoint *point, int *prednum);
extern short EPZSSpatialPredictors     (EPZSParameters *p_EPZS, PixelPos *block, int list, int list_offset, short ref, char **refPic, short ***tmp_mv);
extern void  EPZSTemporalPredictors    (Macroblock *currMB, StorablePicture *ref_picture, EPZSParameters *p_EPZS, MEBlock *mv_block, int *prednum, int stopCriterion, int min_mcost);
extern void  EPZSBlockTypePredictors   (Slice *currSlice, MEBlock *mv_block, SPoint *point, int *prednum);
extern void  EPZSWindowPredictors      (MotionVector *mv, EPZSStructure *predictor, int *prednum, EPZSStructure *windowPred);
extern void  EPZSSpatialMemPredictors  (EPZSParameters *p_EPZS, MEBlock *mv_block, int list, int *prednum, int img_width);
extern int   EPZSDetermineStopCriterion(EPZSParameters *p_EPZS, int* prevSad, MEBlock *mv_block ,int lambda_dist);

extern void  EPZSDelete                (ImageParameters *p_Img);
extern void  EPZSStructDelete          (Slice *currSlice);
extern void  EPZSSliceInit             (Slice *currSlice);
extern int   EPZSInit                  (ImageParameters *p_Img);
extern int   EPZSStructInit            (Slice *currSlice);
extern void  EPZSOutputStats           (InputParameters *p_Inp, FILE * stat, short stats_file);



/*!
***********************************************************************
* \brief
*    Add Predictor/normalize function
*    Returns if succesfully added (i.e. if not 0)
*     
*    
***********************************************************************
*/
static inline int add_predictor(MotionVector *cur_mv, MotionVector prd_mv, int mvScale, int shift_mv)
{
  *cur_mv = prd_mv;
  cur_mv->mv_x = (short) rshift_rnd_sf((mvScale * cur_mv->mv_x), shift_mv);
  cur_mv->mv_y = (short) rshift_rnd_sf((mvScale * cur_mv->mv_y), shift_mv);
  return (*((int*) cur_mv) != 0);
}

static inline void scale_mv(MotionVector *out_mv, int scale, short *mv, int shift_mv)
{
  out_mv->mv_x = (short) rshift_rnd_sf((scale * mv[0]), shift_mv);
  out_mv->mv_y = (short) rshift_rnd_sf((scale * mv[1]), shift_mv);
}


static inline void compute_scaled(MotionVector *MotionVector0, MotionVector *MotionVector1, int tempmv_scale[2], short *mv, int invmv_precision)
{
  MotionVector0->mv_x = (short) iClip3 (-32768, 32767, rshift_rnd_sf((tempmv_scale[LIST_0] * mv[0]), invmv_precision));
  MotionVector0->mv_y = (short) iClip3 (-32768, 32767, rshift_rnd_sf((tempmv_scale[LIST_0] * mv[1]), invmv_precision));
  MotionVector1->mv_x = (short) iClip3 (-32768, 32767, rshift_rnd_sf((tempmv_scale[LIST_1] * mv[0]), invmv_precision));
  MotionVector1->mv_y = (short) iClip3 (-32768, 32767, rshift_rnd_sf((tempmv_scale[LIST_1] * mv[1]), invmv_precision));
}

#endif

