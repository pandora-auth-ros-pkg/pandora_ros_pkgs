/**@file
   Functions for detecting SIFT image features.
   
   For more information, refer to:
   
   Lowe, D.  Distinctive image features from scale-invariant keypoints.
   <EM>International Journal of Computer Vision, 60</EM>, 2 (2004),
   pp.91--110.
  
   Copyright (C) 2006-2010  Rob Hess <hess@eecs.oregonstate.edu>
   
   Note: The SIFT algorithm is patented in the United States and cannot be
   used in commercial products without a license from the University of
   British Columbia.  For more information, refer to the file LICENSE.ubc
   that accompanied this distribution.
   
   @version 1.1.2-20100521
*/

#ifndef SIFT_H
#define SIFT_H

#include "pandora_vision_hazmat/imgfeatures.h"

/******************************** Structures *********************************/

/** holds feature data relevant to detection */
struct detection_data
{
  int r;
  int c;
  int octv;
  int intvl;
  double subintvl;
  double scl_octv;
};

struct feature;

/* returns a feature's detection data */
#define feat_detection_data(f) ( (struct detection_data*)(f->feature_data) )


/*************************** Function Prototypes *****************************/

/**
  Finds SIFT features in an image using default parameter values.  All
  detected features are stored in the array pointed to by \a feat.

  @param img the image in which to detect features
  @param feat a pointer to an array in which to store detected features;
   memory for this array is allocated by this function and must be freed by
   the caller using free(*feat)

  @return Returns the number of features stored in \a feat or -1 on failure
  @see _sift_features()
*/
extern int sift_features( IplImage* img, struct feature** feat );



/**
  Finda SIFT features in an image using user-specified parameter values.  All
  detected features are stored in the array pointed to by \a feat.

  @param img the image in which to detect features
  @param feat a pointer to an array in which to store detected features;
   memory for this array is allocated by this function and must be freed by
   the caller using free(*feat)
  @param intvls the number of intervals sampled per octave of scale space
  @param sigma the amount of Gaussian smoothing applied to each image level
   before building the scale space representation for an octave
  @param contr_thr a threshold on the value of the scale space function
   \f$\left|D(\hat{x})\right|\f$, where \f$\hat{x}\f$ is a vector specifying
   feature location and scale, used to reject unstable features;  assumes
   pixel values in the range [0, 1]
  @param curv_thr threshold on a feature's ratio of principle curvatures
   used to reject features that are too edge-like
  @param img_dbl should be 1 if image doubling prior to scale space
   construction is desired or 0 if not
  @param descr_width the width, \f$n\f$, of the \f$n \times n\f$ array of
   orientation histograms used to compute a feature's descriptor
  @param descr_hist_bins the number of orientations in each of the
   histograms in the array used to compute a feature's descriptor

  @return Returns the number of keypoints stored in \a feat or -1 on failure
  @see sift_features()
*/
extern int _sift_features( 
  IplImage* img, 
  struct feature** feat, 
  int intvls,
  double sigma, 
  double contr_thr, 
  int curv_thr,
  int img_dbl, 
  int descr_width, 
  int descr_hist_bins 
);

#endif
