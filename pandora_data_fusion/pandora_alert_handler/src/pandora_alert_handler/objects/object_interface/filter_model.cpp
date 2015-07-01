/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#include <vector>

#include "pandora_alert_handler/objects/object_interface/filter_model.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

  /**
    * @details Sets the parameters in filter's model and initializes it.
    */
  FilterModel::FilterModel(float system_noise_sd, float measurement_sd)
  {
    initializeSystemModel(system_noise_sd);
    initializeMeasurementModel(measurement_sd);
  }

  void FilterModel::initializeSystemModel(float systemStdDev)
  {
    //!< System Model Initialization
    //!< Filter's combined matrix
    std::vector<MatrixWrapper::Matrix> matrixAB;
    //!< Filter's system matrix A
    MatrixWrapper::Matrix matrixA(1, 1);
    //!< Filter's system matrix B
    MatrixWrapper::Matrix matrixB(1, 1);
    //!< Filter's system noise mean
    MatrixWrapper::ColumnVector systemNoiseMean(1);
    //!< Filter's system noise covariance
    MatrixWrapper::SymmetricMatrix systemNoiseVariance(1);

    matrixA(1, 1) = 1.0;
    matrixB(1, 1) = 0.0;

    matrixAB.push_back(matrixA);
    matrixAB.push_back(matrixB);

    systemNoiseMean(1) = 0.0;
    systemNoiseVariance(1, 1) = pow(systemStdDev, 2);

    BFL::Gaussian systemUncertainty(systemNoiseMean, systemNoiseVariance);
    systemPdfPtr_.reset( new AnalyticGaussian(matrixAB, systemUncertainty) );

    systemModelX_.reset( new SystemModel(systemPdfPtr_.get()) );
    systemModelY_.reset( new SystemModel(systemPdfPtr_.get()) );
    systemModelZ_.reset( new SystemModel(systemPdfPtr_.get()) );
  }

  void FilterModel::initializeMeasurementModel(float measurementStdDev)
  {
    //!< Measurement Model Initialization
    //!< Filter's measurement matrix H
    MatrixWrapper::Matrix matrixH(1, 1);
    matrixH(1, 1) = 1.0;
    //!< Filter's measurement noise mean
    MatrixWrapper::ColumnVector measurementMean(1);
    //!< Filter's measurement noise covariance
    MatrixWrapper::SymmetricMatrix measurementVariance(1);

    measurementMean(1) = 0.0;
    measurementVariance(1, 1) = pow(measurementStdDev, 2);

    BFL::Gaussian measurementUncertainty(measurementMean,
        measurementVariance);
    measurementPdfPtrX_.reset( new AnalyticGaussian(matrixH,
          measurementUncertainty) );
    measurementPdfPtrY_.reset( new AnalyticGaussian(matrixH,
          measurementUncertainty) );
    measurementPdfPtrZ_.reset( new AnalyticGaussian(matrixH,
          measurementUncertainty) );

    measurementModelX_.reset( new MeasurementModel(measurementPdfPtrX_.get()) );
    measurementModelY_.reset( new MeasurementModel(measurementPdfPtrY_.get()) );
    measurementModelZ_.reset( new MeasurementModel(measurementPdfPtrZ_.get()) );
  }

  SystemModelPtrVector FilterModel::getSystemModels() const
  {
    SystemModelPtrVector systemModels;
    systemModels.push_back(systemModelX_);
    systemModels.push_back(systemModelY_);
    systemModels.push_back(systemModelZ_);
    return systemModels;
  }

  MeasurementModelPtrVector FilterModel::getMeasurementModels() const
  {
    MeasurementModelPtrVector measurementModels;
    measurementModels.push_back(measurementModelX_);
    measurementModels.push_back(measurementModelY_);
    measurementModels.push_back(measurementModelZ_);
    return measurementModels;
  }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion
