// "Copyright [year] <Copyright Owner>"

#include "alert_handler/filter_model.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{
  
/**
 * @details Sets the parameters in filter's model and initializes it.
 */
FilterModel::FilterModel(float system_noise_sd, float measurement_noise_sd)
{
  SYSTEM_NOISE_SD = system_noise_sd;
  MEASUREMENT_NOISE_SD = measurement_noise_sd;
  initializeFilterModel();
}

/**
 * @details Sets the parameters in filter's model.
 */
void FilterModel::setParams(float system_noise_sd, float measurement_noise_sd)
{
  SYSTEM_NOISE_SD = system_noise_sd;
  MEASUREMENT_NOISE_SD = measurement_noise_sd;
}

void FilterModel::initializeFilterModel()
{
  //!< System Model Initialization
  //!< Filter's combined matrix
  std::vector<MatrixWrapper::Matrix> matrixAB_;
  //!< Filter's system matrix A
  MatrixWrapper::Matrix matrixA_(1, 1);
  //!< Filter's system matrix B
  MatrixWrapper::Matrix matrixB_(1, 1);
  //!< Filter's system noise mean
  MatrixWrapper::ColumnVector systemNoiseMu_(1);
  //!< Filter's system noise covariance
  MatrixWrapper::SymmetricMatrix systemNoiseVar_(1);

  matrixA_(1, 1) = 1.0;
  matrixB_(1, 1) = 0.0;
  
  matrixAB_.push_back(matrixA_);
  matrixAB_.push_back(matrixB_);
  
  systemNoiseMu_(1) = 0.0;
  systemNoiseVar_(1, 1) = pow(SYSTEM_NOISE_SD, 2);
  
  BFL::Gaussian systemUncertainty(systemNoiseMu_, systemNoiseVar_); 
  systemPdfPtr_.reset( new AnalyticGaussian(matrixAB_, systemUncertainty) );

  systemModelX_.reset( new SystemModel(systemPdfPtr_.get()) );
  systemModelY_.reset( new SystemModel(systemPdfPtr_.get()) );
  systemModelZ_.reset( new SystemModel(systemPdfPtr_.get()) );

  //!< Measurement Model Initialization
  //!< Filter's measurement matrix H
  MatrixWrapper::Matrix matrixH_(1, 1);
  matrixH_(1, 1) = 1.0;
  //!< Filter's measurement noise mean
  MatrixWrapper::ColumnVector measurementNoiseMu_(1);
  //!< Filter's measurement noise covariance
  MatrixWrapper::SymmetricMatrix measurementNoiseVar_(1);
 
  measurementNoiseMu_(1) = 0.0;
  measurementNoiseVar_(1, 1) = pow(MEASUREMENT_NOISE_SD, 2);
  
  BFL::Gaussian measurementUncertainty(measurementNoiseMu_, 
      measurementNoiseVar_);
  measurementPdfPtr_.reset( new AnalyticGaussian(matrixH_, 
      measurementUncertainty ));

  measurementModelX_.reset( new MeasurementModel(measurementPdfPtr_.get()) );
  measurementModelY_.reset( new MeasurementModel(measurementPdfPtr_.get()) );
  measurementModelZ_.reset( new MeasurementModel(measurementPdfPtr_.get()) );
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

