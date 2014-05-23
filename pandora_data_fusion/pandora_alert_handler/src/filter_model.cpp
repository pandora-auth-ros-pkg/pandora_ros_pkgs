// "Copyright [year] <Copyright Owner>"

#include "alert_handler/filter_model.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @details Sets the parameters in filter's model and initializes it.
     */
    FilterModel::FilterModel(float system_noise_sd)
    {
      SYSTEM_NOISE_SD = system_noise_sd;
      initializeSystemModel();
    }

    void FilterModel::initializeSystemModel()
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
      systemNoiseVariance(1, 1) = pow(SYSTEM_NOISE_SD, 2);

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
            measurementUncertainty ));
      measurementPdfPtrY_.reset( new AnalyticGaussian(matrixH, 
            measurementUncertainty ));
      measurementPdfPtrZ_.reset( new AnalyticGaussian(matrixH, 
            measurementUncertainty ));

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

