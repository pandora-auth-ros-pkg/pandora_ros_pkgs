// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_FILTER_MODEL_H
#define ALERT_HANDLER_FILTER_MODEL_H

#include <vector>
#include <boost/shared_ptr.hpp>

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

//!< Type definitions
typedef BFL::LinearAnalyticConditionalGaussian
  AnalyticGaussian;
typedef boost::shared_ptr<AnalyticGaussian> AnalyticGaussianPtr;
typedef BFL::LinearAnalyticSystemModelGaussianUncertainty
  SystemModel;
typedef boost::shared_ptr<SystemModel> SystemModelPtr;
typedef std::vector<SystemModelPtr> SystemModelPtrVector;
typedef BFL::LinearAnalyticMeasurementModelGaussianUncertainty
  MeasurementModel;
typedef boost::shared_ptr<MeasurementModel> MeasurementModelPtr;
typedef std::vector<MeasurementModelPtr> MeasurementModelPtrVector;

/**
 * @class FilterModel
 * @brief Describes the model of a Kalman filter to be used.
 */
class FilterModel
{
 public:

  /**
   * @brief Default constructor.
   */
  FilterModel(float system_noise_sd = 0.05, float measurement_noise_sd = 0.5);

  /**
   * @brief Getter for linear analytic system's model.
   * @return SystemModelPtrVector Vector that contains the models one for
   * each dimension.
   */
  SystemModelPtrVector getSystemModels() const;

  /**
   * @brief Getter for linear analytic measurement's model.
   * @return MeasurementModelPtrVector Vector that contains the models one 
   * for each dimension.
   */
  MeasurementModelPtrVector getMeasurementModels() const;

  /**
   * @brief Initializes the FilterModel according to given parameters.
   * @return void
   */
  void initializeFilterModel();

  /**
   * @brief Sets standar deviation parameters.
   * @param system_noise_sd [float] system model's standar deviation
   * @param measurement_noise_sd [float] measurement model's standar deviation
   * @return void
   */
  void setParams(float system_noise_sd, float measurement_noise_sd);

 private:
 
  //!< Filter's system pdf
  AnalyticGaussianPtr systemPdfPtr_;
  //!< Filter's system model for dimension x
  SystemModelPtr systemModelX_;
  //!< Filter's system model for dimension y
  SystemModelPtr systemModelY_;
  //!< Filter's system model for dimension z
  SystemModelPtr systemModelZ_;
  
  //!< Filter's measurement pdf
  AnalyticGaussianPtr measurementPdfPtr_;
  //!< Filter's measurement model for dimension x
  MeasurementModelPtr measurementModelX_;
  //!< Filter's measurement model for dimension y
  MeasurementModelPtr measurementModelY_;
  //!< Filter's measurement model for dimension z
  MeasurementModelPtr measurementModelZ_;

  //!< params
  //!< System's noise standar deviation
  float SYSTEM_NOISE_SD;
  //!< Measurement's noise standar deviation
  float MEASUREMENT_NOISE_SD;

};

typedef boost::shared_ptr<FilterModel> FilterModelPtr;
typedef boost::shared_ptr<FilterModel const> FilterModelConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_FILTER_MODEL_H
