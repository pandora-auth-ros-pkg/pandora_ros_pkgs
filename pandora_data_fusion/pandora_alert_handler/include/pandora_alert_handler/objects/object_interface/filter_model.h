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

#ifndef PANDORA_ALERT_HANDLER_OBJECTS_OBJECT_INTERFACE_FILTER_MODEL_H
#define PANDORA_ALERT_HANDLER_OBJECTS_OBJECT_INTERFACE_FILTER_MODEL_H

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
    explicit FilterModel(float system_noise_sd = 0.05,
        float measurement_sd = 0.5);

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
      * @brief Initializes SystemModel according to given parameters.
      * @param systemStdDev [float] initial standard deviation
      * @return void
      */
    void initializeSystemModel(float systemStdDev);

    /**
      * @brief Initializes MeasurementModel according to given parameters.
      * @param measurementStdDev [float] initial standard deviation
      * @return void
      */
    void initializeMeasurementModel(float measurementStdDev);

   private:
    //!< Filter's system pdf
    AnalyticGaussianPtr systemPdfPtr_;
    //!< Filter's system model for dimension x
    SystemModelPtr systemModelX_;
    //!< Filter's system model for dimension y
    SystemModelPtr systemModelY_;
    //!< Filter's system model for dimension z
    SystemModelPtr systemModelZ_;

    //!< Filter's measurement pdf for x
    AnalyticGaussianPtr measurementPdfPtrX_;
    //!< Filter's measurement pdf for y
    AnalyticGaussianPtr measurementPdfPtrY_;
    //!< Filter's measurement pdf for z
    AnalyticGaussianPtr measurementPdfPtrZ_;
    //!< Filter's measurement model for dimension x
    MeasurementModelPtr measurementModelX_;
    //!< Filter's measurement model for dimension y
    MeasurementModelPtr measurementModelY_;
    //!< Filter's measurement model for dimension z
    MeasurementModelPtr measurementModelZ_;
  };

  typedef boost::shared_ptr<FilterModel> FilterModelPtr;
  typedef boost::shared_ptr<FilterModel const> FilterModelConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // PANDORA_ALERT_HANDLER_OBJECTS_OBJECT_INTERFACE_FILTER_MODEL_H
