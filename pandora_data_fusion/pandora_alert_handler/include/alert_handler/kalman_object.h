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

#ifndef ALERT_HANDLER_KALMAN_OBJECT_H
#define ALERT_HANDLER_KALMAN_OBJECT_H

#include "alert_handler/object.h"
#include "alert_handler/filter_model.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class KalmanObject
     * @brief Implementation of Object with Kalman filter.
     * Used as base class for objects that will be filtered.
     */
    template <class DerivedObject> class KalmanObject :
        public Object<DerivedObject>
    {
      public:
        //!< Type Definitions
        typedef boost::shared_ptr<DerivedObject> Ptr;
        typedef boost::shared_ptr<DerivedObject const> ConstPtr;

        typedef boost::shared_ptr<BFL::Gaussian> GaussianPtr;
        typedef BFL::ExtendedKalmanFilter Filter;
        typedef boost::shared_ptr<Filter> FilterPtr;

      public:
        KalmanObject() {};

        /**
         * @brief Initialize filter's pdf for the current object
         * @return void
         */
        void initializeObjectFilter();

        /**
         * @brief Update with measurement Object's information the conviction pdf
         * of this object's filter.
         * @param measurement [ConstPtr const&] Object that carries measurement info.
         * @param model [FilterModelConstPtr const&] Filter's model that the update
         * will be based upon.
         * @return void
         */
        virtual void update(const ObjectConstPtr& measurement);

        /**
         * @brief Getter for variance in x dimension.
         * @return float variance
         */
        float getStdDevX() const
        {
          return sqrt(filterX_->PostGet()->CovarianceGet()(1, 1));
        }

        /**
         * @brief Getter for variance in z dimension.
         * @return float variance
         */
        float getStdDevY() const
        {
          return sqrt(filterY_->PostGet()->CovarianceGet()(1, 1));
        }

        /**
         * @brief Getter for variance in y dimension.
         * @return float variance
         */
        float getStdDevZ() const
        {
          return sqrt(filterZ_->PostGet()->CovarianceGet()(1, 1));
        }

        /**
         * @brief Setter for the reference of filter model.
         * @param modelPtr [FilterModelPtr const&]
         * Reference to filter model.
         * @return void
         */
        static FilterModelPtr getFilterModel()
        {
          return modelPtr_;
        }

        /**
         * @brief Setter for the reference of filter model.
         * @param modelPtr [FilterModelPtr const&]
         * Reference to filter model.
         * @return void
         */
        static void setFilterModel(FilterModelPtr modelPtr)
        {
          modelPtr_ = modelPtr;
        }

      protected:
        //!< Filter's prior gaussian for dimension x
        GaussianPtr priorX_;
        //!< Filter's prior gaussian for dimension y
        GaussianPtr priorY_;
        //!< Filter's prior gaussian for dimension z
        GaussianPtr priorZ_;

        //!< Kalman filter for dimension x
        FilterPtr filterX_;
        //!< Kalman filter for dimension y
        FilterPtr filterY_;
        //!< Kalman filter for dimension z
        FilterPtr filterZ_;

        //!< Pointer to filter's model.
        static FilterModelPtr modelPtr_;

      private:
        friend class ObjectListTest;
    };

    template <class DerivedObject>
      FilterModelPtr KalmanObject<DerivedObject>::
      modelPtr_ = FilterModelPtr(new FilterModel);

    template <class DerivedObject>
      void KalmanObject<DerivedObject>::initializeObjectFilter()
      {
        //!< Priors
        //!< Filter's prior mean
        MatrixWrapper::ColumnVector priorMean(1);
        //!< Filter's prior covariance
        MatrixWrapper::SymmetricMatrix priorVariance(1, 1);

        float stdDeviation = Utils::stdDevFromProbability(
            this->distanceThres_, this->probability_);
        priorVariance(1, 1) = pow(stdDeviation, 2);

        priorMean(1) = this->pose_.position.x;
        priorX_.reset(new BFL::Gaussian(priorMean, priorVariance));
        filterX_.reset(new Filter(priorX_.get()));

        priorMean(1) = this->pose_.position.y;
        priorY_.reset(new BFL::Gaussian(priorMean, priorVariance));
        filterY_.reset(new Filter(priorY_.get()));

        priorMean(1) = this->pose_.position.z;
        priorZ_.reset(new BFL::Gaussian(priorMean, priorVariance));
        filterZ_.reset(new Filter(priorZ_.get()));
      }

    /**
     * @details Updates this object's position according to new measurement's
     * position. This update is the result of the change in object's conviction
     * pdf on its position which is calculated from the given filter model
     * and the current measurement. The filter is an implementation of
     * Kalman Filter.
     */
    template <class DerivedObject>
      void KalmanObject<DerivedObject>::
      update(const ObjectConstPtr& measurement)
      {
        ROS_DEBUG_STREAM_NAMED("KALMAN_OBJECT_UPDATE",
            "before measurement std dev = "
            << std::endl << "x : " << getStdDevX()
            << std::endl << "y : " << getStdDevY()
            << std::endl << "z : " << getStdDevZ());
        ROS_DEBUG_STREAM_NAMED("KALMAN_OBJECT_UPDATE",
            "before measurement probability = " << this->getProbability());
        Point measurementPosition = measurement->getPose().position;
        MatrixWrapper::ColumnVector newPosition(1);
        //!< Filter's input vector
        MatrixWrapper::ColumnVector input(1);
        //!< Input is 0.0 as our actions doesn't change the world model.
        input(1) = 0.0;

        //!< Updating existing object's filter pdfs.
        SystemModelPtrVector systemModels;
        systemModels = modelPtr_->getSystemModels();
        MeasurementModelPtrVector measurementModels;
        measurementModels = modelPtr_->getMeasurementModels();

        newPosition(1) = measurementPosition.x;
        filterX_->Update(systemModels[0].get(),
            input, measurementModels[0].get(), newPosition);

        newPosition(1) = measurementPosition.y;
        filterY_->Update(systemModels[1].get(),
            input, measurementModels[1].get(), newPosition);

        newPosition(1) = measurementPosition.z;
        filterZ_->Update(systemModels[2].get(),
            input, measurementModels[2].get(), newPosition);

        //!< Updating existing object's expected pose.
        Pose newObjectPose;
        newObjectPose.position.x = filterX_->PostGet()
          ->ExpectedValueGet()(1);
        newObjectPose.position.y = filterY_->PostGet()
          ->ExpectedValueGet()(1);
        newObjectPose.position.z = filterZ_->PostGet()
          ->ExpectedValueGet()(1);

        //!< Setting existing object's orientation.
        newObjectPose.orientation = this->pose_.orientation;

        this->pose_ = newObjectPose;

        //!< Updating object's probability.
        ROS_DEBUG_STREAM_NAMED("KALMAN_OBJECT_UPDATE",
            "after Measurement std dev = "
            << std::endl << "x : " << getStdDevX()
            << std::endl << "y : " << getStdDevY()
            << std::endl << "z : " << getStdDevZ());
        this->probability_ = (
            Utils::probabilityFromStdDev(this->distanceThres_, getStdDevX()) +
            Utils::probabilityFromStdDev(this->distanceThres_, getStdDevY()) +
            Utils::probabilityFromStdDev(this->distanceThres_, getStdDevZ())) / 3;
        ROS_DEBUG_STREAM_NAMED("KALMAN_OBJECT_UPDATE",
            "after Measurement probability = " << this->probability_);

        //!< Check if object has become a legitimate one.
        this->checkLegit();
      }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_KALMAN_OBJECT_H
