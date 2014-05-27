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

#ifndef ALERT_HANDLER_CONST_ITERATOR_CONST_REF_H
#define ALERT_HANDLER_CONST_ITERATOR_CONST_REF_H

#include <boost/iterator/iterator_adaptor.hpp>

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    template <class Iterator, class Value, class ConstReference>
    class const_iterator_const_ref
      : public boost::iterator_adaptor<
        const_iterator_const_ref< Iterator, Value, ConstReference >  // Derived
        , Iterator                                                     // Base
        , Value                                                        // Value
        , boost::use_default                               // CategoryOrTraversal
        , const ConstReference&                            // Reference
        >
    {
      private:

        struct enabler {};  // a private type avoids misuse

      public:

        const_iterator_const_ref()
          : const_iterator_const_ref::iterator_adaptor_(Iterator()) {}

        explicit const_iterator_const_ref(Iterator p)
          : const_iterator_const_ref::iterator_adaptor_(p) {}

        template <class OtherIterator, class OtherValue, class OtherReference>
          const_iterator_const_ref(
              const_iterator_const_ref<OtherIterator, OtherValue, OtherReference> const& other
              , typename boost::enable_if<
              boost::is_convertible<Iterator, OtherIterator>
              , enabler
              >::type = enabler()
              , typename boost::enable_if<
              boost::is_convertible<Value, OtherValue>
              , enabler
              >::type = enabler()
              , typename boost::enable_if<
              boost::is_convertible<ConstReference, OtherReference>
              , enabler
              >::type = enabler()
              )
          : const_iterator_const_ref::iterator_adaptor_(other.base()) {}

      private:

        friend class boost::iterator_core_access;

    };

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_CONST_ITERATOR_CONST_REF_H
