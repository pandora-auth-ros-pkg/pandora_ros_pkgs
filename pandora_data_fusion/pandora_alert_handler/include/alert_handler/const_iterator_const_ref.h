// "Copyright [year] <Copyright Owner>"

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
