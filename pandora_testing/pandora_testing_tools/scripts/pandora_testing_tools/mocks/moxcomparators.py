import mox
import bcolors
import types

class fieldIs(mox.Comparator):
  """Comparison class used to check identity, instead of equality."""

  def __init__(self, field , obj):
    self.obj_ = obj
    self._field = field

  def equals(self, rhs ):
    return getattr(rhs , self._field) is self.obj_

  def __repr__(self):
    return "< field %s is %r >" %  ( self._field ,  self.obj_)
    
class fieldEquals(mox.Comparator):
  """Comparison class used to check identity, instead of equality."""

  def __init__(self, field , obj):
    self.obj_ = obj
    self._field = field

  def equals(self, rhs ):
    return getattr(rhs , self._field) == self.obj_

  def __repr__(self):
    return "< field %s is %r >" %  ( self._field ,  self.obj_)
    
class msgEquals(mox.Comparator):
  """Comparison class used to check identity, instead of equality."""

  def __init__(self , obj):
    self.obj_ = obj
    self.trace = "\n"

  def recurse(self, fieldValue_, fieldValue, field):
    
    allCool = True
    
    tObj = type(fieldValue_)
    
    if tObj == types.ListType:
      for i,e in enumerate(fieldValue_):
        allCool = allCool and self.recurse(e,fieldValue[i],field+'[%d]'%(i,)) 
      return allCool
      
    try:
      instance_dict = fieldValue_.__slots__
    except AttributeError:
      return fieldValue_ == fieldValue
    
    for attr in instance_dict:
      allCool = allCool and self.recurse(getattr(fieldValue_,attr),getattr(fieldValue,attr),field+'.'+attr)

    return allCool

  def recurseRepr(self, fieldValue_, field):
    
    tObj = type(fieldValue_)
    
    if tObj == types.ListType:
      for i,e in enumerate(fieldValue_):
        self.recurseRepr(fieldValue_[i],field+'[%d]'%(i,)) 
      return 
      
    try:
      instance_dict = fieldValue_.__slots__
    except AttributeError:
      self.trace = self.trace + "Field %s is %r \n" %  ( field ,  fieldValue_)
      return 
    
    for attr in instance_dict:
      self.recurseRepr(getattr(fieldValue_,attr),field+'.'+attr)

  def equals(self, obj):
    return self.recurse(self.obj_, obj, '')

  def __repr__(self):
    #~ self.recurseRepr(self.obj_,'')
    return str(self.obj_)
    
class fieldGoalEquals(mox.Comparator):
  """Comparison class used to check identity, instead of equality."""

  def __init__(self, field , obj):
    self.obj_ = obj
    self._field = field

  def equals(self, rhs ):
    return getattr(rhs.goal , self._field) == self.obj_

  def __repr__(self):
    return "< field %s is %r >" %  ( self._field ,  self.obj_)
    
class Equals(mox.Comparator):
  """Comparison class used to check identity, instead of equality."""

  def __init__(self, obj):
    self.obj_ = obj

  def equals(self, rhs):
    return rhs == self.obj_

  def __repr__(self):
    return "<is %r (%s)>" % (self.obj_, id(self.obj_))


    
class fieldIsAlmost(mox.Comparator):
  def __init__(self, field , float_value, places=7):
    self._float_value = float_value
    self._places = places
    self._field = field

  def equals(self, rhs):
    rhs = getattr(rhs , self._field)
    self.error =  self._float_value - rhs  
    try:
      return round(rhs-self._float_value, self._places) == 0
    except Exception:
      return False

  def __repr__(self):
    return "< field %s is almost %r , error = %r >" %  ( self._field ,  self._float_value,self.error)
    

class listAnd(mox.Comparator):
  
  def __init__(self, args):
    self._comparators = args

  def equals(self, rhs):
    for comparator in self._comparators:
      if not comparator.equals(rhs):
        return False

    return True

  def __repr__(self):
    return bcolors.bColors.HEADER + '<AND %s>' % str(self._comparators) + bcolors.bColors.ENDC
