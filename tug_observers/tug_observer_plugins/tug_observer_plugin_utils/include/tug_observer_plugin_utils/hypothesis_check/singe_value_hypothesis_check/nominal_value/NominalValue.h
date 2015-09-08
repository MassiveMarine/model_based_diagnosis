//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_NOIMALVALUE_H
#define TUG_OBSERVER_PLUGINS_CPP_NOIMALVALUE_H

template <class T>
class NominalValue
{
public:
  virtual bool isNominal(const T& value) = 0;
};


#endif //TUG_OBSERVER_PLUGINS_CPP_NOIMALVALUE_H
