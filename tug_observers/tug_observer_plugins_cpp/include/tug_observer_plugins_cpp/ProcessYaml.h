//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_PROCESSYAML_H
#define TUG_OBSERVER_PLUGINS_CPP_PROCESSYAML_H


#include <ros/param.h>

template<typename _T>
class TypeHelper
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeInvalid;
};

template<>
class TypeHelper<bool>
{
public:
  static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeBoolean;
};

template<>
class TypeHelper<unsigned int>
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeInt;
};

template<>
class TypeHelper<int>
{
public:
  static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeInt;
};

template<>
class TypeHelper<unsigned long>
{
public:
  static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeInt;
};

template<>
class TypeHelper<long>
{
public:
  static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeInt;
};

template<>
class TypeHelper<std::string>
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeString;
};

template<>
class TypeHelper<double>
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeDouble;
};

template<typename _T>
class TypeHelper<std::vector<_T> >
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeArray;
};

template<typename _T, typename _R>
class CastHelper
{

public:
  static _T performCast(XmlRpc::XmlRpcValue xml_value)
  {
    return static_cast<_T>(static_cast<_R>(xml_value));
  }
};

template<typename _T>
class CastHelper<_T, unsigned int>
{

public:
  static _T performCast(XmlRpc::XmlRpcValue xml_value)
  {
    return static_cast<_T>(static_cast<int>(xml_value));
  }
};

template<typename _T>
class CastHelper<_T, unsigned long>
{

public:
  static _T performCast(XmlRpc::XmlRpcValue xml_value)
  {
    return static_cast<_T>(static_cast<int>(xml_value));
  }
};

template<typename _T>
class CastHelper<_T, long>
{

public:
  static _T performCast(XmlRpc::XmlRpcValue xml_value)
  {
    return static_cast<_T>(static_cast<int>(xml_value));
  }
};

template<typename _T, typename _R>
class GetValueHelper
{
  static std::string getTypeName(XmlRpc::XmlRpcValue::Type type)
  {
    switch (type)
    {
      case XmlRpc::XmlRpcValue::TypeInvalid:
        return "TypeInvalid";
      case XmlRpc::XmlRpcValue::TypeBoolean:
        return "TypeBoolean";
      case XmlRpc::XmlRpcValue::TypeInt:
        return "TypeInt";
      case XmlRpc::XmlRpcValue::TypeDouble:
        return "TypeDouble";
      case XmlRpc::XmlRpcValue::TypeString:
        return "TypeString";
      case XmlRpc::XmlRpcValue::TypeDateTime:
        return "TypeDateTime";
      case XmlRpc::XmlRpcValue::TypeBase64:
        return "TypeBase64";
      case XmlRpc::XmlRpcValue::TypeArray:
        return "TypeArray";
      case XmlRpc::XmlRpcValue::TypeStruct:
        return "TypeStruct";
      default:
        return "TypeInvalid";
    }
  }

public:
    static _T getValue(XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
        if(xml_value.getType() != type)
        {
            ROS_FATAL_STREAM("" << xml_value << " has wrong type should have " << getTypeName(type) << " and has " << getTypeName(xml_value.getType()));
            std::stringstream error_stream;
            error_stream << "" << xml_value << " has wrong type should have " << getTypeName(type) << " and has " << getTypeName(xml_value.getType());
            throw std::invalid_argument(error_stream.str());
        }

        return CastHelper<_T, _R>::performCast(xml_value);
    }
};

template<typename _T, typename _R>
class GetValueHelper<std::vector<_T>, std::vector<_R> >
{
    static _T getValue(XmlRpc::XmlRpcValue xml_value)
    {
        return GetValueHelper<_T, _R>::getValue(TypeHelper<_T>::Type, xml_value);
    }
public:
    static std::vector<_T> getValue(XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
        if(xml_value.getType() != type)
        {
            ROS_FATAL_STREAM(" has wrong type should have " << type << " and has " << xml_value.getType());
            std::stringstream error_stream;
            error_stream << " has wrong type should have " << type << " and has " << xml_value.getType();
            throw std::invalid_argument(error_stream.str());
        }

        std::vector<_T> result;
        for(int i = 0; i < xml_value.size(); ++i)
            result.push_back(getValue(xml_value[i]));

        return result;
    }
};

class ProcessYaml
{
public:

    static bool hasValue(std::string name, XmlRpc::XmlRpcValue xml_value)
    {
      return xml_value.hasMember(name);
    }

    template<typename _T>
    static _T getValue(XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
        return GetValueHelper<_T, _T>::getValue(type, xml_value);
    }

    template<typename _T>
    static _T getValue(XmlRpc::XmlRpcValue xml_value)
    {
        return getValue(TypeHelper<_T>::Type, xml_value);
    }

    template<typename _T>
    static _T getValue(std::string name, XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
        if(!xml_value.hasMember(name))
        {
            ROS_FATAL_STREAM(name << " is not a member of " << xml_value);
            std::stringstream error_stream;
            error_stream << name << " is not a member of " << xml_value;
            throw std::invalid_argument(error_stream.str());
        }

        return getValue<_T>(type, xml_value[name]);
    }

    template<typename _T>
    static _T getValue(std::string name, XmlRpc::XmlRpcValue xml_value)
    {
        return getValue<_T>(name, TypeHelper<_T>::Type, xml_value);
    }

    template<typename _T>
    static _T getValue(std::string name, XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value, _T default_value)
    {
        if(!xml_value.hasMember(name))
            return default_value;

        return getValue<_T>(name, type, xml_value);
    }

    template<typename _T>
    static _T getValue(std::string name, XmlRpc::XmlRpcValue xml_value, _T default_value)
    {
        return getValue<_T>(name, TypeHelper<_T>::Type, xml_value, default_value);
    }

    template<typename _T, typename _R>
    static _T getCastedValue(XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
        return GetValueHelper<_T, _R>::getValue(type, xml_value);
    }

  template<typename _T, typename _R>
    static _T getCastedValue(XmlRpc::XmlRpcValue xml_value)
    {
        return getCastedValue<_T, _R>(TypeHelper<_T>::Type, xml_value);
    }

  template<typename _T, typename _R>
    static _T getCastedValue(std::string name, XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
        if(!xml_value.hasMember(name))
        {
            ROS_FATAL_STREAM(name << " is not a member of " << xml_value);
            std::stringstream error_stream;
            error_stream << name << " is not a member of " << xml_value;
            throw std::invalid_argument(error_stream.str());
        }

        return getCastedValue<_T, _R>(type, xml_value[name]);
    }

  template<typename _T, typename _R>
    static _T getCastedValue(std::string name, XmlRpc::XmlRpcValue xml_value)
    {
        return getCastedValue<_T, _R>(name, TypeHelper<_T>::Type, xml_value);
    }

  template<typename _T, typename _R>
    static _T getCastedValue(std::string name, XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value, _T default_value)
    {
        if(!xml_value.hasMember(name))
            return default_value;

        return getCastedValue<_T, _R>(name, type, xml_value);
    }

  template<typename _T, typename _R>
    static _T getCastedValue(std::string name, XmlRpc::XmlRpcValue xml_value, _T default_value)
    {
        return getCastedValue<_T, _R>(name, TypeHelper<_T>::Type, xml_value, default_value);
    }
};

#endif //TUG_OBSERVER_PLUGINS_CPP_PROCESSYAML_H
