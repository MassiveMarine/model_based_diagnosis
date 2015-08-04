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

template<typename _T>
class GetValueHelper
{
public:
    static _T getValue(XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
        if(xml_value.getType() != type)
        {
            ROS_FATAL_STREAM(" has wrong type should have " << type << " and has " << xml_value.getType());
            std::stringstream error_stream;
            error_stream << " has wrong type should have " << type << " and has " << xml_value.getType();
            throw std::invalid_argument(error_stream.str());
        }

        return static_cast<_T>(xml_value);
    }
};

template<typename _T>
class GetValueHelper<std::vector<_T> >
{
    static _T getValue(XmlRpc::XmlRpcValue xml_value)
    {
        return GetValueHelper<_T>::getValue(TypeHelper<_T>::Type, xml_value);
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

    template<typename _T>
    static _T getValue(XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
        return GetValueHelper<_T>::getValue(type, xml_value);
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
};

#endif //TUG_OBSERVER_PLUGINS_CPP_PROCESSYAML_H
