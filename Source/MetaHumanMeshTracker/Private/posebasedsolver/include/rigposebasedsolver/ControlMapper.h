// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once

#include <carbon/Common.h>
#include <map>
#include <string>
#include <algorithm>

namespace epic
{
    namespace rigposebasedsolver
    {
        /**
         * A simple struct which allows two rig UI controls to be mapped to single raw control and vice versa
        */
        struct ControlMapper
        {

            struct ControlMapRange
            {
                std::string controlName;
                std::pair<double, double> inputVal1Val2 = { 0.0, 1.0 };
                std::pair<double, double> outputVal1Val2 = { 0.0, 1.0 }; 
                static const unsigned version;

                double Map(double input) const
                {
                    if (inputVal1Val2.first < inputVal1Val2.second)
                    {
                        input = std::clamp(input, inputVal1Val2.first, inputVal1Val2.second);
                    }
                    else
                    {
                        input = std::clamp(input, inputVal1Val2.second, inputVal1Val2.first);
                    }
                    return outputVal1Val2.first + (input - inputVal1Val2.first) / (inputVal1Val2.second - inputVal1Val2.first) * (outputVal1Val2.second - outputVal1Val2.first);
                }

                double UnMap(double output) const
                {
                    if (outputVal1Val2.first < outputVal1Val2.second)
                    {
                        output = std::clamp(output, outputVal1Val2.first, outputVal1Val2.second);
                    }
                    else
                    {
                        output = std::clamp(output, outputVal1Val2.second, outputVal1Val2.first);
                    }   
                    return inputVal1Val2.first + (output - outputVal1Val2.first) / (outputVal1Val2.second - outputVal1Val2.first) * (inputVal1Val2.second - inputVal1Val2.first);
                }

                bool IsInInputRange(double value) const
                {
                    if (inputVal1Val2.first < inputVal1Val2.second)
                    {
                        return value > inputVal1Val2.first && value <= inputVal1Val2.second;
                    }

                    return value > inputVal1Val2.second && value <= inputVal1Val2.first;
                }

                friend void serialize(const ControlMapRange& item, std::ostream& out);
                friend void deserialize(ControlMapRange& item, std::istream& in);

            };

            ControlMapRange fromControlMapRange1;
            ControlMapRange fromControlMapRange2;
            std::string toName;
            double toDefault;

            /*
            * @returns true if the control name matches one of the 'from' names
            */
            bool MatchesFromControlName(const std::string& controlName) const
            {
                return fromControlMapRange1.controlName == controlName || fromControlMapRange2.controlName == controlName;
            }

            /*
            * @return the max value of the 'to' range
            */
            double ToMax() const
            {
                return std::max({ fromControlMapRange1.outputVal1Val2.first, fromControlMapRange1.outputVal1Val2.second,
                    fromControlMapRange2.outputVal1Val2.first, fromControlMapRange2.outputVal1Val2.second } );
            }

            /*
            * @return the min value of the 'to' range
            */
            double ToMin() const
            {
                return std::min({ fromControlMapRange1.outputVal1Val2.first, fromControlMapRange1.outputVal1Val2.second,
                    fromControlMapRange2.outputVal1Val2.first, fromControlMapRange2.outputVal1Val2.second });
            }


            /*
            *  Map input UI controls to an output raw control. Remove the original UI 'from' controls from controlValues and replace with the 'to' control and value
            *  Special note, values not in the input range (between but not including Val1 and to and including Val2) will be set to default. If Val1 is not equal to
            *  the default the user should extend the range slightly so Val1 will never be hit.
            *  @param[in/out] controlValues: a map of control name to control values, which should contain the 'from' controls
            *  @returns true if successful maps (from controls found) and false otherwise
            */
            bool Map(std::map< std::string, double>& controlValues) const
            {                
                auto it = controlValues.find(fromControlMapRange1.controlName);
                if (it == controlValues.end())
                {
                    return false;
                }
                double value = toDefault;

                if (fromControlMapRange1.IsInInputRange(it->second))
                {
                    value = fromControlMapRange1.Map(it->second);
                }

                controlValues.erase(it);

                it = controlValues.find(fromControlMapRange2.controlName);
                if (it == controlValues.end())
                {
                    return false;
                }

                if (fromControlMapRange2.IsInInputRange(it->second))
                {
                    value = fromControlMapRange2.Map(it->second);
                }

                controlValues[toName] = value;
                controlValues.erase(it);

                return true;
            }

            /*
            *  UnMap an output raw control to corresponding input UI controls. Remove the original raw 'to' controls from controlValues and replace with the 'from' UI controls and values
            *  @param[in/out] controlValues: a map of control name to control values, which should contain the 'to' control
            *  @returns true if successful maps (to control found) and false otherwise
            */
            bool UnMap(std::map< std::string, double>& controlValues) const
            {
                auto it = controlValues.find(toName);
                if (it == controlValues.end())
                {
                    return false;
                }
                double value = it->second;
                controlValues.erase(it);

                controlValues[fromControlMapRange1.controlName] = fromControlMapRange1.UnMap(value);
                controlValues[fromControlMapRange2.controlName] = fromControlMapRange2.UnMap(value);

                return true;
            }
    

            /*
                friend function for serialization
            */
            friend void serialize(const ControlMapper& item, std::ostream& out);

            /*
                friend function for deserialization
            */
            friend void deserialize(ControlMapper& item, std::istream& in);

            static const unsigned version;
        };
    }
}

