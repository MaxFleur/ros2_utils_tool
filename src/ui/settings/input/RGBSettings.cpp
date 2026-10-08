#include "RGBSettings.hpp"

RGBSettings::RGBSettings(Parameters::RGBParameters& parameters, const QString& groupName) :
    AdvancedSettings(parameters, groupName)
{
    registerParameter("switch_red_blue", parameters.exchangeRedBlueValues, false);

    read();
}
