#include "VideoSettings.hpp"

VideoSettings::VideoSettings(Parameters::VideoParameters& parameters, const QString& groupName) :
    RGBSettings(parameters, groupName)
{
    registerParameter("fps", parameters.fps, 30);

    read();
}
