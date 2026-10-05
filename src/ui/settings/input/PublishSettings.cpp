#include "PublishSettings.hpp"

PublishSettings::PublishSettings(Parameters::PublishParameters& parameters, const QString& groupName) :
    VideoSettings(parameters, groupName)
{
    registerParameter("loop", parameters.loop, false);
    registerParameter("scale", parameters.scale, false);
    registerParameter("width", parameters.width, 1280);
    registerParameter("height", parameters.height, 720);

    read();
}
