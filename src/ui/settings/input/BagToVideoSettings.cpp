#include "BagToVideoSettings.hpp"

BagToVideoSettings::BagToVideoSettings(Parameters::BagToVideoParameters& parameters, const QString& groupName) :
    VideoSettings(parameters, groupName)
{
    registerParameter("bw_images", parameters.useBWImages, false);
    registerParameter("lossless_images", parameters.lossless, false);

    read();
}
