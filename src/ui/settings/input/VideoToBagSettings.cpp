#include "VideoToBagSettings.hpp"

VideoToBagSettings::VideoToBagSettings(Parameters::VideoToBagParameters& parameters,
                                       const QString&                    groupName) :
    VideoSettings(parameters, groupName)
{
    registerParameter("use_compression", parameters.useCompression, false);
    registerParameter("custom_fps", parameters.useCustomFPS, false);
    registerParameter("is_compression_jpeg", parameters.isCompressionJPEG, true);

    read();
}
