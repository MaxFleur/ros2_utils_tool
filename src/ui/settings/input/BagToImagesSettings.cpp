#include "BagToImagesSettings.hpp"

BagToImagesSettings::BagToImagesSettings(Parameters::BagToImagesParameters& parameters, const QString& groupName) :
    RGBSettings(parameters, groupName)
{
    registerParameter("format", parameters.format, QString("jpg"));
    registerParameter("quality", parameters.quality, 8);
    registerParameter("bw_images", parameters.useBWImages, false);
    registerParameter("jpg_optimize", parameters.jpgOptimize, false);
    registerParameter("png_bilevel", parameters.pngBilevel, false);

    read();
}
