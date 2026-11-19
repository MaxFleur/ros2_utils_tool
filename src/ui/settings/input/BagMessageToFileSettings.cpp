#include "BagMessageToFileSettings.hpp"

BagMessageToFileSettings::BagMessageToFileSettings(Parameters::BagMessageToFileParameters& parameters,
                                                   const QString&                          groupName) :
    AdvancedSettings(parameters, groupName)
{
    registerParameter("write_single_output_file", parameters.writeSingleOutputFile, true);
    registerParameter("format", parameters.format, QString("yaml"));

    read();
}
