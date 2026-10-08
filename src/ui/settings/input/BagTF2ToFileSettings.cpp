#include "BagTF2ToFileSettings.hpp"

BagTF2ToFileSettings::BagTF2ToFileSettings(Parameters::BagTF2ToFileParameters& parameters,
                                           const QString&                      groupName) :
    AdvancedSettings(parameters, groupName)
{
    registerParameter("keep_timestamps", parameters.keepTimestamps, true);
    registerParameter("compact_output", parameters.compactOutput, true);

    read();
}
