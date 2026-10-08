#include "DeleteSourceSettings.hpp"

DeleteSourceSettings::DeleteSourceSettings(Parameters::DeleteSourceParameters& parameters,
                                           const QString&                      groupName) :
    AdvancedSettings(parameters, groupName)
{
    registerParameter("delete_source", parameters.deleteSource, false);
    registerParameter("compress_per_message", parameters.compressPerMessage, false);

    read();
}
