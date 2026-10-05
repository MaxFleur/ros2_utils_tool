#include "PlayBagSettings.hpp"

PlayBagSettings::PlayBagSettings(Parameters::PlayBagParameters& parameters,
                                 const QString&                 groupName) :
    SelectableBagContentSettings(parameters, groupName)
{
    registerParameter("rate", parameters.rate, 1.0);
    registerParameter("offset", parameters.offset, 0.0);
    registerParameter("loop", parameters.loop, false);
    registerParameter("publish_service_requests", parameters.publishServiceRequests, false);

    read();
}
