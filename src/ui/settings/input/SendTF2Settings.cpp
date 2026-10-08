#include "SendTF2Settings.hpp"

SendTF2Settings::SendTF2Settings(Parameters::SendTF2Parameters& parameters,
                                 const QString&                 groupName) :
    BasicSettings(parameters, groupName)
{
    registerParameter("translation_x", parameters.translation[0], 0.0);
    registerParameter("translation_y", parameters.translation[1], 0.0);
    registerParameter("translation_z", parameters.translation[2], 0.0);
    registerParameter("rotation_x", parameters.rotation[0], 0.0);
    registerParameter("rotation_y", parameters.rotation[1], 0.0);
    registerParameter("rotation_z", parameters.rotation[2], 0.0);
    registerParameter("rotation_w", parameters.rotation[3], 0.0);
    registerParameter("name", parameters.childFrameName, QString("tf_test"));
    registerParameter("rate", parameters.rate, 1);
    registerParameter("is_static", parameters.isStatic, true);

    read();
}
