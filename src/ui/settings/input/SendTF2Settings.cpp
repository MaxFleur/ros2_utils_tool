#include "SendTF2Settings.hpp"

SendTF2Settings::SendTF2Settings(Parameters::SendTF2Parameters& parameters,
                                 const QString&                 groupName) :
    BasicSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
SendTF2Settings::write()
{
    if (!BasicSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "translation_x", m_parameters.translation[0]);
    writeParameter(m_groupName, "translation_y", m_parameters.translation[1]);
    writeParameter(m_groupName, "translation_z", m_parameters.translation[2]);
    writeParameter(m_groupName, "rotation_x", m_parameters.rotation[0]);
    writeParameter(m_groupName, "rotation_y", m_parameters.rotation[1]);
    writeParameter(m_groupName, "rotation_z", m_parameters.rotation[2]);
    writeParameter(m_groupName, "rotation_w", m_parameters.rotation[3]);
    writeParameter(m_groupName, "name", m_parameters.childFrameName);
    writeParameter(m_groupName, "rate", m_parameters.rate);
    writeParameter(m_groupName, "is_static", m_parameters.isStatic);

    return true;
}


bool
SendTF2Settings::read()
{
    if (!BasicSettings::read()) {
        return false;
    }

    m_parameters.translation[0] = readParameter(m_groupName, "translation_x", 0.0);
    m_parameters.translation[1] = readParameter(m_groupName, "translation_y", 0.0);
    m_parameters.translation[2] = readParameter(m_groupName, "translation_z", 0.0);
    m_parameters.rotation[0] = readParameter(m_groupName, "rotation_x", 0.0);
    m_parameters.rotation[1] = readParameter(m_groupName, "rotation_y", 0.0);
    m_parameters.rotation[2] = readParameter(m_groupName, "rotation_z", 0.0);
    m_parameters.rotation[3] = readParameter(m_groupName, "rotation_w", 0.0);
    m_parameters.childFrameName = readParameter(m_groupName, "name", QString("tf_test"));
    m_parameters.rate = readParameter(m_groupName, "rate", 1);
    m_parameters.isStatic = readParameter(m_groupName, "is_static", true);

    return true;
}
