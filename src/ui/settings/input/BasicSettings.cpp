#include "BasicSettings.hpp"

BasicSettings::BasicSettings(Parameters::BasicParameters& parameters, const QString& groupName) :
    GeneralSettings(groupName)
{
    registerParameter("source_dir", parameters.sourceDirectory, QString(""));

    read();
}
