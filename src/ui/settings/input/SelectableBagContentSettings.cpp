#include "SelectableBagContentSettings.hpp"

SelectableBagContentSettings::SelectableBagContentSettings(Parameters::SelectableBagContentParameters& parameters, const QString& groupName) :
    BasicSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
SelectableBagContentSettings::write()
{
    if (!BasicSettings::write()) {
        return false;
    }

    QSettings settings;
    const auto writeArraySettings = [this, &settings] (const QString& settingID, const QVector<Parameters::SelectableBagContent>& paramsVector) {
        settings.remove(settingID);
        settings.beginWriteArray(settingID);
        for (auto i = 0; i < paramsVector.size(); ++i) {
            settings.setArrayIndex(i);
            writeParameter(settings, "name", paramsVector.at(i).name);
            writeParameter(settings, "is_selected", paramsVector.at(i).isSelected);
        }
        settings.endArray();
    };

    settings.beginGroup(m_groupName);
    writeArraySettings("services", m_parameters.services);
    writeArraySettings("topics", m_parameters.topics);
    settings.endGroup();

    return true;
}


bool
SelectableBagContentSettings::read()
{
    if (!BasicSettings::read()) {
        return false;
    }

    QSettings settings;
    const auto readArraySettings = [this, &settings] (const QString& settingID, QVector<Parameters::SelectableBagContent>& paramsVector) {
        const auto size = settings.beginReadArray(settingID);
        for (auto i = 0; i < size; ++i) {
            settings.setArrayIndex(i);
            paramsVector.append({ { readParameter(settings, "name", QString("")) }, readParameter(settings, "is_selected", true) });
        }
        settings.endArray();
    };

    m_parameters.services.clear();
    m_parameters.topics.clear();
    settings.beginGroup(m_groupName);
    readArraySettings("services", m_parameters.services);
    readArraySettings("topics", m_parameters.topics);
    settings.endGroup();

    return true;
}
