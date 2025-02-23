#include "EditBagSettings.hpp"

EditBagSettings::EditBagSettings(Utils::UI::EditBagParameters& parameters,
                                 const QString&                groupName) :
    AdvancedSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
EditBagSettings::write()
{
    if (!AdvancedSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);

    settings.beginWriteArray("topics");
    for (auto i = 0; i < m_parameters.topics.size(); ++i) {
        settings.setArrayIndex(i);
        setSettingsParameter(settings, m_parameters.topics.at(i).renamedTopicName, "renamed_name");
        setSettingsParameter(settings, m_parameters.topics.at(i).originalTopicName, "original_name");
        setSettingsParameter(settings, m_parameters.topics.at(i).lowerBoundary, "lower_boundary");
        setSettingsParameter(settings, m_parameters.topics.at(i).upperBoundary, "upper_boundary");
        setSettingsParameter(settings, m_parameters.topics.at(i).isSelected, "is_selected");
    }
    settings.endArray();
    setSettingsParameter(settings, m_parameters.deleteSource, "delete_source");
    setSettingsParameter(settings, m_parameters.updateTimestamps, "update_timestamps");

    settings.endGroup();
    return true;
}


bool
EditBagSettings::read()
{
    if (!AdvancedSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.topics.clear();

    const auto size = settings.beginReadArray("topics");
    for (auto i = 0; i < size; ++i) {
        settings.setArrayIndex(i);
        m_parameters.topics.append({ settings.value("renamed_name").toString(), settings.value("original_name").toString(),
                                     settings.value("lower_boundary").value<size_t>(), settings.value("upper_boundary").value<size_t>(),
                                     settings.value("is_selected").toBool() });
    }
    settings.endArray();
    m_parameters.deleteSource = settings.value("delete_source").isValid() ? settings.value("delete_source").toBool() : false;
    m_parameters.updateTimestamps = settings.value("update_timestamps").isValid() ? settings.value("update_timestamps").toBool() : false;

    settings.endGroup();
    return true;
}
