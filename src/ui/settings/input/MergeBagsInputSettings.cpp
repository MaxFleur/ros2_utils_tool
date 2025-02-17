#include "MergeBagsInputSettings.hpp"

MergeBagsInputSettings::MergeBagsInputSettings(Utils::UI::MergeBagsInputParameters& parameters,
                                               const QString&                       groupName) :
    AdvancedInputSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
MergeBagsInputSettings::write()
{
    if (!AdvancedInputSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);

    settings.beginWriteArray("topics");
    for (auto i = 0; i < m_parameters.topics.size(); ++i) {
        settings.setArrayIndex(i);
        setSettingsParameter(settings, m_parameters.topics.at(i).name, "name");
        setSettingsParameter(settings, m_parameters.topics.at(i).bagDir, "dir");
        setSettingsParameter(settings, m_parameters.topics.at(i).isSelected, "is_selected");
    }
    settings.endArray();
    setSettingsParameter(settings, m_parameters.secondSourceDirectory, "second_source");

    settings.endGroup();
    return true;
}


bool
MergeBagsInputSettings::read()
{
    if (!AdvancedInputSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.topics.clear();

    const auto size = settings.beginReadArray("topics");
    for (auto i = 0; i < size; ++i) {
        settings.setArrayIndex(i);
        m_parameters.topics.append({ settings.value("name").toString(), settings.value("dir").toString(), settings.value("is_selected").toBool() });
    }
    settings.endArray();
    m_parameters.secondSourceDirectory = settings.value("second_source").isValid() ? settings.value("second_source").toString() : "";

    settings.endGroup();
    return true;
}
