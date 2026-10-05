#include "SelectableBagContentSettings.hpp"

SelectableBagContentSettings::SelectableBagContentSettings(Parameters::SelectableBagContentParameters& parameters, const QString& groupName) :
    BasicSettings(parameters, groupName)
{
    const auto readItem = [] (QSettings& settings, Parameters::SelectableBagContent& item) {
        item.name = readParameter(settings, "name", QString(""));
        item.isSelected = readParameter(settings, "is_selected", true);
    };
    const auto writeItem = [] (QSettings& settings, const Parameters::SelectableBagContent& item) {
        writeParameter(settings, "name", item.name);
        writeParameter(settings, "is_selected", item.isSelected);
    };

    registerArrayParameter("services", parameters.services, readItem, writeItem);
    registerArrayParameter("topics", parameters.topics, readItem, writeItem);

    read();
}
