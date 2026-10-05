#include "DummyBagSettings.hpp"

DummyBagSettings::DummyBagSettings(Parameters::DummyBagParameters& parameters, const QString& groupName) :
    BasicSettings(parameters, groupName)
{
    const auto readParameters = [] (QSettings& settings, Parameters::DummyBagParameters::DummyBagTopic& topic) {
        topic.name = readParameter(settings, "name", QString(""));
        topic.type = readParameter(settings, "type", QString(""));
    };
    const auto writeParameters = [] (QSettings& settings, const Parameters::DummyBagParameters::DummyBagTopic& topic) {
        writeParameter(settings, "name", topic.name);
        writeParameter(settings, "type", topic.type);
    };

    registerArrayParameter("topics", parameters.topics, readParameters, writeParameters);
    registerParameter("msg_count", parameters.messageCount, 100);
    registerParameter("rate", parameters.rate, 10);
    registerParameter("use_custom_rate", parameters.useCustomRate, false);

    read();
}
