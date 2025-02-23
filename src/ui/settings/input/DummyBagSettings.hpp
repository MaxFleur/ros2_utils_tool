#pragma once

#include "BasicSettings.hpp"

// Store dummy bag creation parameters
class DummyBagSettings : public BasicSettings {
public:
    DummyBagSettings(Utils::UI::DummyBagParameters& parameters,
                     const QString&                 groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::DummyBagParameters& m_parameters;
};
