#pragma once

#include "RGBSettings.hpp"

// Store bag to image tool parameters
class BagToImagesSettings : public RGBSettings {
public:
    BagToImagesSettings(Parameters::BagToImagesParameters& parameters,
                        const QString&                     groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::BagToImagesParameters& m_parameters;
};
