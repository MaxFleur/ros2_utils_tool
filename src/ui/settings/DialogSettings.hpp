#pragma once

#include "GeneralSettings.hpp"
#include "Parameters.hpp"

template<typename T>
concept DialogSettingsParameter = std::same_as<T, unsigned int> || std::same_as<T, bool>;

// Settings modified from settings dialog
class DialogSettings : public GeneralSettings {
public:
    DialogSettings(Parameters::DialogParameters& parameters,
                   const QString&                groupName);

    // Make this static because we need to access the variable from many different places
    // in the application without wanting to use this as extra dependency
    template<typename T>
    requires DialogSettingsParameter<T>
    static T
    getStaticParameter(const QString& identifier,
                       T              defaultValue)
    {
        QSettings settings;
        settings.beginGroup("dialog");
        T staticParameter;

        if constexpr (std::is_same_v<T, unsigned int>) {
            staticParameter = settings.value(identifier).isValid() ? settings.value(identifier).toInt() : defaultValue;
        } else {
            staticParameter = settings.value(identifier).isValid() ? settings.value(identifier).toBool() : defaultValue;
        }

        settings.endGroup();
        return staticParameter;
    }

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::DialogParameters& m_parameters;
};
