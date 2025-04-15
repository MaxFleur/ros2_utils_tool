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

    // Make these static because we need to access and modify some values from many different places
    // without having to use the entire dialog parameter instance
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

    template<typename T>
    requires DialogSettingsParameter<T>
    static void
    writeStaticParameter(const QString& identifier,
                         T              value)
    {
        QSettings settings;
        settings.beginGroup("dialog");

        if (settings.value(identifier).value<T>() == value) {
            return;
        }

        settings.setValue(identifier, value);
    }

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::DialogParameters& m_parameters;
};
