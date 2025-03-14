#pragma once

#include <QSettings>

template<typename T>
concept GeneralSettingsParameter = std::same_as<T, int> || std::same_as<T, size_t> ||
                                   std::same_as<T, bool> || std::same_as<T, QString>;

// Basic settings, from which all other settings derive
// Each setting as write and read functions. Read functions are called automatically
// in the ctor, while the writing is called every time a parameter is changed.
// Settings for the input widgets follow the parameter hierarchy structure defined in Utils/UI
class GeneralSettings {
public:
    GeneralSettings(const QString& groupName) : m_groupName(groupName)
    {
    }

    virtual bool
    write() = 0;

protected:
    virtual bool
    read() = 0;

    // Called whenever we want to write values
    template<typename T>
    requires GeneralSettingsParameter<T>
    void
    setSettingsParameter(QSettings&     settings,
                         T              parameter,
                         const QString& identifier)
    {
        if (settings.value(identifier).value<T>() == parameter) {
            return;
        }
        // Simple conversion between size_t and QVariant is not possible
        if constexpr (std::is_same_v<T, size_t>) {
            QVariant v;
            v.setValue(parameter);
            settings.setValue(identifier, v);
        } else {
            settings.setValue(identifier, parameter);
        }
    }

protected:
    const QString m_groupName;
};
