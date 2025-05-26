#pragma once

#include <QSettings>

template<typename T>
concept GeneralSettingsParameter = std::same_as<T, int> || std::same_as<T, unsigned int> || std::same_as<T, size_t> ||
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
    // Variant without settings parameter
    template<typename T>
    requires GeneralSettingsParameter<T>
    void
    writeParameter(const QString& groupName,
                   const QString& identifier,
                   T              parameter) const
    {
        QSettings settings;
        settings.beginGroup(groupName);

        writeParameter(settings, identifier, parameter);
        settings.endGroup();
    }

    // Use predefined settings to write
    template<typename T>
    requires GeneralSettingsParameter<T>
    void
    writeParameter(QSettings&     settings,
                   const QString& identifier,
                   T              parameter) const
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

    // Read without settings parameter
    template<typename T>
    requires GeneralSettingsParameter<T>
    T
    readParameter(const QString& groupName,
                  const QString& identifier,
                  T              defaultValue) const
    {
        QSettings settings;
        settings.beginGroup(groupName);

        const auto parameter = readParameter(settings, identifier, defaultValue);
        settings.endGroup();

        return parameter;
    }

    // Read based on stored type, using predefined settings
    template<typename T>
    requires GeneralSettingsParameter<T>
    T
    readParameter(QSettings&     settings,
                  const QString& identifier,
                  T              defaultValue) const
    {
        T value;
        if constexpr (std::is_same_v<T, int> || std::is_same_v<T, unsigned int> ) {
            value = settings.value(identifier).isValid() ? settings.value(identifier).toInt() : defaultValue;
        } else if constexpr (std::is_same_v<T, size_t>) {
            value = settings.value(identifier).value<size_t>();
        } else if constexpr (std::is_same_v<T, bool>) {
            value = settings.value(identifier).isValid() ? settings.value(identifier).toBool() : defaultValue;
        } else {
            value = settings.value(identifier).isValid() ? settings.value(identifier).toString() : defaultValue;
        }

        return value;
    }

protected:
    const QString m_groupName;
};
