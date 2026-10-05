#pragma once

#include <QSettings>

template<typename T>
concept GeneralSettingsParameter = std::same_as<T, int> || std::same_as<T, unsigned int> || std::same_as<T, size_t> ||
                                   std::same_as<T, bool> || std::same_as<T, double> || std::same_as<T, QString>;

// Basic settings, from which all other settings derive
// Each setting class registers all of its parameters in its constructor.
// Reading (done automatically in each ctor) and writing is then handled generically by walking the registered parameters
class GeneralSettings {
public:
    GeneralSettings(const QString& groupName, bool checkSaveGate = true) :
        m_groupName(groupName), m_checkSaveGate(checkSaveGate)
    {
    }

    bool
    write()
    {
        return mainReadWriteOperation(true);
    }

protected:
    // Register a simple parameter, stored directly under its identifier
    template<typename T>
    requires GeneralSettingsParameter<T>
    void
    registerParameter(const QString& identifier,
                      T&             parameter,
                      T              defaultValue)
    {
        m_parameters.push_back(std::make_unique<ParameterEntryImpl<T> >(identifier, parameter, defaultValue));
    }

    // Register a parameter that is stored as an array, one array entry per element.
    template<typename T, typename ReadItemFunction, typename WriteItemFunction>
    void
    registerArrayParameter(const QString&    identifier,
                           QVector<T>&       items,
                           ReadItemFunction  readItem,
                           WriteItemFunction writeItem)
    {
        m_parameters.push_back(std::make_unique<ArrayParameterEntry<T> >(identifier, items, std::move(readItem), std::move(writeItem)));
    }

    // Read all registered parameters, called automatically in each ctor
    bool
    read()
    {
        return mainReadWriteOperation(false);
    }

    // Use predefined settings to write
    template<typename T>
    requires GeneralSettingsParameter<T>
    static void
    writeParameter(QSettings&     settings,
                   const QString& identifier,
                   T              parameter)
    {
        if (const auto& storedParameter = settings.value(identifier); storedParameter.isValid() && storedParameter.value<T>() == parameter) {
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

    // Read based on stored type, using predefined settings
    template<typename T>
    requires GeneralSettingsParameter<T>
    static T
    readParameter(QSettings&     settings,
                  const QString& identifier,
                  T              defaultValue)
    {
        T value;
        if constexpr (std::is_same_v<T, int> || std::is_same_v<T, unsigned int>) {
            value = settings.value(identifier).isValid() ? settings.value(identifier).toInt() : defaultValue;
        } else if constexpr (std::is_same_v<T, size_t>) {
            const auto& storedParameter = settings.value(identifier);
            value = storedParameter.isValid() ? storedParameter.value<size_t>() : defaultValue;
        } else if constexpr (std::is_same_v<T, bool>) {
            value = settings.value(identifier).isValid() ? settings.value(identifier).toBool() : defaultValue;
        } else if constexpr (std::is_same_v<T, double>) {
            value = settings.value(identifier).isValid() ? settings.value(identifier).toDouble() : defaultValue;
        } else {
            value = settings.value(identifier).isValid() ? settings.value(identifier).toString() : defaultValue;
        }

        return value;
    }

private:
    // Main operation which writes to or loads a parameter from file.
    bool
    mainReadWriteOperation(bool write);

    struct ParameterEntry {
        virtual
        ~ParameterEntry() = default;

        virtual void
        write(QSettings& settings) const = 0;

        virtual void
        read(QSettings& settings) = 0;
    };

    // Standard parameter, stores a single identifier, value and default.
    // Writes to and (re)reads the parameter from file.
    template<typename T>
    struct ParameterEntryImpl : ParameterEntry {
        ParameterEntryImpl(const QString& identifier,
                           T&             parameter,
                           T              defaultValue) :
            m_identifier(std::move(identifier)), m_parameter(parameter), m_defaultValue(defaultValue)
        {
        }

        void
        write(QSettings& settings) const override
        {
            writeParameter(settings, m_identifier, m_parameter);
        }

        void
        read(QSettings& settings) override
        {
            m_parameter = readParameter(settings, m_identifier, m_defaultValue);
        }

        QString m_identifier;
        T&      m_parameter;
        T       m_defaultValue;
    };

    // Some settings (e.g. dummy bag) write arrays of topics, need to handle them separately
    template<typename T>
    struct ArrayParameterEntry : ParameterEntry {
        ArrayParameterEntry(const QString&                            identifier,
                            QVector<T>&                               items,
                            std::function<void(QSettings&, T&)>       readFunction,
                            std::function<void(QSettings&, const T&)> writeFunction) :
            m_items(items), m_identifier(std::move(identifier)),
            m_readFunction(std::move(readFunction)), m_writeFunction(std::move(writeFunction))
        {
        }

        void
        write(QSettings& settings) const override
        {
            settings.remove(m_identifier);

            settings.beginWriteArray(m_identifier);
            for (auto i = 0; i < m_items.size(); ++i) {
                settings.setArrayIndex(i);
                m_writeFunction(settings, m_items.at(i));
            }
            settings.endArray();
        }

        void
        read(QSettings& settings) override
        {
            m_items.clear();

            const auto size = settings.beginReadArray(m_identifier);
            for (auto i = 0; i < size; ++i) {
                settings.setArrayIndex(i);
                T item = {};
                m_readFunction(settings, item);
                m_items.append(std::move(item));
            }
            settings.endArray();
        }

        QVector<T>&                               m_items;
        QString                                   m_identifier;

        std::function<void(QSettings&, T&)>       m_readFunction;
        std::function<void(QSettings&, const T&)> m_writeFunction;
    };

private:
    std::vector<std::unique_ptr<ParameterEntry> > m_parameters;

    const QString m_groupName;
    // All settings excluding dialog settings can be ignored, use this as a safeguard
    const bool m_checkSaveGate;
};
