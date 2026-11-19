#pragma once

#include <rosbag2_storage/serialized_bag_message.hpp>

#include <QJsonDocument>
#include <QJsonObject>

#include "yaml-cpp/yaml.h"

#include <fstream>

namespace MessageFile
{
enum class FileFormat {
    YAML,
    JSON,
    CSV
};


inline std::optional<FileFormat>
fileFormatFromString(const std::string& formatString)
{
    if (formatString == "yaml") {
        return FileFormat::YAML;
    } else if (formatString == "json") {
        return FileFormat::JSON;
    } else if (formatString == "csv") {
        return FileFormat::CSV;
    }

    return std::nullopt;
}


inline std::string
fileFormatToString(FileFormat format)
{
    switch (format) {
    case FileFormat::YAML:
        return "yaml";
    case FileFormat::JSON:
        return "json";
    case FileFormat::CSV:
        return "csv";
    }

    return "";
}


// Write a single line for one or multiple csv files
inline void
writeCSVLine(std::ostream& stream, const std::vector<std::string>& fields)
{
    for (size_t i = 0; i < fields.size(); ++i) {
        if (i != 0) {
            stream << ',';
        }

        // Escape the field if it contains characters which would break the format
        const auto& field = fields.at(i);
        if (field.find_first_of(",\"\r\n") == std::string::npos) {
            stream << field;
            continue;
        }

        stream << '"';
        for (const auto character : field) {
            if (character == '"') {
                stream << "\"\"";
            } else {
                stream << character;
            }
        }
        stream << '"';
    }
    stream << '\n';
}


// Write bag message content previously converted to a csv compatible format
// to a string which can then be written to a file
inline std::string
csvContentToString(const std::vector<std::string>&               headers,
                   const std::vector<std::vector<std::string> >& rows)
{
    if (headers.empty()) {
        return "";
    }

    std::ostringstream stream;
    writeCSVLine(stream, headers);

    for (const auto& row : rows) {
        writeCSVLine(stream, row);
    }

    return stream.str();
}


inline std::vector<std::string>
csvValuesForHeaders(const std::vector<std::pair<std::string, std::string> >& content,
                    const std::vector<std::string>&                          headers)
{
    std::vector<std::string> values;
    values.reserve(headers.size());

    for (const auto& header : headers) {
        const auto value = std::find_if(content.begin(), content.end(), [&header] (const auto& entry) {
            return entry.first == header;
        });
        values.push_back(value != content.end() ? value->second : "");
    }

    return values;
}


using Message = rosbag2_storage::SerializedBagMessage;
using StringConverter = std::function<std::string (const Message&, const std::string&)>;
using CSVConverter = std::function<std::vector<std::pair<std::string, std::string> >(const Message&, const std::string&)>;

// Base writer for streaming all messages of a bag topic into a single output file
class SingleFileWriter
{
public:
    explicit SingleFileWriter(const std::string& filePath) :
        m_file(filePath)
    {
    }

    virtual
    ~SingleFileWriter() = default;

    virtual bool
    write(size_t             index,
          const Message&     message,
          const std::string& messageType) = 0;

    virtual bool
    finish() = 0;

    [[nodiscard]] bool
    isOpen() const
    {
        return m_file.is_open();
    }

protected:
    std::ofstream m_file;
};


class YAMLFileWriter : public SingleFileWriter
{
public:
    YAMLFileWriter(const std::string& filePath, StringConverter converter) :
        SingleFileWriter(filePath), m_convert(std::move(converter))
    {
        if (m_file.is_open()) {
            m_emitter = std::make_unique<YAML::Emitter>(m_file);
            *m_emitter << YAML::BeginMap;
        }
    }

    bool
    write(size_t             index,
          const Message&     message,
          const std::string& messageType) override
    {
        *m_emitter << YAML::Key << std::to_string(index) << YAML::Value << YAML::Load(m_convert(message, messageType));
        return m_emitter->good() && m_file.good();
    }

    bool
    finish() override
    {
        if (!m_emitter) {
            return false;
        }

        *m_emitter << YAML::EndMap;
        const auto succeeded = m_emitter->good();

        m_emitter.reset();
        m_file.close();

        return succeeded && !m_file.fail();
    }

private:
    StringConverter m_convert;
    std::unique_ptr<YAML::Emitter> m_emitter;
};


class JSONFileWriter : public SingleFileWriter
{
public:
    JSONFileWriter(const std::string& filePath, StringConverter converter) :
        SingleFileWriter(filePath), m_convert(std::move(converter))
    {
        if (m_file.is_open()) {
            m_file << "{\n";
        }
    }

    bool
    write(size_t             index,
          const Message&     message,
          const std::string& messageType) override
    {
        const auto messageObject = QJsonDocument::fromJson(QByteArray::fromStdString(m_convert(message, messageType))).object();
        if (!m_firstMessage) {
            m_file << ",\n";
        }

        m_file << "    \"" << index << "\": " << QJsonDocument(messageObject).toJson(QJsonDocument::Compact).toStdString();
        m_firstMessage = false;

        return m_file.good();
    }

    bool
    finish() override
    {
        m_file << (m_firstMessage ? "}\n" : "\n}\n");
        const auto succeeded = m_file.good();

        m_file.close();

        return succeeded && !m_file.fail();
    }

private:
    StringConverter m_convert;
    bool m_firstMessage = true;
};


class CSVFileWriter : public SingleFileWriter
{
public:
    CSVFileWriter(const std::string& filePath, CSVConverter converter) :
        SingleFileWriter(filePath), m_convert(std::move(converter))
    {
    }

    bool
    write(size_t,
          const Message&     message,
          const std::string& messageType) override
    {
        const auto csvContent = m_convert(message, messageType);

        if (!m_headerWritten) {
            for (const auto& csvEntry : csvContent) {
                m_headers.push_back(csvEntry.first);
            }
            writeCSVLine(m_file, m_headers);
            m_headerWritten = true;
        }

        writeCSVLine(m_file, csvValuesForHeaders(csvContent, m_headers));

        return m_file.good();
    }

    bool
    finish() override
    {
        m_file.close();

        return !m_file.fail();
    }

private:
    CSVConverter m_convert;
    std::vector<std::string> m_headers;
    bool m_headerWritten = false;
};
} // namespace MessageFile
