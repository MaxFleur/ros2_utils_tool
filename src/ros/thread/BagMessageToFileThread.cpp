// ROS2 offers functions to convert any type of ROS2 message to yaml, including custom messages.
// However, these functions are only available in the Python implementation, not in C++.
// Thus, we use CPython here to access the python functions underneath

// CPython and Qt use equal macro names (e.g. "slot"). To avoid collisions,
// declare the Python header before everything else
#include <Python.h>

#include "BagMessageToFileThread.hpp"
#include "MessageFileWriter.hpp"

#include "UtilsGeneral.hpp"
#include "UtilsROS.hpp"

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <fstream>

namespace
{
// Write a bag message's processed content to file
bool
writeFileContent(const std::string& filePath, const std::string& content)
{
    std::ofstream file(filePath);
    if (!file.is_open()) {
        return false;
    }

    file << content;
    file.close();

    return !file.fail();
}


// Instance which starts the embedded Python interpreter. Slow as hell, so make sure to initialize only once per process.
class EmbeddedPython
{
public:
    // CPython does not support being finalized and re-initialized within the same process,
    // so the instance is intentionally never destroyed
    static EmbeddedPython&
    instance()
    {
        static auto* const embeddedPython = new EmbeddedPython();
        return *embeddedPython;
    }

private:
    EmbeddedPython()
    {
        Py_Initialize();
        // Release the GIL so that any thread (also from later runs) can attach to the interpreter via PyGILState_Ensure()
        PyEval_SaveThread();
    }
};

// Wrapper around PyObject* which decreases the reference count on destruction
class PyObjectRef
{
public:
    explicit PyObjectRef(PyObject* object = nullptr) : m_pythonObject(object)
    {
    }

    ~PyObjectRef()
    {
        Py_XDECREF(m_pythonObject);
    }

    PyObjectRef(const PyObjectRef&) = delete;
    PyObjectRef(PyObjectRef&& other) noexcept : m_pythonObject(other.m_pythonObject)
    {
        other.m_pythonObject = nullptr;
    }

    PyObjectRef&
    operator =(PyObjectRef&& other) noexcept
    {
        if (this != &other) {
            Py_XDECREF(m_pythonObject);
            m_pythonObject = other.m_pythonObject;
            other.m_pythonObject = nullptr;
        }
        return *this;
    }

    PyObject*
    get() const
    {
        return m_pythonObject;
    }

    operator PyObject*() const {
        return m_pythonObject;
    }

private:
    PyObject* m_pythonObject;
};

// Converts serialized ROS2 bag messages into YAML, JSON or CSV by calling
// rosidl_runtime_py.convert functions from the embedded CPython interpreter
class MessageConverterBridge
{
public:
    MessageConverterBridge()
    {
        EmbeddedPython::instance();

        const auto gilState = PyGILState_Ensure();

        m_deserializeMessage = PyObjectRef(PyObject_GetAttrString(PyImport_ImportModule("rclpy.serialization"), "deserialize_message"));
        m_getMessage = PyObjectRef(PyObject_GetAttrString(PyImport_ImportModule("rosidl_runtime_py.utilities"), "get_message"));
        m_messageToYaml = PyObjectRef(PyObject_GetAttrString(PyImport_ImportModule("rosidl_runtime_py.convert"), "message_to_yaml"));
        m_messageToOrderedDict = PyObjectRef(PyObject_GetAttrString(PyImport_ImportModule("rosidl_runtime_py.convert"), "message_to_ordereddict"));
        m_jsonDumps = PyObjectRef(PyObject_GetAttrString(PyImport_ImportModule("json"), "dumps"));
        // No standard ROS python function for this, need to declare it ourselves
        m_csvModule = PyObjectRef(PyModule_New("_ros2_utils_tool_csv"));
        const auto moduleDictionary = PyModule_GetDict(m_csvModule.get());

        // CSV output needs a flat representation of the message
        const PyObjectRef flattenDefinition(PyRun_String(R"(
import json

def flatten_message(value, prefix=""):
    items = []
    if isinstance(value, dict):
        for key, child in value.items():
            child_prefix = f"{prefix}.{key}" if prefix else str(key)
            items.extend(flatten_message(child, child_prefix))
    elif isinstance(value, (list, tuple)):
        items.append((prefix, json.dumps(value)))
    elif isinstance(value, bool):
        items.append((prefix, "true" if value else "false"))
    elif value is None:
        items.append((prefix, ""))
    else:
        items.append((prefix, str(value)))
    return items
)", Py_file_input, moduleDictionary, moduleDictionary));
        m_flattenMessage = PyObjectRef(PyObject_GetAttrString(m_csvModule.get(), "flatten_message"));

        PyGILState_Release(gilState);
    }

    ~MessageConverterBridge()
    {
        // Python objects have to be released while the GIL is held
        const auto gilState = PyGILState_Ensure();

        m_messageClasses.clear();
        m_flattenMessage = PyObjectRef();
        m_csvModule = PyObjectRef();

        PyGILState_Release(gilState);
    }

    // Converts one serialized message from a bag
    std::string
    toYaml(const rosbag2_storage::SerializedBagMessage& message,
           const std::string&                           messageType)
    {
        std::string result;
        const auto gilState = PyGILState_Ensure();

        // Scoped, so that the reference counts of all temporary Python objects are decreased while the GIL is still held
        {
            // Copy the raw serialized data into a Python bytes object
            const PyObjectRef serializedBytes(PyBytes_FromStringAndSize(reinterpret_cast<const char*>(message.serialized_data->buffer), message.serialized_data->buffer_length));
            const PyObjectRef pythonMessage(PyObject_CallFunctionObjArgs(m_deserializeMessage.get(), serializedBytes.get(), getMessageClass(messageType), nullptr));
            const PyObjectRef yamlObject(PyObject_CallOneArg(m_messageToYaml.get(), pythonMessage));

            result = PyUnicode_AsUTF8(yamlObject);
        }

        PyGILState_Release(gilState);
        return result;
    }

    // Converts one serialized message from a bag. message_to_json is not available in Jazzy,
    // so message_to_ordereddict and json.dumps are used instead
    std::string
    toJson(const rosbag2_storage::SerializedBagMessage& message,
           const std::string&                           messageType)
    {
        std::string result;
        const auto gilState = PyGILState_Ensure();

        // Scoped, so that the reference counts of all temporary Python objects are decreased while the GIL is still held
        {
            const PyObjectRef serializedBytes(PyBytes_FromStringAndSize(reinterpret_cast<const char*>(message.serialized_data->buffer), message.serialized_data->buffer_length));
            const PyObjectRef pythonMessage(PyObject_CallFunctionObjArgs(m_deserializeMessage.get(), serializedBytes.get(), getMessageClass(messageType), nullptr));
            const PyObjectRef orderedDictObject(PyObject_CallOneArg(m_messageToOrderedDict.get(), pythonMessage));
            const PyObjectRef jsonObject(PyObject_CallOneArg(m_jsonDumps.get(), orderedDictObject));

            result = PyUnicode_AsUTF8(jsonObject);
        }

        PyGILState_Release(gilState);
        return result;
    }

    // Converts one serialized message from a bag into flattened CSV column names and values
    std::vector<std::pair<std::string, std::string> >
    toCsv(const rosbag2_storage::SerializedBagMessage& message,
          const std::string&                           messageType)
    {
        std::vector<std::pair<std::string, std::string> > result;
        const auto gilState = PyGILState_Ensure();

        // Scoped, so that the reference counts of all temporary Python objects are decreased while the GIL is still held
        {
            const PyObjectRef serializedBytes(PyBytes_FromStringAndSize(reinterpret_cast<const char*>(message.serialized_data->buffer), message.serialized_data->buffer_length));
            const PyObjectRef pythonMessage(PyObject_CallFunctionObjArgs(m_deserializeMessage.get(), serializedBytes.get(), getMessageClass(messageType), nullptr));
            const PyObjectRef orderedDictObject(PyObject_CallOneArg(m_messageToOrderedDict.get(), pythonMessage));
            const PyObjectRef flattenedObject(PyObject_CallOneArg(m_flattenMessage.get(), orderedDictObject));

            const auto numberOfFields = PyList_Size(flattenedObject.get());
            result.reserve(numberOfFields);

            for (Py_ssize_t i = 0; i < numberOfFields; ++i) {
                auto* const field = PyList_GetItem(flattenedObject.get(), i);
                auto* const key = PyTuple_GetItem(field, 0);
                auto* const value = PyTuple_GetItem(field, 1);

                result.emplace_back(PyUnicode_AsUTF8(key), PyUnicode_AsUTF8(value));
            }
        }

        PyGILState_Release(gilState);
        return result;
    }

private:
    // Returns the cached Python message class for the given type string
    PyObject*
    getMessageClass(const std::string& messageType)
    {
        const auto iterator = m_messageClasses.find(messageType);
        if (iterator != m_messageClasses.end()) {
            return iterator->second;
        }

        // Store class name if unknown so far
        PyObjectRef messageClass(PyObject_CallFunction(m_getMessage.get(), "s", messageType.c_str()));
        return m_messageClasses.emplace(messageType, std::move(messageClass)).first->second.get();
    }

private:
    PyObjectRef m_deserializeMessage;
    PyObjectRef m_getMessage;
    PyObjectRef m_messageToYaml;
    PyObjectRef m_messageToOrderedDict;
    PyObjectRef m_jsonDumps;
    PyObjectRef m_csvModule;
    PyObjectRef m_flattenMessage;

    std::unordered_map<std::string, PyObjectRef> m_messageClasses;
};


std::unique_ptr<MessageFile::SingleFileWriter>
createSingleFileWriter(MessageFile::FileFormat format,
                       const std::string&      filePath,
                       MessageConverterBridge& bridge)
{
    switch (format) {
    case MessageFile::FileFormat::YAML:
        return std::make_unique<MessageFile::YAMLFileWriter>(filePath, [&bridge] (const auto& message, const auto& messageType) {
            return bridge.toYaml(message, messageType);
        });
    case MessageFile::FileFormat::JSON:
        return std::make_unique<MessageFile::JSONFileWriter>(filePath, [&bridge] (const auto& message, const auto& messageType) {
            return bridge.toJson(message, messageType);
        });
    case MessageFile::FileFormat::CSV:
        return std::make_unique<MessageFile::CSVFileWriter>(filePath, [&bridge] (const auto& message, const auto& messageType) {
            return bridge.toCsv(message, messageType);
        });
    }

    return nullptr;
}


bool
writeMessageToFile(MessageFile::FileFormat                      format,
                   MessageConverterBridge&                      bridge,
                   const rosbag2_storage::SerializedBagMessage& message,
                   const std::string&                           messageType,
                   const std::string&                           filePath)
{
    std::string content;

    switch (format) {
    case MessageFile::FileFormat::YAML:
        content = bridge.toYaml(message, messageType);
        break;
    case MessageFile::FileFormat::JSON:
        content = bridge.toJson(message, messageType);
        break;
    case MessageFile::FileFormat::CSV: {
        const auto csvContent = bridge.toCsv(message, messageType);

        std::vector<std::string> headers;
        std::vector<std::string> values;
        headers.reserve(csvContent.size());
        values.reserve(csvContent.size());

        for (const auto& [header, value] : csvContent) {
            headers.push_back(header);
            values.push_back(value);
        }

        content = MessageFile::csvContentToString(headers, { values });
        break;
    }
    }

    return writeFileContent(filePath, content);
}
} // namespace


BagMessageToFileThread::BagMessageToFileThread(const Parameters::BagMessageToFileParameters& parameters, QObject* parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters)
{
}


void
BagMessageToFileThread::run()
{
    // Prepare parameters
    const auto topicType = Utils::ROS::getTopicType(m_parameters.sourceDirectory, m_parameters.topicName);
    const auto messageCount = Utils::ROS::getTopicMessageCount(m_parameters.sourceDirectory, m_parameters.topicName);
    if (!topicType || !messageCount) {
        emit failed();
        return;
    }

    Utils::General::createAndClearDirectory(m_parameters.targetDirectory.toStdString());

    const auto fileFormat = MessageFile::fileFormatFromString(m_parameters.format.toStdString());
    if (!fileFormat) {
        emit failed();
        return;
    }

    const auto topicNameStdString = m_parameters.topicName.toStdString();
    const auto targetDirectoryStd = m_parameters.targetDirectory.toStdString();
    const auto fileExtension = "." + MessageFile::fileFormatToString(*fileFormat);

    MessageConverterBridge bridge;

    std::unique_ptr<MessageFile::SingleFileWriter> singleFileWriter;
    if (m_parameters.writeSingleOutputFile) {
        // Check if the file is writable
        const auto targetFilePath = targetDirectoryStd + "/topic" + fileExtension;
        singleFileWriter = createSingleFileWriter(*fileFormat, targetFilePath, bridge);

        if (!singleFileWriter || !singleFileWriter->isOpen()) {
            emit failed();
            return;
        }
    }

    rosbag2_cpp::Reader reader;
    reader.open(m_sourceDirectory);

    auto iterationCount = 0;
    while (reader.has_next()) {
        if (isInterruptionRequested()) {
            reader.close();
            return;
        }

        const auto message = reader.read_next();
        if (message->topic_name != topicNameStdString) {
            continue;
        }

        auto writeSucceeded = false;
        if (singleFileWriter) {
            writeSucceeded = singleFileWriter->write(iterationCount, *message, topicType->toStdString());
        } else {
            const auto targetFilePath = targetDirectoryStd + "/" + topicNameStdString + "_" + std::to_string(iterationCount) + fileExtension;
            writeSucceeded = writeMessageToFile(*fileFormat, bridge, *message, topicType->toStdString(), targetFilePath);
        }

        if (!writeSucceeded) {
            reader.close();
            emit failed();
            return;
        }

        iterationCount++;
        emit progressChanged(QString("Writing Message %1 of %2...").arg(iterationCount).arg(*messageCount),
                             (static_cast<float>(iterationCount) / static_cast<float>(*messageCount) * 100));
    }

    if (singleFileWriter && !singleFileWriter->finish()) {
        emit failed();
    }
    reader.close();
}
