// ROS2 offers functions to convert any type of ROS2 message to yaml, including custom messages.
// However, these functions are only available in the Python implementation, not in C++.
// Thus, we use CPython here to access the python functions underneath

// CPython and Qt use equal macro names (e.g. "slot"). To avoid collisions,
// declare the Python header before everything else
#include <Python.h>

#include "BagMessageToFileThread.hpp"

#include "UtilsGeneral.hpp"
#include "UtilsROS.hpp"

#include <QJsonDocument>
#include <QJsonObject>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include "yaml-cpp/yaml.h"

#include <fstream>

namespace
{
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

// Converts serialized ROS2 bag messages into YAML or JSON by calling
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

    std::unordered_map<std::string, PyObjectRef> m_messageClasses;
};
} // namespace


BagMessageToFileThread::BagMessageToFileThread(const Parameters::BagMessageToFileParameters& parameters, QObject* parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters)
{
}


void
BagMessageToFileThread::run()
{
    const auto targetDirectoryStd = m_parameters.targetDirectory.toStdString();
    Utils::General::createAndClearDirectory(targetDirectoryStd);

    const auto topicTypeStdString = Utils::ROS::getTopicType(m_parameters.sourceDirectory, m_parameters.topicName)->toStdString();
    const auto& topicNameStdString = m_parameters.topicName.toStdString();
    const auto totalInstances = *Utils::ROS::getTopicMessageCount(m_parameters.sourceDirectory, m_parameters.topicName);

    YAML::Node messagesNode;
    QJsonObject messagesObject;
    std::string topicContentString = "";
    const auto fileExtension = m_parameters.isYaml ? ".yaml" : ".json";

    auto bridge = std::make_unique<MessageConverterBridge>();
    auto iterationCount = 0;
    rosbag2_storage::SerializedBagMessageSharedPtr message;

    auto reader = std::make_unique<rosbag2_cpp::Reader>();
    reader->open(m_sourceDirectory);

    while (reader->has_next()) {
        if (isInterruptionRequested()) {
            reader->close();
            return;
        }

        // Read and deserialize the message
        message = reader->read_next();
        if (message->topic_name != topicNameStdString) {
            continue;
        }

        if (m_parameters.writeSingleOutputFile) {
            if (m_parameters.isYaml) {
                messagesNode[std::to_string(iterationCount)] = YAML::Load(bridge->toYaml(*message, topicTypeStdString));
            } else {
                messagesObject[QString::number(iterationCount)] = QJsonDocument::fromJson(QByteArray::fromStdString(bridge->toJson(*message, topicTypeStdString))).object();
            }
        } else {
            topicContentString = m_parameters.isYaml ? bridge->toYaml(*message, topicTypeStdString) : bridge->toJson(*message, topicTypeStdString);
            std::ofstream out(targetDirectoryStd + "/" + topicNameStdString + "_" + std::to_string(iterationCount) + fileExtension);

            out << topicContentString;
            out.close();
        }

        iterationCount++;
        emit progressChanged("Writing Message " + QString::number(iterationCount) + " of " + QString::number(totalInstances) + "...",
                             (static_cast<float>(iterationCount) / static_cast<float>(totalInstances) * 100));
    }

    if (m_parameters.writeSingleOutputFile) {
        std::ofstream fout(m_parameters.isYaml ? targetDirectoryStd + "/topic.yaml" : targetDirectoryStd + "/topic.json");

        try {
            m_parameters.isYaml ? fout << messagesNode
                                : fout << QJsonDocument(messagesObject).toJson(QJsonDocument::Indented).toStdString();
        } catch (std::ofstream::failure& /* exeption */) {
            emit failed();
        }
    }
    reader->close();

    emit finished();
}
