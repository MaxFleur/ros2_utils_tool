#include "TF2ToFileThread.hpp"

#include "UtilsGeneral.hpp"
#include "UtilsROS.hpp"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rosbag2_cpp/reader.hpp"

#include <filesystem>
#include <fstream>

TF2ToFileThread::TF2ToFileThread(const Parameters::TF2ToFileParameters& parameters, QObject* parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent), m_parameters(parameters)
{
}


void
TF2ToFileThread::run()
{
    const auto& targetDirectory = m_parameters.targetDirectory;
    if (std::filesystem::exists(targetDirectory.toStdString())) {
        std::filesystem::remove_all(targetDirectory.toStdString());
    }

    const auto isFormatJson = Utils::General::getFileExtension(targetDirectory) == "json";
    const auto messageCount = Utils::ROS::getTopicMessageCount(m_sourceDirectory, m_topicName);
    auto iterationCount = 0;

    auto reader = std::make_shared<rosbag2_cpp::Reader>();
    reader->open(m_sourceDirectory);

    rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializiation;
    auto tfMessage = std::make_shared<tf2_msgs::msg::TFMessage>();

    QJsonArray messagesArray;
    YAML::Node transformsNode;

    const auto writeMessageToJson = [this, tfMessage, &messagesArray, &iterationCount] {
        QJsonObject transformsObject;
        fillNode(tfMessage, transformsObject);

        QJsonObject messageObject;
        messageObject["message_" + QString::number(iterationCount)] = transformsObject;
        messagesArray.append(messageObject);
    };

    const auto writeMessageToYaml = [this, tfMessage, &transformsNode, &iterationCount] {
        YAML::Node transformNode;
        fillNode(tfMessage, transformNode);

        transformsNode["message_" + std::to_string(iterationCount)] = transformNode;
    };

    while (reader->has_next()) {
        if (isInterruptionRequested()) {
            break;
        }

        auto bagMessage = reader->read_next();
        if (bagMessage->topic_name != m_topicName) {
            continue;
        }

        rclcpp::SerializedMessage serializedMsg(*bagMessage->serialized_data);
        serializiation.deserialize_message(&serializedMsg, tfMessage.get());

        isFormatJson ? writeMessageToJson() : writeMessageToYaml();

        iterationCount++;
        emit progressChanged("Writing message " + QString::number(iterationCount) + " of " + QString::number(*messageCount) + "...",
                             (static_cast<float>(iterationCount) / static_cast<float>(*messageCount) * 100));
    }

    if (isFormatJson) {
        QJsonDocument doc(messagesArray);
        QFile outFile(targetDirectory);

        if (!outFile.open(QIODevice::WriteOnly)) {
            emit failed();
        }

        outFile.write(doc.toJson(m_parameters.compactOutput ? QJsonDocument::Compact : QJsonDocument::Indented));
        outFile.close();
    } else {
        std::ofstream fout(targetDirectory.toStdString());

        try {
            fout << transformsNode;
        } catch (std::ofstream::failure& /* exeption */) {
            emit failed();
        }
    }

    emit finished();
}
