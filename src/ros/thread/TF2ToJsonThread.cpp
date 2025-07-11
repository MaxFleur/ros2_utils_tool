#include "TF2ToJsonThread.hpp"

#include "UtilsROS.hpp"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <filesystem>

TF2ToJsonThread::TF2ToJsonThread(const Parameters::TF2ToJsonParameters& parameters, QObject* parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent), m_parameters(parameters)
{
}


void
TF2ToJsonThread::run()
{
    const auto& targetDirectory = m_parameters.targetDirectory;
    if (std::filesystem::exists(targetDirectory.toStdString())) {
        std::filesystem::remove_all(targetDirectory.toStdString());
    }

    const auto messageCount = Utils::ROS::getTopicMessageCount(m_sourceDirectory, m_topicName);
    auto iterationCount = 0;

    auto reader = std::make_shared<rosbag2_cpp::Reader>();
    reader->open(m_sourceDirectory);

    rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializiation;
    auto tfMessage = std::make_shared<tf2_msgs::msg::TFMessage>();
    QJsonArray messagesArray;

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

        QJsonObject transformsObject;
        for (size_t i = 0; i < tfMessage->transforms.size(); i++) {
            QJsonObject tfEntry;
            tfEntry["header_frame_id"] = QString::fromStdString(tfMessage->transforms.at(i).header.frame_id);
            tfEntry["child_frame_id"] = QString::fromStdString(tfMessage->transforms.at(i).child_frame_id);
            if (m_parameters.keepTimestamps) {
                tfEntry["header_stamp"] = QString::number(tfMessage->transforms.at(i).header.stamp.nanosec / 1e9, 'f', 6);
            }

            QJsonObject translation;
            translation["x"] = tfMessage->transforms.at(i).transform.translation.x;
            translation["y"] = tfMessage->transforms.at(i).transform.translation.y;
            translation["z"] = tfMessage->transforms.at(i).transform.translation.z;

            QJsonObject rotation;
            rotation["x"] = tfMessage->transforms.at(i).transform.rotation.x;
            rotation["y"] = tfMessage->transforms.at(i).transform.rotation.y;
            rotation["z"] = tfMessage->transforms.at(i).transform.rotation.z;
            rotation["w"] = tfMessage->transforms.at(i).transform.rotation.w;

            tfEntry["translation"] = translation;
            tfEntry["rotation"] = rotation;

            transformsObject["transform_" + QString::number(i)] = tfEntry;
        }

        QJsonObject messageObject;
        messageObject["message_" + QString::number(iterationCount)] = transformsObject;
        messagesArray.append(messageObject);

        iterationCount++;
        emit progressChanged("Writing message " + QString::number(iterationCount) + " of " + QString::number(*messageCount) + "...",
                             ((float) iterationCount / (float) *messageCount) * 100);
    }

    QJsonDocument doc(messagesArray);
    QFile outFile(targetDirectory);

    if (!outFile.open(QIODevice::WriteOnly)) {
        emit failed();
    }

    outFile.write(doc.toJson(m_parameters.compactOutput ? QJsonDocument::Compact : QJsonDocument::Indented));
    outFile.close();

    emit finished();
}
