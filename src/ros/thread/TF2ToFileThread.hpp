#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

#include <QJsonObject>

#include "tf2_msgs/msg/tf_message.hpp"

#include "yaml-cpp/yaml.h"

template<typename T>
concept NodeParameter = std::same_as<T, YAML::Node> || std::same_as<T, QJsonObject>;

// Used to send nonstatic transformations at a custom rate
class TF2ToFileThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    TF2ToFileThread(const Parameters::TF2ToFileParameters& parameters,
                    QObject*                               parent = nullptr);

    void
    run() override;

private:
    // Both YAML node and QJsonObject are very similar in terms of array handling, so
    // templatize to avoid messy boilerplate code for json and yaml handling
    template<typename T>
    requires NodeParameter<T>
    void
    fillNode(std::shared_ptr<tf2_msgs::msg::TFMessage> tfMessage,
             T&                                        finalNode) const
    {
        for (size_t i = 0; i < tfMessage->transforms.size(); i++) {
            const auto& currentTransform = tfMessage->transforms.at(i);
            T iterationNode;

            T translation;
            translation["x"] = currentTransform.transform.translation.x;
            translation["y"] = currentTransform.transform.translation.y;
            translation["z"] = currentTransform.transform.translation.z;

            T rotation;
            rotation["x"] = currentTransform.transform.rotation.x;
            rotation["y"] = currentTransform.transform.rotation.y;
            rotation["z"] = currentTransform.transform.rotation.z;
            rotation["w"] = currentTransform.transform.rotation.w;

            iterationNode["translation"] = translation;
            iterationNode["rotation"] = rotation;

            if constexpr (std::is_same_v<T, QJsonObject>) {
                iterationNode["header_frame_id"] = QString::fromStdString(currentTransform.header.frame_id);
                iterationNode["child_frame_id"] = QString::fromStdString(currentTransform.child_frame_id);
                if (m_parameters.keepTimestamps) {
                    iterationNode["header_stamp"] = QString::number(currentTransform.header.stamp.nanosec / 1e9, 'f', 6);
                }

                finalNode["transform_" + QString::number(i)] = iterationNode;
            } else {
                iterationNode["header_frame_id"] = currentTransform.header.frame_id;
                iterationNode["child_frame_id"] = currentTransform.child_frame_id;
                if (m_parameters.keepTimestamps) {
                    iterationNode["header_stamp"] = currentTransform.header.stamp.nanosec / 1e9;
                }

                finalNode["transform_" + std::to_string(i)] = iterationNode;
            }
        }
    }

private:
    const Parameters::TF2ToFileParameters& m_parameters;
};
