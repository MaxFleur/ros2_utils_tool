#include "TopicComboBoxWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QComboBox>

TopicComboBoxWidget::TopicComboBoxWidget(Parameters::AdvancedParameters& parameters, const QString& headerText,
                                         const QString& iconPath, const QString& sourceFormLayoutName, const QString& targetFormLayoutName,
                                         const QString& settingsIdentifier, OUTPUT_TYPE outputType, QWidget *parent) :
    AdvancedInputWidget(parameters, headerText, iconPath, sourceFormLayoutName, targetFormLayoutName, settingsIdentifier, outputType, parent)
{
    m_topicNameComboBox = new QComboBox;
    m_topicNameComboBox->setMinimumWidth(200);

    if (!m_parameters.sourceDirectory.isEmpty()) {
        mainFillOperation();

        if (!m_parameters.topicName.isEmpty()) {
            m_topicNameComboBox->setCurrentText(m_parameters.topicName);
        }
    }

    connect(m_topicNameComboBox, &QComboBox::currentTextChanged, this, [this] (const QString& text) {
        writeParameterToSettings(m_parameters.topicName, text, m_settings);
    });;
}


void
TopicComboBoxWidget::fillTopicComboBox()
{
    mainFillOperation();

    if (m_topicNameComboBox->count() == 0) {
        Utils::UI::createCriticalMessageBox("Topic not found!", "The bag file does not contain any corresponding topics!");
        return;
    }
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}


void
TopicComboBoxWidget::mainFillOperation()
{
    if (m_sourceLineEdit->text().isEmpty()) {
        return;
    }

    m_topicNameComboBox->clear();

    const auto fillComboBoxWithTopics = [this] (const QString& topicType, bool isYaml = false) {
        QVector<QString> topics;

        if (isYaml) {
            const auto& metadata = Utils::ROS::getBagMetadata(m_sourceLineEdit->text());
            for (const auto& topic : metadata.topics_with_message_count) {
                topics.push_back(QString::fromStdString(topic.topic_metadata.name));
            }
        } else {
            topics = Utils::ROS::getBagTopicNames(m_sourceLineEdit->text(), topicType);
        }

        if (topics.empty()) {
            return;
        }

        for (const auto& topic : topics) {
            m_topicNameComboBox->addItem(topic);
        }
    };

    switch (m_outputType) {
    case OUTPUT_TYPE::OUTPUT_VIDEO:
    case OUTPUT_TYPE::OUTPUT_IMAGES:
        fillComboBoxWithTopics("sensor_msgs/msg/Image");
        fillComboBoxWithTopics("sensor_msgs/msg/CompressedImage");
        break;
    case OUTPUT_TYPE::OUTPUT_PCDS:
        fillComboBoxWithTopics("sensor_msgs/msg/PointCloud2");
        break;
    case OUTPUT_TYPE::OUTPUT_TF_TO_FILE:
        fillComboBoxWithTopics("tf2_msgs/msg/TFMessage");
        break;
    case OUTPUT_TYPE::OUTPUT_YAML:
        fillComboBoxWithTopics("tf2_msgs/msg/TFMessage", true);
        break;
    default:
        break;
    }
}
