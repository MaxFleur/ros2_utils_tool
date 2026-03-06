#include "TopicComboBoxWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QComboBox>

TopicComboBoxWidget::TopicComboBoxWidget(Parameters::AdvancedParameters& parameters, const QString& headerText,
                                         const QString& iconPath, const QString& sourceFormLayoutName, const QString& targetFormLayoutName,
                                         const QString& settingsIdentifier, int outputFormat, QWidget *parent) :
    AdvancedInputWidget(parameters, headerText, iconPath, sourceFormLayoutName, targetFormLayoutName, settingsIdentifier, outputFormat, parent)
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

    const auto fill = [this] (const QString& topicType) {
        const auto videoTopics = Utils::ROS::getBagTopicNames(m_sourceLineEdit->text(), topicType);
        if (videoTopics.empty()) {
            return;
        }

        for (const auto& videoTopic : videoTopics) {
            m_topicNameComboBox->addItem(videoTopic);
        }
    };

    switch (m_outputFormat) {
    case OUTPUT_VIDEO:
    case OUTPUT_IMAGES:
        fill("sensor_msgs/msg/Image");
        fill("sensor_msgs/msg/CompressedImage");
        break;
    case OUTPUT_PCDS:
        fill("sensor_msgs/msg/PointCloud2");
        break;
    case OUTPUT_TF_TO_FILE:
        fill("tf2_msgs/msg/TFMessage");
        break;
    default:
        break;
    }
}
