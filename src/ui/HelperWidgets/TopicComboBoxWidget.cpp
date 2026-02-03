#include "TopicComboBoxWidget.hpp"

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
        QString topicType;

        switch (outputFormat) {
        case OUTPUT_VIDEO:
        case OUTPUT_IMAGES:
            topicType = "sensor_msgs/msg/Image";
            break;
        case OUTPUT_PCDS:
            topicType = "sensor_msgs/msg/PointCloud2";
            break;
        case OUTPUT_TF_TO_FILE:
            topicType = "tf2_msgs/msg/TFMessage";
            break;
        default:
            break;
        }

        Utils::UI::fillComboBoxWithTopics(m_topicNameComboBox, m_parameters.sourceDirectory, topicType);

        if (!m_parameters.topicName.isEmpty()) {
            m_topicNameComboBox->setCurrentText(m_parameters.topicName);
        }
    }

    connect(m_topicNameComboBox, &QComboBox::currentTextChanged, this, [this] (const QString& text) {
        writeParameterToSettings(m_parameters.topicName, text, m_settings);
    });;
}


void
TopicComboBoxWidget::findSourceButtonPressed()
{
    AdvancedInputWidget::findSourceButtonPressed();

    QString topicType;
    switch (m_outputFormat) {
    case OUTPUT_VIDEO:
        topicType = "sensor_msgs/msg/Image";
        break;
    case OUTPUT_IMAGES:
        topicType = "sensor_msgs/msg/Image";
        break;
    case OUTPUT_PCDS:
        topicType = "sensor_msgs/msg/PointCloud2";
        break;
    case OUTPUT_TF_TO_FILE:
        topicType = "tf2_msgs/msg/TFMessage";
        break;
    default:
        break;
    }

    if (const auto containsTopics = Utils::UI::fillComboBoxWithTopics(m_topicNameComboBox, m_sourceLineEdit->text(), topicType); !containsTopics) {
        Utils::UI::createCriticalMessageBox("Topic not found!", "The bag file does not contain any corresponding topics!");
        return;
    }
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() && !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}
