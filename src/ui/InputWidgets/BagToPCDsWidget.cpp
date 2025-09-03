#include "BagToPCDsWidget.hpp"

#include "UtilsUI.hpp"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>

BagToPCDsWidget::BagToPCDsWidget(Parameters::AdvancedParameters& parameters, QWidget *parent) :
    TopicComboBoxWidget(parameters, "Bag to PCD Files", ":/icons/bag_to_pcd", "Bag File:", "PCDs Location:", "bag_to_pcds", OUTPUT_PCDS, parent),
    m_parameters(parameters), m_settings(parameters, "bag_to_pcds")
{
    m_sourceLineEdit->setToolTip("The source bag file directory.");
    m_topicNameComboBox->setToolTip("The point cloud bag topic.\nIf the bag contains multiple point cloud topics, you can choose one of them.");
    m_targetLineEdit->setToolTip("The target point cloud files directory.");

    m_basicOptionsFormLayout->insertRow(1, "Topic Name:", m_topicNameComboBox);

    m_controlsLayout->addStretch();

    // Generally, only enable this if the source bag, topic name and target dir line edit contain text
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());
}
