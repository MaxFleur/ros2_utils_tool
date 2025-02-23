#include "BagToPCDsWidget.hpp"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>
#include <QVBoxLayout>

BagToPCDsWidget::BagToPCDsWidget(Utils::UI::AdvancedParameters& parameters, QWidget *parent) :
    AdvancedInputWidget(parameters, "Bag to PCD Files", ":/icons/bag_to_pcd", "bag_to_pcds", OUTPUT_PCDS, parent),
    m_parameters(parameters), m_settings(parameters, "bag_to_pcds")
{
    m_topicNameComboBox->setToolTip("The point cloud bag topic.\nIf the bag contains multiple point cloud topics, you can choose one of them.");
    m_targetLineEdit->setToolTip("The directory where the point cloud files should be stored.");

    auto* const pcdsLocationButton = new QToolButton;
    auto* const searchImagesPathLayout = Utils::UI::createLineEditButtonLayout(m_targetLineEdit, pcdsLocationButton);

    auto* const basicOptionsFormLayout = new QFormLayout;
    basicOptionsFormLayout->addRow("Bag File:", m_findSourceLayout);
    basicOptionsFormLayout->addRow("Topic Name:", m_topicNameComboBox);
    basicOptionsFormLayout->addRow("PCDs Location:", searchImagesPathLayout);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(basicOptionsFormLayout);
    controlsLayout->addStretch();

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(controlsLayout);
    controlsSqueezedLayout->addStretch();

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    // Generally, only enable this if the source bag, topic name and target dir line edit contain text
    enableOkButton(!m_parameters.sourceDirectory.isEmpty() &&
                   !m_parameters.topicName.isEmpty() && !m_parameters.targetDirectory.isEmpty());

    connect(pcdsLocationButton, &QPushButton::clicked, this, &BagToPCDsWidget::targetLocationButtonPressed);
}
