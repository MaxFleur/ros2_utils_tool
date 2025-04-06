#include "StartWidget.hpp"

#include "SettingsDialog.hpp"
#include "UtilsUI.hpp"

#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>
#include <QVBoxLayout>

StartWidget::StartWidget(Parameters::DialogParameters& dialogParameters, QWidget *parent) :
    QWidget(parent), m_dialogParameters(dialogParameters)
{
    m_headerLabel = new QLabel("ROS TOOLS");
    Utils::UI::setWidgetFontSize(m_headerLabel);
    m_headerLabel->setAlignment(Qt::AlignHCenter);

    m_settingsButton = new QPushButton;
    m_settingsButton->setFlat(true);

    auto settingsButtonSizePolicy = m_settingsButton->sizePolicy();
    settingsButtonSizePolicy.setRetainSizeWhenHidden(true);
    m_settingsButton->setSizePolicy(settingsButtonSizePolicy);

    auto* const settingsButtonLayout = new QHBoxLayout;
    settingsButtonLayout->addStretch();
    settingsButtonLayout->addWidget(m_settingsButton);

    // Create four widgets: One for providing the overview for bag and publishing tools,
    // one for bag, one for publishing and one for info tools
    // Overview widget
    m_conversionToolsButton = createToolButton("Conversion\nTools");
    m_bagToolsButton = createToolButton("Bag Tools");
    m_publishingToolsButton = createToolButton("Publishing\nTools");
    m_infoToolsButton = createToolButton("Info\nTools");

    auto* const overallUpperLayout = new QHBoxLayout;
    overallUpperLayout->addStretch();
    overallUpperLayout->addWidget(m_conversionToolsButton);
    overallUpperLayout->addWidget(m_bagToolsButton);
    overallUpperLayout->addStretch();

    auto* const overallLowerLayout = new QHBoxLayout;
    overallLowerLayout->addStretch();
    overallLowerLayout->addWidget(m_publishingToolsButton);
    overallLowerLayout->addWidget(m_infoToolsButton);
    overallLowerLayout->addStretch();

    auto* const overallToolsMainLayout = new QVBoxLayout;
    overallToolsMainLayout->addLayout(overallUpperLayout);
    overallToolsMainLayout->addLayout(overallLowerLayout);

    auto* const overallToolsWidget = new QWidget;
    overallToolsWidget->setLayout(overallToolsMainLayout);

    // Conversion tools widget
    m_bagToVideoPushButton = createToolButton("Bag to Video");
    m_videoToBagPushButton = createToolButton("Video to Bag");
    m_bagToPCDsPushButton = createToolButton("Bag to\nPCD Files");
    m_PCDsToBagPushButton = createToolButton("PCD Files\nto Bag");
    m_bagToImagesPushButton = createToolButton("Bag to Images");

    auto* const conversionToolsUpperLayout = new QHBoxLayout;
    conversionToolsUpperLayout->addStretch();
    conversionToolsUpperLayout->addWidget(m_bagToVideoPushButton);
    conversionToolsUpperLayout->addWidget(m_videoToBagPushButton);
    conversionToolsUpperLayout->addStretch();

    auto* const conversionToolsCenterLayout = new QHBoxLayout;
    conversionToolsCenterLayout->addStretch();
    conversionToolsCenterLayout->addWidget(m_bagToPCDsPushButton);
    conversionToolsCenterLayout->addWidget(m_PCDsToBagPushButton);
    conversionToolsCenterLayout->addStretch();

    auto* const conversionToolsLowerLayout = new QHBoxLayout;
    conversionToolsLowerLayout->addStretch();
    conversionToolsLowerLayout->addWidget(m_bagToImagesPushButton);
    conversionToolsLowerLayout->addStretch();

    auto* const conversionToolsMainLayout = new QVBoxLayout;
    conversionToolsMainLayout->addStretch();
    conversionToolsMainLayout->addLayout(conversionToolsUpperLayout);
    conversionToolsMainLayout->addLayout(conversionToolsCenterLayout);
    conversionToolsMainLayout->addLayout(conversionToolsLowerLayout);
    conversionToolsMainLayout->addStretch();

    auto* const conversionToolsWidget = new QWidget;
    conversionToolsWidget->setLayout(conversionToolsMainLayout);

    // Bag tools widget
    m_editBagButton = createToolButton("Edit Bag");
    m_mergeBagsButton = createToolButton("Merge Bags");
    m_recordBagButton = createToolButton("Record Bag");
    m_dummyBagButton = createToolButton("Create\nDummy Bag");
    m_compressBagButton = createToolButton("Compress\nBag");
    m_decompressBagButton = createToolButton("Decompress\nBag");

    auto* const bagToolsUpperLayout = new QHBoxLayout;
    bagToolsUpperLayout->addStretch();
    bagToolsUpperLayout->addWidget(m_editBagButton);
    bagToolsUpperLayout->addWidget(m_mergeBagsButton);
    bagToolsUpperLayout->addStretch();

    auto* const bagToolsCenterLayout = new QHBoxLayout;
    bagToolsCenterLayout->addStretch();
    bagToolsCenterLayout->addWidget(m_recordBagButton);
    bagToolsCenterLayout->addWidget(m_dummyBagButton);
    bagToolsCenterLayout->addStretch();

    auto* const bagToolsLowerLayout = new QHBoxLayout;
    bagToolsLowerLayout->addStretch();
    bagToolsLowerLayout->addWidget(m_compressBagButton);
    bagToolsLowerLayout->addWidget(m_decompressBagButton);
    bagToolsLowerLayout->addStretch();

    auto* const bagToolsMainLayout = new QVBoxLayout;
    bagToolsMainLayout->addStretch();
    bagToolsMainLayout->addLayout(bagToolsUpperLayout);
    bagToolsMainLayout->addLayout(bagToolsCenterLayout);
    bagToolsMainLayout->addLayout(bagToolsLowerLayout);
    bagToolsMainLayout->addStretch();

    auto* const bagToolsWidget = new QWidget;
    bagToolsWidget->setLayout(bagToolsMainLayout);

    // Publishing tools widget
    m_publishVideoButton = createToolButton("Publish Video\nas ROS Topic");
    m_publishImagesButton = createToolButton("Publish Images\nas ROS Topic");

    auto* const publishingToolsMainLayout = new QHBoxLayout;
    publishingToolsMainLayout->addStretch();
    publishingToolsMainLayout->addWidget(m_publishVideoButton);
    publishingToolsMainLayout->addWidget(m_publishImagesButton);
    publishingToolsMainLayout->addStretch();

    auto* const publishingToolsWidget = new QWidget;
    publishingToolsWidget->setLayout(publishingToolsMainLayout);

    // Info tools widget
    m_topicServiceInfoButton = createToolButton("Topics and\nService Info");
    m_bagInfoButton = createToolButton("Bag\nInfos");

    auto* const infoToolsMainLayout = new QHBoxLayout;
    infoToolsMainLayout->addStretch();
    infoToolsMainLayout->addWidget(m_topicServiceInfoButton);
    infoToolsMainLayout->addWidget(m_bagInfoButton);
    infoToolsMainLayout->addStretch();

    auto* const infoToolsWidget = new QWidget;
    infoToolsWidget->setLayout(infoToolsMainLayout);

    setButtonIcons();

    m_backButton = new QPushButton("Back");
    m_backButton->setVisible(false);

    auto* const backButtonLayout = new QHBoxLayout;
    backButtonLayout->addWidget(m_backButton);
    backButtonLayout->addStretch();

    m_versionLabel = new QLabel("v0.10.0");
    m_versionLabel->setToolTip("Compression/Decompression tools and restructured UI settings!");

    auto* const versionLayout = new QHBoxLayout;
    versionLayout->addStretch();
    versionLayout->addWidget(m_versionLabel);

    m_mainLayout = new QVBoxLayout;
    m_mainLayout->addLayout(settingsButtonLayout);
    m_mainLayout->addWidget(m_headerLabel);
    m_mainLayout->addStretch();
    m_mainLayout->addWidget(overallToolsWidget);
    m_mainLayout->addStretch();
    m_mainLayout->addLayout(versionLayout);
    m_mainLayout->addLayout(backButtonLayout);
    setLayout(m_mainLayout);

    const auto switchToOverallTools = [this, conversionToolsWidget, bagToolsWidget, publishingToolsWidget,
                                       infoToolsWidget, overallToolsWidget] {
        switch (m_widgetOnInstantiation) {
        case WIDGET_CONVERSION:
            replaceWidgets(conversionToolsWidget, overallToolsWidget, WIDGET_OVERALL, true);
            break;
        case WIDGET_BAG:
            replaceWidgets(bagToolsWidget, overallToolsWidget, WIDGET_OVERALL, true);
            break;
        case WIDGET_PUBLISHING:
            replaceWidgets(publishingToolsWidget, overallToolsWidget, WIDGET_OVERALL, true);
            break;
        case WIDGET_INFO:
            replaceWidgets(infoToolsWidget, overallToolsWidget, WIDGET_OVERALL, true);
            break;
        default:
            break;
        }
    };
    const auto switchToConversionTools = [this, overallToolsWidget, conversionToolsWidget] {
        replaceWidgets(overallToolsWidget, conversionToolsWidget, WIDGET_CONVERSION, false);
    };
    const auto switchToBagTools = [this, overallToolsWidget, bagToolsWidget] {
        replaceWidgets(overallToolsWidget, bagToolsWidget, WIDGET_BAG, false);
    };
    const auto switchToPublishingTools = [this, overallToolsWidget, publishingToolsWidget] {
        replaceWidgets(overallToolsWidget, publishingToolsWidget, WIDGET_PUBLISHING, false);
    };
    const auto switchToInfoTools = [this, overallToolsWidget, infoToolsWidget] {
        replaceWidgets(overallToolsWidget, infoToolsWidget, WIDGET_INFO, false);
    };

    connect(m_settingsButton, &QPushButton::clicked, this, &StartWidget::openSettingsDialog);

    connect(m_backButton, &QPushButton::clicked, this, switchToOverallTools);
    connect(m_conversionToolsButton, &QPushButton::clicked, this, switchToConversionTools);
    connect(m_bagToolsButton, &QPushButton::clicked, this, switchToBagTools);
    connect(m_publishingToolsButton, &QPushButton::clicked, this, switchToPublishingTools);
    connect(m_infoToolsButton, &QPushButton::clicked, this, switchToInfoTools);

    connect(m_bagToVideoPushButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(Utils::UI::TOOL_BAG_TO_VIDEO);
    });
    connect(m_videoToBagPushButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(Utils::UI::TOOL_VIDEO_TO_BAG);
    });
    connect(m_bagToPCDsPushButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(Utils::UI::TOOL_BAG_TO_PCDS);
    });
    connect(m_PCDsToBagPushButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(Utils::UI::TOOL_PCDS_TO_BAG);
    });
    connect(m_bagToImagesPushButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(Utils::UI::TOOL_BAG_TO_IMAGES);
    });
    connect(m_editBagButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(Utils::UI::TOOL_EDIT_BAG);
    });
    connect(m_mergeBagsButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(Utils::UI::TOOL_MERGE_BAGS);
    });
    connect(m_recordBagButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(Utils::UI::TOOL_RECORD_BAG);
    });
    connect(m_dummyBagButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(Utils::UI::TOOL_DUMMY_BAG);
    });
    connect(m_compressBagButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(Utils::UI::TOOL_COMPRESS_BAG);
    });
    connect(m_decompressBagButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(Utils::UI::TOOL_DECOMPRESS_BAG);
    });
    connect(m_publishVideoButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(Utils::UI::TOOL_PUBLISH_VIDEO);
    });
    connect(m_publishImagesButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(Utils::UI::TOOL_PUBLISH_IMAGES);
    });
    connect(m_topicServiceInfoButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(Utils::UI::TOOL_TOPICS_SERVICES_INFO);
    });
    connect(m_bagInfoButton, &QPushButton::clicked, this, [this] {
        emit toolRequested(Utils::UI::TOOL_BAG_INFO);
    });

    switch (m_widgetOnInstantiation) {
    case WIDGET_CONVERSION:
        switchToConversionTools();
        break;
    case WIDGET_BAG:
        switchToBagTools();
        break;
    case WIDGET_PUBLISHING:
        switchToPublishingTools();
        break;
    case WIDGET_INFO:
        switchToInfoTools();
        break;
    default:
        break;
    }
}


void
StartWidget::openSettingsDialog()
{
    auto* const settingsDialog = new SettingsDialog(m_dialogParameters);
    settingsDialog->exec();
}


// Used to switch between the four overall widgets
void
StartWidget::replaceWidgets(QWidget* fromWidget, QWidget* toWidget, int widgetIdentifier, bool otherItemVisibility)
{
    // If the back button is visible, the other elements should be hidden and vice versa
    m_backButton->setVisible(!otherItemVisibility);
    m_settingsButton->setVisible(otherItemVisibility);
    m_versionLabel->setVisible(otherItemVisibility);
    m_widgetOnInstantiation = widgetIdentifier;

    switch (m_widgetOnInstantiation) {
    case WIDGET_OVERALL:
        m_headerLabel->setText("ROS TOOLS");
        break;
    case WIDGET_CONVERSION:
        m_headerLabel->setText("CONVERSION TOOLS");
        break;
    case WIDGET_BAG:
        m_headerLabel->setText("BAG TOOLS");
        break;
    case WIDGET_PUBLISHING:
        m_headerLabel->setText("PUBLISHING TOOLS");
        break;
    case WIDGET_INFO:
        m_headerLabel->setText("INFO TOOLS");
        break;
    default:
        break;
    }

    m_mainLayout->replaceWidget(fromWidget, toWidget);
    fromWidget->setVisible(false);
    toWidget->setVisible(true);
}


QPointer<QToolButton>
StartWidget::createToolButton(const QString& buttonText)
{
    auto* const toolButton = new QToolButton;
    toolButton->setText(buttonText);
    toolButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    toolButton->setIconSize(QSize(100, 45));
    toolButton->setFixedSize(QSize(150, 150));

    Utils::UI::setWidgetFontSize(toolButton, true);

    return toolButton;
}


void
StartWidget::setButtonIcons()
{
    const auto isDarkMode = Utils::UI::isDarkMode();
    m_settingsButton->setIcon(QIcon(isDarkMode ? ":/icons/gear_white.svg" : ":/icons/gear_black.svg"));

    m_conversionToolsButton->setIcon(QIcon(isDarkMode ? ":/icons/conversion_tools_white.svg" : ":/icons/conversion_tools_black.svg"));
    m_bagToolsButton->setIcon(QIcon(isDarkMode ? ":/icons/bag_tools_white.svg" : ":/icons/bag_tools_black.svg"));
    m_publishingToolsButton->setIcon(QIcon(isDarkMode ? ":/icons/publishing_tools_white.svg" : ":/icons/publishing_tools_black.svg"));
    m_infoToolsButton->setIcon(QIcon(isDarkMode ? ":/icons/info_tools_white.svg" : ":/icons/info_tools_black.svg"));

    m_bagToVideoPushButton->setIcon(QIcon(isDarkMode ? ":/icons/bag_to_video_white.svg" : ":/icons/bag_to_video_black.svg"));
    m_videoToBagPushButton->setIcon(QIcon(isDarkMode ? ":/icons/video_to_bag_white.svg" : ":/icons/video_to_bag_black.svg"));
    m_bagToPCDsPushButton->setIcon(QIcon(isDarkMode ? ":/icons/bag_to_pcd_white.svg" : ":/icons/bag_to_pcd_black.svg"));
    m_PCDsToBagPushButton->setIcon(QIcon(isDarkMode ? ":/icons/pcd_to_bag_white.svg" : ":/icons/pcd_to_bag_black.svg"));
    m_bagToImagesPushButton->setIcon(QIcon(isDarkMode ? ":/icons/bag_to_images_white.svg" : ":/icons/bag_to_images_black.svg"));

    m_editBagButton->setIcon(QIcon(isDarkMode ? ":/icons/edit_bag_white.svg" : ":/icons/edit_bag_black.svg"));
    m_mergeBagsButton->setIcon(QIcon(isDarkMode ? ":/icons/merge_bags_white.svg" : ":/icons/merge_bags_black.svg"));
    m_recordBagButton->setIcon(QIcon(isDarkMode ? ":/icons/record_bag_white.svg" : ":/icons/record_bag_black.svg"));
    m_dummyBagButton->setIcon(QIcon(isDarkMode ? ":/icons/dummy_bag_white.svg" : ":/icons/dummy_bag_black.svg"));
    m_compressBagButton->setIcon(QIcon(isDarkMode ? ":/icons/compress_bag_white.svg" : ":/icons/compress_bag_black.svg"));
    m_decompressBagButton->setIcon(QIcon(isDarkMode ? ":/icons/decompress_bag_white.svg" : ":/icons/decompress_bag_black.svg"));

    m_publishVideoButton->setIcon(QIcon(isDarkMode ? ":/icons/publish_video_white.svg" : ":/icons/publish_video_black.svg"));
    m_publishImagesButton->setIcon(QIcon(isDarkMode ? ":/icons/publish_images_white.svg" : ":/icons/publish_images_black.svg"));

    m_topicServiceInfoButton->setIcon(QIcon(isDarkMode ? ":/icons/topics_services_info_white.svg"
                                                       : ":/icons/topics_services_info_black.svg"));
    m_bagInfoButton->setIcon(QIcon(isDarkMode ? ":/icons/bag_info_white.svg" : ":/icons/bag_info_black.svg"));
}


bool
StartWidget::event(QEvent *event)
{
    [[unlikely]] if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setButtonIcons();
    }
    return QWidget::event(event);
}
