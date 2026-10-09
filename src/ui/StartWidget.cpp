#include "StartWidget.hpp"

#include "SettingsDialog.hpp"

#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>
#include <QVBoxLayout>

StartWidget::StartWidget(Parameters::DialogParameters& dialogParameters, QWidget *parent) :
    QWidget(parent), m_dialogParameters(dialogParameters)
{
    m_headerLabel = new QLabel("ROS2 UTILS TOOL");
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

    const auto createToolButton = [this] (const QString& buttonText,
                                          const QString& tooltipText = "",
                                          const std::optional<Utils::UI::TOOL_ID>& toolId = std::nullopt) {
        QPointer<QToolButton> toolButton = new QToolButton;
        toolButton->setText(buttonText);
        toolButton->setToolTip(tooltipText);
        toolButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton->setIconSize(QSize(100, 45));
        toolButton->setFixedSize(QSize(150, 150));

        Utils::UI::setWidgetFontSize(toolButton, true);
        if (!toolId) {
            return toolButton;
        }
        // Only actual tool buttons need to do this
        QObject::connect(toolButton, &QToolButton::clicked, this, [this, toolId] {
            emit toolRequested(toolId.value());
        });
        return toolButton;
    };
    const auto createDualButtonLayout = [] (QPointer<QToolButton> leftButton, QPointer<QToolButton> rightButton) {
        auto* const layout = new QHBoxLayout;

        layout->addStretch();
        layout->addWidget(leftButton);
        if (rightButton) {
            layout->addWidget(rightButton);
        }
        layout->addStretch();

        return layout;
    };

    // Create five widgets: One for providing the overview for bag and publishing tools,
    // one for conversion, one for bag, one for publishing and one for info tools
    // Overview widget
    m_conversionToolsButton = createToolButton("Conversion\nTools");
    m_bagToolsButton = createToolButton("Bag Tools");
    m_publishingToolsButton = createToolButton("Publishing\nTools");
    m_infoToolsButton = createToolButton("Info\nTools");

    auto* const overallToolsMainLayout = new QVBoxLayout;
    overallToolsMainLayout->addLayout(createDualButtonLayout(m_conversionToolsButton, m_bagToolsButton));
    overallToolsMainLayout->addLayout(createDualButtonLayout(m_publishingToolsButton, m_infoToolsButton));

    auto* const overallToolsWidget = new QWidget;
    overallToolsWidget->setLayout(overallToolsMainLayout);

    // Conversion tools widget
    m_bagToVideoPushButton = createToolButton("Bag to Video", "Convert images in a ROS bag video topic to a video file.", Utils::UI::TOOL_ID::BAG_TO_VIDEO);
    m_videoToBagPushButton = createToolButton("Video to Bag", "Convert a video file to a ROS bag.", Utils::UI::TOOL_ID::VIDEO_TO_BAG);
    m_bagToPCDsPushButton = createToolButton("Bag to\nPCD Files", "Convert point clouds in a ROS bag topic to a set of pcd files.", Utils::UI::TOOL_ID::BAG_TO_PCDS);
    m_PCDsToBagPushButton = createToolButton("PCD Files\nto Bag", "Convert a set of pcd files to a ROS bag.", Utils::UI::TOOL_ID::PCDS_TO_BAG);
    m_bagToImagesPushButton = createToolButton("Bag to Images", "Convert images in a ROS bag video topic to a set of image files.", Utils::UI::TOOL_ID::BAG_TO_IMAGES);
    m_tf2ToFilePushButton = createToolButton("Bag TF2\nto File", "Convert transformations in a ROS bag tf2 topic to file.", Utils::UI::TOOL_ID::BAG_TF2_TO_FILE);
    m_bagMessageToFilePushButton = createToolButton("Bag Message\nto File", "Convert bag topic messages to file.", Utils::UI::TOOL_ID::BAG_MESSAGE_TO_FILE);

    auto* const conversionToolsMainLayout = new QVBoxLayout;
    conversionToolsMainLayout->addStretch();
    conversionToolsMainLayout->addLayout(createDualButtonLayout(m_bagToVideoPushButton, m_videoToBagPushButton));
    conversionToolsMainLayout->addLayout(createDualButtonLayout(m_bagToPCDsPushButton, m_PCDsToBagPushButton));
    conversionToolsMainLayout->addLayout(createDualButtonLayout(m_bagToImagesPushButton, m_tf2ToFilePushButton));
    conversionToolsMainLayout->addLayout(createDualButtonLayout(m_bagMessageToFilePushButton, nullptr));
    conversionToolsMainLayout->addStretch();

    auto* const conversionToolsWidget = new QWidget;
    conversionToolsWidget->setLayout(conversionToolsMainLayout);

    // Bag tools widget
    m_editBagButton = createToolButton("Edit Bag", "Rename, remove and crop topics in a ROS bag.", Utils::UI::TOOL_ID::EDIT_BAG);
    m_mergeBagsButton = createToolButton("Merge Bags", "Merge selected topics of two ROS bag files into a new one.", Utils::UI::TOOL_ID::MERGE_BAGS);
    m_recordBagButton = createToolButton("Record Bag", "Record selected topics into a bag file.", Utils::UI::TOOL_ID::RECORD_BAG);
    m_dummyBagButton = createToolButton("Create\nDummy Bag", "Create a ROS bag file with dummy data.", Utils::UI::TOOL_ID::DUMMY_BAG);
    m_compressBagButton = createToolButton("Compress\nBag", "Decrease a ROS bag by creating a compressed variant.", Utils::UI::TOOL_ID::COMPRESS_BAG);
    m_decompressBagButton = createToolButton("Decompress\nBag", "Decompress a compressed ROS bag.", Utils::UI::TOOL_ID::DECOMPRESS_BAG);
    m_playBagButton = createToolButton("Play Bag", "Play a ROS bag.", Utils::UI::TOOL_ID::PLAY_BAG);

    auto* const bagToolsMainLayout = new QVBoxLayout;
    bagToolsMainLayout->addStretch();
    bagToolsMainLayout->addLayout(createDualButtonLayout(m_editBagButton, m_mergeBagsButton));
    bagToolsMainLayout->addLayout(createDualButtonLayout(m_recordBagButton, m_dummyBagButton));
    bagToolsMainLayout->addLayout(createDualButtonLayout(m_compressBagButton, m_decompressBagButton));
    bagToolsMainLayout->addLayout(createDualButtonLayout(m_playBagButton, nullptr));
    bagToolsMainLayout->addStretch();

    auto* const bagToolsWidget = new QWidget;
    bagToolsWidget->setLayout(bagToolsMainLayout);

    // Publishing tools widget
    m_publishVideoButton = createToolButton("Publish Video\nas ROS Topic", "Publish video file images as a ROS image topic.", Utils::UI::TOOL_ID::PUBLISH_VIDEO);
    m_publishImagesButton = createToolButton("Publish Images\nas ROS Topic", "Publish a set of image files as a ROS image topic.", Utils::UI::TOOL_ID::PUBLISH_IMAGES);
    m_sendTF2Button = createToolButton("Send TF2\nMessage", "Send a tf2 message to /tf or /tf_static.", Utils::UI::TOOL_ID::SEND_TF2);

    auto* const publishingToolsMainLayout = new QVBoxLayout;
    publishingToolsMainLayout->addStretch();
    publishingToolsMainLayout->addLayout(createDualButtonLayout(m_publishVideoButton, m_publishImagesButton));
    publishingToolsMainLayout->addLayout(createDualButtonLayout(m_sendTF2Button, nullptr));
    publishingToolsMainLayout->addStretch();

    auto* const publishingToolsWidget = new QWidget;
    publishingToolsWidget->setLayout(publishingToolsMainLayout);

    // Info tools widget
    m_topicServiceInfoButton = createToolButton("Topics and\nService Info",
                                                "Show available topics and services with additional information.",
                                                Utils::UI::TOOL_ID::TOPICS_SERVICES_INFO);
    m_bagInfoButton = createToolButton("Bag\nInfos", "Show information for a selected ROS bag.", Utils::UI::TOOL_ID::BAG_INFO);

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

    m_versionLabel = new QLabel("v1.1.2");
    m_versionLabel->setToolTip("Bug fixes and build system improvements.");

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

    const auto switchToOverallTools = [this, conversionToolsWidget, bagToolsWidget, publishingToolsWidget, infoToolsWidget, overallToolsWidget] {
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
    settingsDialog->setAttribute(Qt::WA_DeleteOnClose);
    settingsDialog->exec();
}


// Used to switch between the four overall widgets
void
StartWidget::replaceWidgets(QWidget* fromWidget, QWidget* toWidget, int widgetIdentifier, bool otherItemVisibility)
{
    // If the back button is visible, the other elements should be hidden and vice versa
    m_backButton->setVisible(!otherItemVisibility);
    m_versionLabel->setVisible(otherItemVisibility);
    m_widgetOnInstantiation = widgetIdentifier;

    switch (m_widgetOnInstantiation) {
    case WIDGET_OVERALL:
        m_headerLabel->setText("ROS2 UTILS TOOL");
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


void
StartWidget::setButtonIcons()
{
    const auto isDarkMode = Utils::UI::isDarkMode();
    m_settingsButton->setIcon(QIcon(isDarkMode ? ":/icons/widgets/gear_white.svg" : ":/icons/widgets/gear_black.svg"));

    m_conversionToolsButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/conversion_tools_white.svg" : ":/icons/tools/conversion_tools_black.svg"));
    m_bagToolsButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/bag_tools_white.svg" : ":/icons/tools/bag_tools_black.svg"));
    m_publishingToolsButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/publishing_tools_white.svg" : ":/icons/tools/publishing_tools_black.svg"));
    m_infoToolsButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/info_tools_white.svg" : ":/icons/tools/info_tools_black.svg"));

    m_bagToVideoPushButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/bag_to_video_white.svg" : ":/icons/tools/bag_to_video_black.svg"));
    m_videoToBagPushButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/video_to_bag_white.svg" : ":/icons/tools/video_to_bag_black.svg"));
    m_bagToPCDsPushButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/bag_to_pcd_white.svg" : ":/icons/tools/bag_to_pcd_black.svg"));
    m_PCDsToBagPushButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/pcd_to_bag_white.svg" : ":/icons/tools/pcd_to_bag_black.svg"));
    m_bagToImagesPushButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/bag_to_images_white.svg" : ":/icons/tools/bag_to_images_black.svg"));
    m_tf2ToFilePushButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/bag_tf2_to_file_white.svg" : ":/icons/tools/bag_tf2_to_file_black.svg"));
    m_bagMessageToFilePushButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/bag_message_to_file_white.svg" : ":/icons/tools/bag_message_to_file_black.svg"));

    m_editBagButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/edit_bag_white.svg" : ":/icons/tools/edit_bag_black.svg"));
    m_mergeBagsButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/merge_bags_white.svg" : ":/icons/tools/merge_bags_black.svg"));
    m_recordBagButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/record_bag_white.svg" : ":/icons/tools/record_bag_black.svg"));
    m_dummyBagButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/dummy_bag_white.svg" : ":/icons/tools/dummy_bag_black.svg"));
    m_compressBagButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/compress_bag_white.svg" : ":/icons/tools/compress_bag_black.svg"));
    m_decompressBagButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/decompress_bag_white.svg" : ":/icons/tools/decompress_bag_black.svg"));
    m_playBagButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/play_bag_white.svg" : ":/icons/tools/play_bag_black.svg"));

    m_publishVideoButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/publish_video_white.svg" : ":/icons/tools/publish_video_black.svg"));
    m_publishImagesButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/publish_images_white.svg" : ":/icons/tools/publish_images_black.svg"));
    m_sendTF2Button->setIcon(QIcon(isDarkMode ? ":/icons/tools/send_tf2_white.svg" : ":/icons/tools/send_tf2_black.svg"));

    m_topicServiceInfoButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/topics_services_info_white.svg"
                                                       : ":/icons/tools/topics_services_info_black.svg"));
    m_bagInfoButton->setIcon(QIcon(isDarkMode ? ":/icons/tools/bag_info_white.svg" : ":/icons/tools/bag_info_black.svg"));
}


bool
StartWidget::event(QEvent *event)
{
    [[unlikely]] if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setButtonIcons();
    }
    return QWidget::event(event);
}
