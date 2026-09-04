#include "ConfigureRecordBagWidget.hpp"

#include "BagTreeWidget.hpp"
#include "LowDiskSpaceWidget.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QSpinBox>
#include <QVBoxLayout>

ConfigureRecordBagWidget::ConfigureRecordBagWidget(Parameters::RecordBagParameters& parameters, QWidget *parent) :
    BasicBagWidget(parameters, "Record Bag", ":/icons/tools/record_bag", "record_bag", "Unselect all Topics you don't want to record.", parent),
    m_parameters(parameters), m_settings(parameters, "record_bag")
{
    auto* const includeROSTopicsCheckBox = new QCheckBox("Include ROS Topics");
    includeROSTopicsCheckBox->setCheckState(m_parameters.includeROSTopics ? Qt::Checked : Qt::Unchecked);
    includeROSTopicsCheckBox->setToolTip("Includes topics associated with ROS, for example '/parameter_events'.");

    m_refreshButton = new QPushButton("Refresh List");
    m_refreshButton->setVisible(false);

    auto* const sourceFormLayout = new QFormLayout;
    sourceFormLayout->addRow("Bag Location:", m_findSourceLayout);

    auto* const refreshButtonLayout = new QHBoxLayout;
    refreshButtonLayout->addWidget(includeROSTopicsCheckBox);
    refreshButtonLayout->addStretch();
    refreshButtonLayout->addWidget(m_refreshButton);

    m_lowDiskSpaceWidget = new LowDiskSpaceWidget;

    auto* const advancedOptionsCheckBox = new QCheckBox;
    advancedOptionsCheckBox->setChecked(m_parameters.showAdvancedOptions);
    advancedOptionsCheckBox->setText("Show Advanced Options");

    const auto createCheckBox = [this] (const QString& toolTip, bool& enableValue) {
        auto* const checkBox = new QCheckBox;
        checkBox->setCheckState(enableValue ? Qt::Checked : Qt::Unchecked);
        checkBox->setToolTip(toolTip);

        connect(checkBox, &QCheckBox::stateChanged, this, [this, &enableValue] (int state) {
            writeParameterToSettings(enableValue, state == Qt::Checked, m_settings);
        });

        return checkBox;
    };
    const auto createLayout = [this, createCheckBox] (const QString& toolTip, int maximumSpinBoxRange, int& spinBoxValueToSave, bool& enableValue) {
        auto* const spinBox = new QSpinBox;
        spinBox->setRange(0, maximumSpinBoxRange);
        spinBox->setValue(spinBoxValueToSave);
        spinBox->setEnabled(enableValue);
        spinBox->setToolTip(toolTip);

        auto* const checkBox = createCheckBox(toolTip, enableValue);

        auto* const layout = new QHBoxLayout;
        layout->addWidget(checkBox);
        layout->addWidget(spinBox);
        layout->addStretch();

        connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this, &spinBoxValueToSave] (int value) {
            writeParameterToSettings(spinBoxValueToSave, value, m_settings);
        });
        connect(checkBox, &QCheckBox::stateChanged, this, [spinBox] (int state) {
            spinBox->setEnabled(state == Qt::Checked);
        });

        return layout;
    };


    auto* const sizeLayout = createLayout("Split the bag file if a certain size is reached.", 102400, m_parameters.maxSizeInMB, m_parameters.useCustomSize);
    auto* const durationLayout = createLayout("Split the bag file if a certain time is reached.", 7200, m_parameters.maxDurationInSeconds, m_parameters.useCustomDuration);

    auto* const spacerWidget = new QWidget;
    spacerWidget->setFixedHeight(5);

    auto* const includeHiddenTopicsCheckBox = createCheckBox("Whether to include topics not publically shown.", m_parameters.includeHiddenTopics);
    auto* const includeUnpublishedTopicsCheckBox = createCheckBox("Whether to include topics where nothing has been published so far.", m_parameters.includeUnpublishedTopics);

    auto* const secondSpacerWidget = new QWidget;
    secondSpacerWidget->setFixedHeight(5);

    auto* const noCompressionRadioButton = new QRadioButton("None");
    auto* const compressPerFileRadioButton = new QRadioButton("Per File");
    auto* const compressPerMessageRadioButton = new QRadioButton("Per Message");
    if (m_parameters.useCompression) {
        m_parameters.isCompressionFile ? compressPerFileRadioButton->setChecked(true) : compressPerMessageRadioButton->setChecked(true);
    } else {
        noCompressionRadioButton->setChecked(true);
    }

    auto* const advancedOptionsFormLayout = new QFormLayout;
    advancedOptionsFormLayout->addRow("Use Maximum Size (in MB):", sizeLayout);
    advancedOptionsFormLayout->addRow("Use Maximum Time (in Seconds):", durationLayout);
    advancedOptionsFormLayout->addRow("", spacerWidget);
    advancedOptionsFormLayout->addRow("Include Hidden Topics:", includeHiddenTopicsCheckBox);
    advancedOptionsFormLayout->addRow("Include Unpublished Topics:", includeUnpublishedTopicsCheckBox);
    advancedOptionsFormLayout->addRow("", secondSpacerWidget);
    advancedOptionsFormLayout->addRow("Compression:", noCompressionRadioButton);
    advancedOptionsFormLayout->addRow("", compressPerFileRadioButton);
    advancedOptionsFormLayout->addRow("", compressPerMessageRadioButton);

    auto* const advancedOptionsWidget = new QWidget;
    advancedOptionsWidget->setLayout(advancedOptionsFormLayout);
    advancedOptionsWidget->setVisible(m_parameters.showAdvancedOptions);

    m_controlsLayout->addSpacing(30);
    m_controlsLayout->addWidget(m_unselectLabel);
    m_controlsLayout->addWidget(m_treeWidget);
    m_controlsLayout->addLayout(refreshButtonLayout);
    m_controlsLayout->addSpacing(5);
    m_controlsLayout->addLayout(sourceFormLayout);
    m_controlsLayout->addSpacing(5);
    m_controlsLayout->addWidget(m_lowDiskSpaceWidget);
    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addWidget(advancedOptionsCheckBox);
    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addWidget(advancedOptionsWidget);
    m_controlsLayout->addStretch();

    connect(includeROSTopicsCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.includeROSTopics, state == Qt::Checked, m_settings);
    });
    connect(m_refreshButton, &QPushButton::clicked, this, &ConfigureRecordBagWidget::populateTreeWidget);
    connect(advancedOptionsCheckBox, &QCheckBox::stateChanged, this, [this, advancedOptionsWidget] (int state) {
        writeParameterToSettings(m_parameters.showAdvancedOptions, state == Qt::Checked, m_settings);
        advancedOptionsWidget->setVisible(state == Qt::Checked);
    });
    connect(noCompressionRadioButton, &QRadioButton::clicked, this, [this] {
        writeParameterToSettings(m_parameters.useCompression, false, m_settings);
    });
    connect(compressPerFileRadioButton, &QRadioButton::clicked, this, [this] {
        writeParameterToSettings(m_parameters.useCompression, true, m_settings);
        writeParameterToSettings(m_parameters.isCompressionFile, true, m_settings);
    });
    connect(compressPerMessageRadioButton, &QRadioButton::clicked, this, [this] {
        writeParameterToSettings(m_parameters.useCompression, true, m_settings);
        writeParameterToSettings(m_parameters.isCompressionFile, false, m_settings);
    });

    setPixmapLabelIcon();
    setLowDiskSpaceWidgetVisibility(m_sourceLineEdit->text());
    populateTreeWidget();
}


void
ConfigureRecordBagWidget::handleTreeAfterSource()
{
    enableOkButton();
}


void
ConfigureRecordBagWidget::okButtonPressed() const
{
    if (!m_okButton->isEnabled()) {
        return;
    }
    if (const auto sufficientSpace = showLowDiskSpaceMessageBox(); !sufficientSpace) {
        return;
    }
    if (!Utils::UI::continueForExistingTarget(m_parameters.sourceDirectory, "Bag file", "bag file")) {
        return;
    }

    emit okPressed();
}


void
ConfigureRecordBagWidget::populateTreeWidget()
{
    m_parameters.services.clear();
    m_parameters.topics.clear();
    m_treeWidget->clear();
    m_treeWidget->blockSignals(true);

    // Topics
    const auto& currentTopicsAndTypes = Utils::ROS::getTopicInformation();
    for (const auto& topic : currentTopicsAndTypes) {
        // Ignore ROS's own topics
        if ((QString::fromStdString(topic.first) == "/parameter_events" || QString::fromStdString(topic.first) == "/rosout" ||
             QString::fromStdString(topic.first) == "/events/read_split" || QString::fromStdString(topic.first) == "/events/write_split" ||
             topic.first.ends_with("/_service_event")) && !m_parameters.includeROSTopics) {
            continue;
        }

        m_treeWidget->createItemWithTopicNameAndType(QString::fromStdString(topic.first), QString::fromStdString(topic.second.at(0)), true);
        m_parameters.topics.push_back({ { QString::fromStdString(topic.first) }, true });
    }
    // Services
    const auto& currentServices = Utils::ROS::getServiceNamesAndTypes();
    // ROS will automatically create services with these appendices, but they are not important for us, so exclude them
    const std::vector<std::string> exclusionList { "/describe_parameters", "/get_parameter_types", "/get_parameters", "/get_type_description",
                                                   "/list_parameters", "/set_parameters", "/set_parameters_atomically" };
    for (const auto& service : currentServices) {
        // Also ignore these services
        if (service.first.starts_with("/services_node") || service.first.starts_with("/rosbag2_player") ||
            service.first.starts_with("/rosbag2_recorder")) {
            continue;
        }
        const auto foundExclusionAppendix = std::ranges::any_of(exclusionList, [&service] (const std::string& exclusionAppendix) {
            return service.first.ends_with(exclusionAppendix);
        });
        if (foundExclusionAppendix) {
            continue;
        }

        m_treeWidget->createItemWithTopicNameAndType(QString::fromStdString(service.first), QString::fromStdString(service.second.at(0)), true);
        m_parameters.services.push_back({ { QString::fromStdString(service.first) }, true });
    }
    m_settings.write();

    m_treeWidget->resizeColumns();
    // Just take a random item to get its height
    auto* item = m_treeWidget->topLevelItem(m_treeWidget->topLevelItemCount() - 1);
    const auto height = m_treeWidget->visualItemRect(item).height();
    m_treeWidget->setMinimumHeight((height * m_treeWidget->topLevelItemCount()) + HEIGHT_OFFSET);
    m_treeWidget->blockSignals(false);

    m_unselectLabel->setVisible(true);
    m_treeWidget->setVisible(true);
    m_refreshButton->setVisible(true);

    enableOkButton();
}


void
ConfigureRecordBagWidget::enableOkButton()
{
    const auto isAnyTopicEnabled = std::ranges::any_of(m_parameters.topics, [] (const auto& topic) {
        return topic.isSelected == true;
    });
    const auto isAnyServiceEnabled = std::ranges::any_of(m_parameters.services, [] (const auto& service) {
        return service.isSelected == true;
    });
    m_okButton->setEnabled((isAnyTopicEnabled || isAnyServiceEnabled) && !m_sourceLineEdit->text().isEmpty());
}
