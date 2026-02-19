#include "UtilsUI.hpp"

#include "DialogSettings.hpp"
#include "UtilsROS.hpp"

#include <QFileDialog>
#include <QMessageBox>

#include <filesystem>

namespace Utils::UI
{
void
setWidgetFontSize(QWidget* widget, bool isButton)
{
    auto font = widget->font();
    font.setPointSize(isButton ? FONT_SIZE_BUTTON : FONT_SIZE_HEADER);
    widget->setFont(font);
}


bool
fillComboBoxWithTopics(QPointer<QComboBox> comboBox, const QString& bagDirectory, const QString& topicType)
{
    const auto videoTopics = Utils::ROS::getBagTopics(bagDirectory, topicType);
    if (videoTopics.empty()) {
        return false;
    }

    comboBox->clear();
    for (const auto& videoTopic : videoTopics) {
        comboBox->addItem(videoTopic);
    }

    return true;
}


QCheckBox*
createCheckBox(const QString& toolTipText, bool checkState)
{
    auto* const checkBox = new QCheckBox;
    checkBox->setToolTip(toolTipText);
    checkBox->setCheckState(checkState ? Qt::Checked : Qt::Unchecked);

    return checkBox;
}


QHBoxLayout*
createLineEditButtonLayout(QPointer<QLineEdit> lineEdit, QPointer<QToolButton> toolButton)
{
    // Do not let the user add anything, stick to specific dialogs with file directories
    lineEdit->setReadOnly(true);
    toolButton->setText("...");

    auto* const layout = new QHBoxLayout;
    layout->addWidget(lineEdit);
    layout->addWidget(toolButton);

    return layout;
}


bool
continueWithInvalidROS2Names()
{
    if (!DialogSettings::getStaticParameter("warn_ros2_name_convention", true)) {
        return true;
    }

    const auto headerText = "Renamed topic name(s) invalid!";
    const auto mainText = "The renamed topic name(s) do not follow the ROS2 naming convention! More information can be found here:<br>"
                          "<a href='https://design.ros2.org/articles/topic_and_service_names.html'>https://design.ros2.org/articles/topic_and_service_names.html</a><br>"
                          "Do you still want to continue?";

    auto *const msgBox = new QMessageBox(QMessageBox::Warning, headerText, mainText, QMessageBox::Yes | QMessageBox::No);

    auto* const checkBox = createMessageBoxCheckBox("warn_ros2_name_convention");
    msgBox->setCheckBox(checkBox);

    return msgBox->exec() == QMessageBox::Yes;
}


bool
continueForExistingTarget(const QString& targetDirectory, const QString& headerTextBeginning,
                          const QString& targetIdentifier)
{
    if (!DialogSettings::getStaticParameter("warn_target_overwrite", true)) {
        return true;
    }

    if (std::filesystem::exists(targetDirectory.toStdString())) {
        auto *const msgBox = new QMessageBox(QMessageBox::Warning, headerTextBeginning + " already exists!",
                                             "The specified " + targetIdentifier + " already exists! Are you sure you want to continue? "
                                             "This will overwrite all target files.",
                                             QMessageBox::Yes | QMessageBox::No);

        auto* const checkBox = createMessageBoxCheckBox("warn_target_overwrite");
        msgBox->setCheckBox(checkBox);

        if (const auto ret = msgBox->exec(); ret == QMessageBox::No) {
            return false;
        }
    }

    return true;
}


QCheckBox*
createMessageBoxCheckBox(const QString& optionsIdentifier)
{
    auto* const checkBox = new QCheckBox("Don't show this again");

    QObject::connect(checkBox, &QCheckBox::stateChanged, [optionsIdentifier] (int state) {
        DialogSettings::writeStaticParameter(optionsIdentifier, state == Qt::Unchecked);
    });

    return checkBox;
}


void
createCriticalMessageBox(const QString& headerText, const QString& mainText)
{
    auto *const msgBox = new QMessageBox(QMessageBox::Critical, headerText, mainText, QMessageBox::Ok);
    msgBox->exec();
}


const QString
replaceTextAppendix(const QString& inputText, const QString& newAppendix)
{
    auto newText = inputText;
    newText.truncate(newText.lastIndexOf(QChar('.')));
    newText += "." + newAppendix;

    return newText;
}


std::optional<QString>
isBagDirectoryValid(QWidget* parent)
{
    const auto bagDirectory = QFileDialog::getExistingDirectory(parent, "Open Source Bag File", "",
                                                                QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (bagDirectory.isEmpty()) {
        return {};
    }
    if (!Utils::ROS::doesDirectoryContainBagFile(bagDirectory)) {
        createCriticalMessageBox("Invalid bag file!", "The source bag file seems to be invalid or broken!");
        return {};
    }

    return bagDirectory;
}


bool
isDarkMode()
{
    const QWidget widget;
    const auto color = widget.palette().color(QPalette::Window);
    const auto luminance = sqrt(0.299 * std::pow(color.redF(), 2) +
                                0.587 * std::pow(color.greenF(), 2) +
                                0.114 * std::pow(color.blueF(), 2));
    return luminance < 0.2;
}
}
