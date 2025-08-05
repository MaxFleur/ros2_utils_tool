#include "UtilsUI.hpp"

#include "DialogSettings.hpp"
#include "UtilsROS.hpp"

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
    const auto headerText = "Renamed topic name(s) invalid!";
    const auto mainText = "The renamed topic name(s) do not follow the ROS2 naming convention! More information can be found here:<br>"
                          "<a href='https://design.ros2.org/articles/topic_and_service_names.html'>https://design.ros2.org/articles/topic_and_service_names.html</a><br>"
                          "Do you still want to continue?";
    auto *const msgBox = new QMessageBox(QMessageBox::Warning, headerText, mainText, QMessageBox::Yes | QMessageBox::No);
    return msgBox->exec() == QMessageBox::Yes;
}


void
createCriticalMessageBox(const QString& headerText, const QString& mainText)
{
    auto *const msgBox = new QMessageBox(QMessageBox::Critical, headerText, mainText, QMessageBox::Ok);
    msgBox->exec();
}


bool
continueForExistingTarget(const QString& targetDirectory, const QString& headerTextBeginning,
                          const QString& targetIdentifier)
{
    if (!DialogSettings::getStaticParameter("warn_target_overwrite", true)) {
        return true;
    }

    if (std::filesystem::exists(targetDirectory.toStdString())) {
        auto* const checkBox = new QCheckBox("Don't show this again");

        auto *const msgBox = new QMessageBox(QMessageBox::Warning, headerTextBeginning + " already exists!",
                                             "The specified " + targetIdentifier + " already exists! Are you sure you want to continue? "
                                             "This will overwrite all target files.",
                                             QMessageBox::Yes | QMessageBox::No);
        msgBox->setCheckBox(checkBox);
        QObject::connect(checkBox, &QCheckBox::stateChanged, [] (int state) {
            DialogSettings::writeStaticParameter("warn_target_overwrite", state == Qt::Unchecked);
        });

        if (const auto ret = msgBox->exec(); ret == QMessageBox::No) {
            return false;
        }
    }

    return true;
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
