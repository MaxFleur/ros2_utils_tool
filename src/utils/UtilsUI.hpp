#pragma once

#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QPointer>
#include <QToolButton>
#include <QWidget>

// Util functions for user interface related things
namespace Utils::UI
{
// Create a larger font for some buttons and widget headers
void
setWidgetFontSize(QWidget* widget,
                  bool     isButton = false);

[[maybe_unused]] bool
fillComboBoxWithTopics(QPointer<QComboBox> comboBox,
                       const QString&      bagDirectory,
                       const QString&      topicType);

[[nodiscard]] QCheckBox*
createCheckBox(const QString& toolTipText,
               bool           checkState);

// Creates a layout of a lineedit along with a tool button
[[nodiscard]] QHBoxLayout*
createLineEditButtonLayout(QPointer<QLineEdit>   lineEdit,
                           QPointer<QToolButton> toolButton);

// Create a messagebox asking if a user should continue with invalid ROS2 names
[[nodiscard]] bool
continueWithInvalidROS2Names();

// Creates a messagebox informing of a critical error
void
createCriticalMessageBox(const QString& headerText,
                         const QString& mainText);

bool
continueForExistingTarget(const QString& targetDirectory,
                          const QString& headerTextBeginning,
                          const QString& targetIdentifier);

// Checks if the application is in dark mode
[[nodiscard]] bool
isDarkMode();

enum class TOOL_ID {
    BAG_TO_VIDEO,
    VIDEO_TO_BAG,
    BAG_TO_PCDS,
    PCDS_TO_BAG,
    BAG_TO_IMAGES,
    EDIT_BAG,
    MERGE_BAGS,
    RECORD_BAG,
    DUMMY_BAG,
    COMPRESS_BAG,
    DECOMPRESS_BAG,
    PUBLISH_VIDEO,
    PUBLISH_IMAGES,
    TOPICS_SERVICES_INFO,
    BAG_INFO
};

static constexpr int FONT_SIZE_HEADER = 16;
static constexpr int FONT_SIZE_BUTTON = 14;
}
