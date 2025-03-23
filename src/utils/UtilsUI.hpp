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

static constexpr int FONT_SIZE_HEADER = 16;
static constexpr int FONT_SIZE_BUTTON = 14;

static constexpr int TOOL_BAG_TO_VIDEO = 0;
static constexpr int TOOL_VIDEO_TO_BAG = 1;
static constexpr int TOOL_BAG_TO_PCDS = 2;
static constexpr int TOOL_PCDS_TO_BAG = 3;
static constexpr int TOOL_BAG_TO_IMAGES = 4;
static constexpr int TOOL_EDIT_BAG = 5;
static constexpr int TOOL_MERGE_BAGS = 6;
static constexpr int TOOL_DUMMY_BAG = 7;
static constexpr int TOOL_BAG_INFO = 8;
static constexpr int TOOL_COMPRESS_BAG = 9;
static constexpr int TOOL_DECOMPRESS_BAG = 10;
static constexpr int TOOL_PUBLISH_VIDEO = 11;
static constexpr int TOOL_PUBLISH_IMAGES = 12;
}
