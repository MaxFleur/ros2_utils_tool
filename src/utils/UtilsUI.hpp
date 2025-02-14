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
// Parameters used to store input widget input if the widget is closed and then reopened
// or if the application is restarted (if the user configured it like that)
struct InputParameters {
    virtual
    ~InputParameters() = default;

    QString sourceDirectory = "";
    QString topicName = "";
};
struct DummyBagInputParameters : InputParameters {
    struct DummyBagTopic {
        QString type;
        QString name;
    };

    QVector<DummyBagTopic> topics = {};
    int                    messageCount = 100;
};

struct AdvancedInputParameters : InputParameters {
    QString targetDirectory = "";
    bool    showAdvancedOptions = false;
};
struct ImageInputParameters : AdvancedInputParameters {
    QString format = "jpg";
    int     quality = 8;
    bool    exchangeRedBlueValues = false;
    bool    useBWImages = false;
    bool    jpgOptimize = false;
    bool    pngBilevel = false;
};
struct VideoInputParameters : AdvancedInputParameters {
    QString format = "mp4";
    int     fps = 30;
    bool    useHardwareAcceleration = false;
    bool    exchangeRedBlueValues = false;
    bool    useBWImages = false;
    bool    lossless = false;
};
struct BagInputParameters : AdvancedInputParameters {
    int  fps = 30;
    bool useCustomFPS = false;
    bool useHardwareAcceleration = false;
    bool exchangeRedBlueValues = false;
};
struct EditBagInputParameters : AdvancedInputParameters {
    struct EditBagTopic {
        QString renamedTopicName = "";
        QString originalTopicName;
        size_t  lowerBoundary = 0;
        size_t  upperBoundary;
        bool    isSelected = true;
    };

    QVector<EditBagTopic> topics = {};
    bool                  deleteSource = false;
    bool                  updateTimestamps = false;
};
struct MergeBagsInputParameters : AdvancedInputParameters {
    struct MergeBagTopic {
        QString name = "";
        // Topic names might be identical across bags, but cannot be identical in the same bag.
        // So store the bag directory as an additional identifier
        QString bagDir = "";
        bool    isSelected = true;
    };

    QVector<MergeBagTopic> topics = {};
    QString                secondSourceDirectory = "";
};

struct PublishParameters : AdvancedInputParameters {
    int  fps = 30;
    int  width = 1280;
    int  height = 720;
    bool exchangeRedBlueValues = false;
    bool loop = false;
    bool useHardwareAcceleration = false;
    bool scale = false;
};

struct DialogParameters {
    bool saveParameters = false;
    bool checkROS2NameConform = false;
};

// Create a larger font for some buttons and widget headers
void
setWidgetFontSize(QWidget* widget,
                  bool     isButton = false);

[[maybe_unused]] bool
fillComboBoxWithTopics(QPointer<QComboBox> comboBox,
                       const QString&      bagDirectory);

[[nodiscard]] QCheckBox*
createCheckBox(const QString& toolTipText,
               bool           checkState);

// Creates a layout of a lineedit along with a tool button
[[nodiscard]] QHBoxLayout*
createLineEditButtonLayout(QPointer<QLineEdit>   lineEdit,
                           QPointer<QToolButton> toolButton);

// Create a messagebox informing of invalid ROS2 topic names
[[nodiscard]] QMessageBox*
createInvalidROSNameMessageBox();

// Creates a messagebox informing of a critical error
void
createCriticalMessageBox(const QString& headerText,
                         const QString& mainText);

// Checks if the application is in dark mode
[[nodiscard]] bool
isDarkMode();

static constexpr int FONT_SIZE_HEADER = 16;
static constexpr int FONT_SIZE_BUTTON = 14;

static constexpr int TOOL_BAG_TO_VIDEO = 0;
static constexpr int TOOL_VIDEO_TO_BAG = 1;
static constexpr int TOOL_BAG_TO_IMAGES = 2;
static constexpr int TOOL_EDIT_BAG = 3;
static constexpr int TOOL_MERGE_BAGS = 4;
static constexpr int TOOL_DUMMY_BAG = 5;
static constexpr int TOOL_BAG_INFO = 6;
static constexpr int TOOL_PUBLISH_VIDEO = 7;
static constexpr int TOOL_PUBLISH_IMAGES = 8;
}
