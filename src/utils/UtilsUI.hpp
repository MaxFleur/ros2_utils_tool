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
// Parameters used to configure the threads and
// store information in case an input widget is closed and reopened
struct BasicParameters {
    virtual
    ~BasicParameters() = default;

    QString sourceDirectory = "";
    QString topicName = "";
};
struct DummyBagParameters : BasicParameters {
    struct DummyBagTopic {
        QString type;
        QString name;
    };

    QVector<DummyBagTopic> topics = {};
    int                    messageCount = 100;
};

struct AdvancedParameters : BasicParameters {
    QString targetDirectory = "";
    bool    showAdvancedOptions = false;
};
struct BagToImagesParameters : AdvancedParameters {
    QString format = "jpg";
    int     quality = 8;
    bool    exchangeRedBlueValues = false;
    bool    useBWImages = false;
    bool    jpgOptimize = false;
    bool    pngBilevel = false;
};
struct BagToVideoParameters : AdvancedParameters {
    QString format = "mp4";
    int     fps = 30;
    bool    useHardwareAcceleration = false;
    bool    exchangeRedBlueValues = false;
    bool    useBWImages = false;
    bool    lossless = false;
};
struct VideoToBagParameters : AdvancedParameters {
    int  fps = 30;
    bool useCustomFPS = false;
    bool useHardwareAcceleration = false;
    bool exchangeRedBlueValues = false;
};
struct EditBagParameters : AdvancedParameters {
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
struct MergeBagsParameters : AdvancedParameters {
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

struct PublishParameters : AdvancedParameters {
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
    bool usePredefinedTopicNames = true;
    bool checkROS2NameConform = false;
};

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
static constexpr int TOOL_PUBLISH_VIDEO = 9;
static constexpr int TOOL_PUBLISH_IMAGES = 10;
}
