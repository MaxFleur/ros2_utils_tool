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
    bool    switchRedBlueValues = false;
    bool    useBWImages = false;
    bool    jpgOptimize = false;
    bool    pngBilevel = false;
};
struct VideoInputParameters : AdvancedInputParameters {
    QString format = "mp4";
    int     fps = 30;
    bool    useHardwareAcceleration = false;
    bool    switchRedBlueValues = false;
    bool    useBWImages = false;
    bool    lossless = false;
};
struct BagInputParameters : AdvancedInputParameters {
    int  fps = 30;
    bool useCustomFPS = false;
    bool useHardwareAcceleration = false;
    bool switchRedBlueValues = false;
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

struct PublishParameters : AdvancedInputParameters {
    int  fps = 30;
    bool switchRedBlueValues = false;
    bool loop = false;
    bool useHardwareAcceleration = false;
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
}
