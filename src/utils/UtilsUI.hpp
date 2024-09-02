#pragma once

#include <QComboBox>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QPointer>
#include <QToolButton>
#include <QWidget>

// Util functions for user interface related things
namespace Utils::UI
{
struct BasicParameters {
    QString bagDirectory = "";
    QString topicName = "";
};
struct VideoParameters : BasicParameters {
    QString videoDirectory = "";
    bool    useHardwareAcceleration = false;
};
struct ImageParameters : BasicParameters {
    QString imagesDirectory = "";
    QString format = "jpg";
    int     quality = 8;
};
struct PlayBagParameters : BasicParameters {
    bool loop = false;
};

// Create a larger font for a certain widget
void
setWidgetHeaderFont(QWidget* widget);

[[maybe_unused]] bool
fillComboBoxWithTopics(QPointer<QComboBox> comboBox,
                       const QString&      bagDirectory);

// Creates a layout of a lineedit along with a tool button
[[nodiscard]] QHBoxLayout*
createLineEditButtonLayout(QPointer<QLineEdit>   lineEdit,
                           QPointer<QToolButton> toolButton);

// Checks if the application is using a dark mode
[[nodiscard]] bool
isDarkMode();
}
