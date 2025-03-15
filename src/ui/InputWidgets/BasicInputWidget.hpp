#pragma once

#include "BasicSettings.hpp"

#include <QPointer>
#include <QWidget>

class QDialogButtonBox;
class QHBoxLayout;
class QLabel;
class QLineEdit;
class QPushButton;
class QToolButton;

template<typename T, typename U>
concept InputParameterSetting = (std::same_as<T, U> &&
                                 (std::same_as<T, QString> || std::same_as<T, int> ||
                                  std::same_as<T, size_t> || std::same_as<T, bool>));

// The basic input widget all other input widgets derive from
// Each input widget uses a corresponding parameter and settings member.
// The parameters are used to store all input for reusing if the widget is closed and opened again,
// while the settings are used to write the parameters to file in case the parameters should be
// used after closing and restarting the application.

// The basic widget provides a few members and functions used by all derived members.
class BasicInputWidget : public QWidget
{
    Q_OBJECT

public:
    // Set the header text and top icon automatically
    BasicInputWidget(const QString& headerText,
                     const QString& iconPath,
                     QWidget*       parent = 0);

signals:
    void
    back();

    void
    okPressed();

protected:
    void
    enableOkButton(bool enable);

    void
    setPixmapLabelIcon();

    bool
    event(QEvent *event) override;

    template<typename T, typename U>
    requires InputParameterSetting<T, U>
    void
    writeParameterToSettings(T&             settingsParameter,
                             const U&       newValue,
                             BasicSettings& inputSettings)
    {
        settingsParameter = newValue;
        inputSettings.write();
    }

protected:
    QPointer<QLabel> m_headerLabel;
    QPointer<QLabel> m_headerPixmapLabel;
    // All input widgets need some sort of source file, provide it here
    QPointer<QLineEdit> m_sourceLineEdit;
    QPointer<QToolButton> m_findSourceButton;
    // Also need an ok and cancel button
    QPointer<QPushButton> m_backButton;
    QPointer<QPushButton> m_okButton;

    QPointer<QHBoxLayout> m_findSourceLayout;
    QPointer<QHBoxLayout> m_buttonLayout;
    QPointer<QDialogButtonBox> m_dialogButtonBox;

    QString m_iconPath;
};
