#pragma once

#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QLabel;
class QPushButton;
class QToolButton;
class QVBoxLayout;

// The starting widget showing all available ui tools
class StartWidget : public QWidget
{
    Q_OBJECT
public:
    explicit
    StartWidget(Utils::UI::DialogParameters& dialogParameters,
                QWidget*                     parent = 0);

signals:
    void
    toolRequested(int id);

private slots:
    void
    openSettingsDialog();

private:
    void
    replaceWidgets(QWidget* fromWidget,
                   QWidget* toWidget,
                   int      widgetIdentifier,
                   bool     otherItemVisibility);

    QPointer<QToolButton>
    createToolButton(const QString& buttonText);

    void
    setButtonIcons();

    bool
    event(QEvent *event) override;

private:
    QPointer<QLabel> m_headerLabel;

    // Buttons for tools
    QPointer<QToolButton> m_conversionToolsButton;
    QPointer<QToolButton> m_bagToolsButton;
    QPointer<QToolButton> m_publishingToolsButton;

    QPointer<QToolButton> m_bagToVideoPushButton;
    QPointer<QToolButton> m_videoToBagPushButton;
    QPointer<QToolButton> m_bagToPCDsPushButton;
    QPointer<QToolButton> m_PCDsToBagPushButton;
    QPointer<QToolButton> m_bagToImagesPushButton;

    QPointer<QToolButton> m_editBagButton;
    QPointer<QToolButton> m_mergeBagsButton;
    QPointer<QToolButton> m_dummyBagButton;
    QPointer<QToolButton> m_bagInfoButton;

    QPointer<QToolButton> m_publishVideoButton;
    QPointer<QToolButton> m_publishImagesButton;

    // Widgets for other elements
    QPointer<QPushButton> m_settingsButton;
    QPointer<QPushButton> m_backButton;
    QPointer<QLabel> m_versionLabel;

    QPointer<QVBoxLayout> m_mainLayout;

    Utils::UI::DialogParameters& m_dialogParameters;

    // Used to remember which widget was active when we switch to the input widget, but cancel
    inline static int m_widgetOnInstantiation = 0;

    static constexpr int WIDGET_OVERALL = 0;
    static constexpr int WIDGET_CONVERSION = 1;
    static constexpr int WIDGET_BAG = 2;
    static constexpr int WIDGET_PUBLISHING = 3;
};
