#pragma once

#include "Parameters.hpp"
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
    StartWidget(Parameters::DialogParameters& dialogParameters,
                QWidget*                      parent = 0);

signals:
    void
    toolRequested(Utils::UI::TOOL_ID id);

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
    createToolButton(const QString& buttonText,
                     const QString& tooltipText = "") const;

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
    QPointer<QToolButton> m_infoToolsButton;

    QPointer<QToolButton> m_bagToVideoPushButton;
    QPointer<QToolButton> m_videoToBagPushButton;
    QPointer<QToolButton> m_bagToPCDsPushButton;
    QPointer<QToolButton> m_PCDsToBagPushButton;
    QPointer<QToolButton> m_bagToImagesPushButton;
    QPointer<QToolButton> m_tf2ToFilePushButton;

    QPointer<QToolButton> m_editBagButton;
    QPointer<QToolButton> m_mergeBagsButton;
    QPointer<QToolButton> m_recordBagButton;
    QPointer<QToolButton> m_dummyBagButton;
    QPointer<QToolButton> m_compressBagButton;
    QPointer<QToolButton> m_decompressBagButton;

    QPointer<QToolButton> m_publishVideoButton;
    QPointer<QToolButton> m_publishImagesButton;
    QPointer<QToolButton> m_sendTF2Button;

    QPointer<QToolButton> m_topicServiceInfoButton;
    QPointer<QToolButton> m_bagInfoButton;

    // Widgets for other elements
    QPointer<QPushButton> m_settingsButton;
    QPointer<QPushButton> m_backButton;
    QPointer<QLabel> m_versionLabel;

    QPointer<QVBoxLayout> m_mainLayout;

    Parameters::DialogParameters& m_dialogParameters;

    // Used to remember which widget was active when we switch to the input widget, but cancel
    inline static int m_widgetOnInstantiation = 0;

    static constexpr int WIDGET_OVERALL = 0;
    static constexpr int WIDGET_CONVERSION = 1;
    static constexpr int WIDGET_BAG = 2;
    static constexpr int WIDGET_PUBLISHING = 3;
    static constexpr int WIDGET_INFO = 4;
};
