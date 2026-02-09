#pragma once

#include "BasicInputWidget.hpp"
#include "Parameters.hpp"
#include "SendTF2Settings.hpp"

#include <QPointer>
#include <QWidget>

class NodeWrapper;

class QCheckBox;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;
class QLineEdit;
class QSpinBox;
class QToolButton;
class QTimer;

// Widget used to configure sending a TF2 message
class SendTF2Widget : public BasicInputWidget
{
    Q_OBJECT

public:
    SendTF2Widget(Parameters::SendTF2Parameters& parameters,
                  QWidget*                       parent = 0);

private slots:
    void
    staticCheckBoxPressed(int state);

    void
    saveToFileButtonPressed();

    void
    loadFromFileButtonPressed();

    void
    okButtonPressed();

private:
    void
    animateInfoLabel(const QString& labelText);

    // Have to overwrite this one because we are using more additional icons then just the top one
    void
    setPixmapLabelIcon() const;

    bool
    event(QEvent *event);

private:
    QPointer<QDoubleSpinBox> m_translationXSpinBox;
    QPointer<QDoubleSpinBox> m_translationYSpinBox;
    QPointer<QDoubleSpinBox> m_translationZSpinBox;
    QPointer<QDoubleSpinBox> m_rotationXSpinBox;
    QPointer<QDoubleSpinBox> m_rotationYSpinBox;
    QPointer<QDoubleSpinBox> m_rotationZSpinBox;
    QPointer<QDoubleSpinBox> m_rotationWSpinBox;

    QPointer<QLineEdit> m_childFrameNameLineEdit;
    QPointer<QCheckBox> m_isStaticCheckBox;
    QPointer<QSpinBox> m_rateSpinBox;

    QPointer<QToolButton> m_saveToFileButton;
    QPointer<QToolButton> m_loadFromFileButton;

    QPointer<QFormLayout> m_formLayout;

    QPointer<QLabel> m_transformSentLabel;
    QPointer<QTimer> m_timer;

    std::shared_ptr<NodeWrapper> m_nodeWrapper;

    Parameters::SendTF2Parameters& m_parameters;

    SendTF2Settings m_settings;

    static constexpr double SPINBOX_LOWER_RANGE = -1000.0;
    static constexpr double SPINBOX_UPPER_RANGE = 1000.0;
    static constexpr int NUMBER_OF_DECIMALS = 5;
    static constexpr int LABEL_SHOWN_DURATION = 1500;
};
