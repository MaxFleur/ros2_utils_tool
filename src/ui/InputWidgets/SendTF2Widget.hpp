#pragma once

#include "BasicInputWidget.hpp"
#include "Parameters.hpp"
#include "SendTF2Settings.hpp"

#include <QPointer>
#include <QWidget>

class NodeWrapper;

class QCheckBox;
class QFormLayout;
class QLabel;
class QSpinBox;
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
    okButtonPressed() const;

private:
    QPointer<QCheckBox> m_isStaticCheckBox;
    QPointer<QSpinBox> m_rateSpinBox;
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
