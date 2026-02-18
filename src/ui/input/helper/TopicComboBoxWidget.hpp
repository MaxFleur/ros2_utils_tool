#pragma once

#include "AdvancedInputWidget.hpp"

class QComboBox;

// Augments the advanced input widget with an additional topic combo box
class TopicComboBoxWidget : public AdvancedInputWidget
{
    Q_OBJECT

public:
    TopicComboBoxWidget(Parameters::AdvancedParameters& parameters,
                        const QString&                  headerText,
                        const QString&                  iconPath,
                        const QString&                  sourceFormLayoutName,
                        const QString&                  targetFormLayoutName,
                        const QString&                  settingsIdentifier,
                        int                             outputFormat,
                        QWidget*                        parent = 0);

protected slots:
    void
    fillTopicComboBox() override;

protected:
    QPointer<QComboBox> m_topicNameComboBox;
};
