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
                        const OUTPUT_TYPE               outputType,
                        QWidget*                        parent = 0);

protected slots:
    // returns if the source directory contains any topics suitable for this tool
    bool
    fillTopicComboBox(const QString& sourceDirectory) override;

private:
    void
    mainFillOperation(const QString& sourceDirectory);

protected:
    QPointer<QComboBox> m_topicNameComboBox;
};
