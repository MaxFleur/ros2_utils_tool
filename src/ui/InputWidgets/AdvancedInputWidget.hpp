#pragma once

#include "AdvancedSettings.hpp"
#include "BasicInputWidget.hpp"
#include "Parameters.hpp"

class QComboBox;
class QFormLayout;
class QLineEdit;
class QVBoxLayout;

// Derived from basic input, provides functions to search for an input bag or a target directory
class AdvancedInputWidget : public BasicInputWidget
{
    Q_OBJECT

public:
    AdvancedInputWidget(Parameters::AdvancedParameters& parameters,
                        const QString&                  headerText,
                        const QString&                  iconPath,
                        const QString&                  sourceFormLayoutName,
                        const QString&                  targetFormLayoutName,
                        const QString&                  settingsIdentifier,
                        int                             outputFormat,
                        QWidget*                        parent = 0);

protected slots:
    virtual void
    findSourceButtonPressed();

    void
    findTargetButtonPressed();

    virtual void
    okButtonPressed() const;

    void
    setFileFormat(const QString& videoFormat)
    {
        m_fileFormat = videoFormat;
    }

protected:
    QPointer<QComboBox> m_topicNameComboBox;
    QPointer<QLineEdit> m_targetLineEdit;

    QPointer<QFormLayout> m_basicOptionsFormLayout;
    QPointer<QVBoxLayout> m_controlsLayout;

    static constexpr int OUTPUT_VIDEO = 0;
    static constexpr int OUTPUT_IMAGES = 1;
    static constexpr int OUTPUT_PCDS = 2;
    static constexpr int OUTPUT_JSON = 3;
    static constexpr int OUTPUT_BAG = 4;

private:
    Parameters::AdvancedParameters& m_parameters;

    AdvancedSettings m_settings;

    QString m_fileFormat;

    const int m_outputFormat;
};
