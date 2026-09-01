#pragma once

#include "AdvancedSettings.hpp"
#include "BasicInputWidget.hpp"
#include "Parameters.hpp"

class QFormLayout;
class QLineEdit;

// Derived from basic input, provides functions to search for an input bag or a target directory
class AdvancedInputWidget : public BasicInputWidget
{
    Q_OBJECT

protected:
    enum class OUTPUT_TYPE {
        OUTPUT_VIDEO,
        OUTPUT_IMAGES,
        OUTPUT_PCDS,
        OUTPUT_TF_TO_FILE,
        OUTPUT_YAML,
        OUTPUT_BAG,
        OUTPUT_BAG_EDITED,
        OUTPUT_BAG_MERGED,
        OUTPUT_BAG_COMPRESSED,
        OUTPUT_BAG_DECOMPRESSED
    };

public:
    AdvancedInputWidget(Parameters::AdvancedParameters& parameters,
                        const QString&                  headerText,
                        const QString&                  iconPath,
                        const QString&                  sourceFormLayoutName,
                        const QString&                  targetFormLayoutName,
                        const QString&                  settingsIdentifier,
                        OUTPUT_TYPE                     outputType,
                        QWidget*                        parent = 0);

protected slots:
    virtual void
    findSourceButtonPressed();

    void
    findTargetButtonPressed();

    void
    enableAdvancedOkButton();

    virtual void
    okButtonPressed() const;

    void
    setFileFormat(const QString& fileFormat)
    {
        m_fileFormat = fileFormat;
    }

    // Only for topic combo box widgets
    virtual void
    fillTopicComboBox()
    {
    }

protected:
    void
    fillTargetLineEdit();

protected:
    QPointer<QLineEdit> m_targetLineEdit;

    QPointer<QFormLayout> m_basicOptionsFormLayout;

    Parameters::AdvancedParameters& m_parameters;

    AdvancedSettings m_settings;

    const OUTPUT_TYPE m_outputType;

private:
    QString m_fileFormat;
};
