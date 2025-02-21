#pragma once

#include "BasicInputWidget.hpp"
#include "AdvancedInputSettings.hpp"
#include "UtilsUI.hpp"

class QComboBox;
class QLineEdit;

// Derived from basic input, provides functions to search for an input bag or a target directory
class AdvancedInputWidget : public BasicInputWidget
{
    Q_OBJECT

public:
    AdvancedInputWidget(Utils::UI::AdvancedInputParameters& parameters,
                        const QString&                      headerText,
                        const QString&                      iconPath,
                        const QString&                      settingsIdentifier,
                        int                                 outputFormat,
                        QWidget*                            parent = 0);

protected slots:
    virtual void
    searchButtonPressed();

    void
    targetLocationButtonPressed();

    virtual void
    okButtonPressed();

    void
    setVideoFormat(const QString& videoFormat)
    {
        m_videoFormat = videoFormat;
    }

protected:
    QPointer<QComboBox> m_topicNameComboBox;
    QPointer<QLineEdit> m_targetLineEdit;

    static constexpr int OUTPUT_VIDEO = 0;
    static constexpr int OUTPUT_IMAGES = 1;
    static constexpr int OUTPUT_PCDS = 2;
    static constexpr int OUTPUT_BAG = 3;

private:
    Utils::UI::AdvancedInputParameters& m_parameters;

    AdvancedInputSettings m_settings;

    QString m_videoFormat;

    const int m_outputFormat;
};
