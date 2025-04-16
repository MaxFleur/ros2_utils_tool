#pragma once

#include "BasicInputWidget.hpp"
#include "CompressBagSettings.hpp"
#include "Parameters.hpp"

class QComboBox;
class QLineEdit;

// The widget used to manage compressing a bag file
class ChangeCompressionWidget : public BasicInputWidget
{
    Q_OBJECT

public:
    ChangeCompressionWidget(Parameters::CompressBagParameters& parameters,
                            bool                               compress,
                            QWidget*                           parent = 0);

private slots:
    void
    sourceButtonPressed();

    void
    targetButtonPressed();

    void
    okButtonPressed() const;

private:
    [[nodiscard]] bool
    isBagFileValid(const QString& bagDirectory) const;

private:
    QPointer<QLineEdit> m_targetLineEdit;

    Parameters::CompressBagParameters& m_parameters;

    CompressBagSettings m_settings;

    bool m_compress;
};
