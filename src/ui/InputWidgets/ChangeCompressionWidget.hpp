#pragma once

#include "AdvancedInputWidget.hpp"
#include "CompressBagSettings.hpp"
#include "Parameters.hpp"

// The widget used to manage compressing a bag file
class ChangeCompressionWidget : public AdvancedInputWidget
{
    Q_OBJECT

public:
    ChangeCompressionWidget(Parameters::CompressBagParameters& parameters,
                            bool                               compress,
                            QWidget*                           parent = 0);

private slots:
    void
    findSourceButtonPressed() override;

    void
    okButtonPressed() const override;

private:
    [[nodiscard]] bool
    isBagFileValid(const QString& bagDirectory) const;

private:
    Parameters::CompressBagParameters& m_parameters;

    CompressBagSettings m_settings;

    bool m_compress;
};
