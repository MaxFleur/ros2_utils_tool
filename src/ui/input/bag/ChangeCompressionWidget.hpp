#pragma once

#include "AdvancedInputWidget.hpp"
#include "DeleteSourceSettings.hpp"
#include "Parameters.hpp"

// The widget used to manage compressing a bag file
class ChangeCompressionWidget : public AdvancedInputWidget
{
    Q_OBJECT

public:
    ChangeCompressionWidget(Parameters::DeleteSourceParameters& parameters,
                            bool                                compress,
                            QWidget*                            parent = 0);

private slots:
    void
    findSourceButtonPressed() override;

private:
    bool
    areInputsValid() const override;

    [[nodiscard]] bool
    isBagFileValid(const QString& bagDirectory) const;

private:
    Parameters::DeleteSourceParameters& m_parameters;

    DeleteSourceSettings m_settings;

    bool m_compress;
};
