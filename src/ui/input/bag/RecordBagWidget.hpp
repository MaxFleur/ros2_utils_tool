#pragma once

#include "BasicBagWidget.hpp"
#include "Parameters.hpp"
#include "RecordBagSettings.hpp"

#include <QPointer>
#include <QWidget>

class BagTreeWidget;

class QPushButton;

// Widget used to manage recording a bag file
class RecordBagWidget : public BasicBagWidget
{
    Q_OBJECT

public:
    RecordBagWidget(Parameters::RecordBagParameters& parameters,
                    QWidget*                         parent = 0);

private slots:
    void
    handleTreeAfterSource() override;

    void
    populateTreeWidget() override;

    void
    enableOkButton() override;

private:
    QPointer<QPushButton> m_refreshButton;

    Parameters::RecordBagParameters& m_parameters;

    RecordBagSettings m_settings;

    static constexpr int HEIGHT_OFFSET = 80;
};
