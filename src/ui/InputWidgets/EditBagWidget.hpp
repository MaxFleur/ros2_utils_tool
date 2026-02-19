#pragma once

#include "BasicBagWidget.hpp"
#include "EditBagSettings.hpp"
#include "Parameters.hpp"

#include <QPointer>
#include <QWidget>

class QCheckBox;
class QLabel;

// Widget for editing a bag file
class EditBagWidget : public BasicBagWidget
{
    Q_OBJECT
public:
    explicit
    EditBagWidget(Parameters::EditBagParameters& parameters,
                  bool                           warnROS2NameConvention,
                  QWidget*                       parent = 0);

private slots:
    void
    findSourceButtonPressed() override;

    void
    createTopicTree();

    void
    itemCheckStateChanged(QTreeWidgetItem* item,
                          int              column) override;

    void
    okButtonPressed() const override;

private:
    QPointer<QLabel> m_editLabel;
    QPointer<QLabel> m_differentDirsLabel;

    QPointer<QCheckBox> m_updateTimestampsCheckBox;

    Parameters::EditBagParameters& m_parameters;

    EditBagSettings m_settings;

    const bool m_warnROS2NameConvention;

    static constexpr int COL_MESSAGE_COUNT = 3;
    static constexpr int COL_RENAMING = 4;

    static constexpr int HEIGHT_OFFSET = 80;
};
