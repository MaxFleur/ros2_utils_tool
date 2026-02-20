#include "BasicBagWidget.hpp"

#include "BagTreeWidget.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QShortcut>
#include <QToolButton>
#include <QVBoxLayout>

#include <filesystem>

BasicBagWidget::BasicBagWidget(Parameters::SelectableBagTopicParameters& parameters,
                               const QString& titleText, const QString& iconText, const QString& settingsText, const QString& unselectLabelText, QWidget *parent)
    : BasicInputWidget(titleText, iconText, parent), m_parameters(parameters),
    m_settings(parameters, settingsText), m_isPlayBag(settingsText == "play_bag")
{
    if (!std::filesystem::exists(m_parameters.sourceDirectory.toStdString()) ||
        !Utils::ROS::doesDirectoryContainBagFile(m_parameters.sourceDirectory)) {
        m_parameters.sourceDirectory = QString();
        writeParameterToSettings(m_parameters.sourceDirectory, QString(), m_settings);
    }
    m_sourceLineEdit->setText(m_parameters.sourceDirectory);

    m_unselectLabel = new QLabel(unselectLabelText);
    m_unselectLabel->setVisible(false);
    auto labelFont = m_unselectLabel->font();
    labelFont.setBold(true);
    m_unselectLabel->setFont(labelFont);

    m_treeWidget = new BagTreeWidget;
    m_treeWidget->setMinimumWidth(380);

    m_okButton->setEnabled(true);

    m_controlsLayout = new QVBoxLayout;
    m_controlsLayout->addStretch();
    m_controlsLayout->addSpacing(20);
    m_controlsLayout->addWidget(m_headerPixmapLabel);
    m_controlsLayout->addWidget(m_headerLabel);
    m_controlsLayout->addSpacing(30);

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(m_controlsLayout);
    controlsSqueezedLayout->addStretch();

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    auto* const okShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);

    connect(m_findSourceButton, &QPushButton::clicked, this, &BasicBagWidget::findSourceButtonPressed);
    connect(m_treeWidget, &QTreeWidget::itemChanged, this, &BasicBagWidget::itemCheckStateChanged);
    connect(m_okButton, &QPushButton::clicked, this, [this] {
        emit okPressed();
    });
    connect(okShortCut, &QShortcut::activated, this, [this] {
        emit okPressed();
    });
}


void
BasicBagWidget::findSourceButtonPressed()
{
    QString fileName;
    if (m_isPlayBag) {
        const auto isValid = Utils::UI::isBagDirectoryValid(this);
        if (isValid == std::nullopt) {
            return;
        }

        fileName = *isValid;
    } else {
        fileName = QFileDialog::getSaveFileName(this, "Save Bag File");
        if (fileName.isEmpty()) {
            return;
        }
    }

    writeParameterToSettings(m_parameters.sourceDirectory, fileName, m_settings);
    m_settings.write();
    m_sourceLineEdit->setText(fileName);

    handleTreeAfterSource();
}


void
BasicBagWidget::itemCheckStateChanged(QTreeWidgetItem* item, int column)
{
    if (column != COL_CHECKBOXES) {
        return;
    }

    const auto rowIndex = m_treeWidget->indexOfTopLevelItem(item);
    writeParameterToSettings(m_parameters.topics[rowIndex].isSelected, item->checkState(COL_CHECKBOXES) == Qt::Checked, m_settings);
    enableOkButton();
}
