#include "BasicBagWidget.hpp"

#include "BagTreeWidget.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QFileDialog>
#include <QLabel>
#include <QPushButton>
#include <QShortcut>
#include <QTreeWidgetItem>

#include <filesystem>

BasicBagWidget::BasicBagWidget(Parameters::SelectableBagContentParameters& parameters,
                               const QString& titleText, const QString& iconText, const QString& settingsText, const QString& unselectLabelText, QWidget *parent)
    : BasicInputWidget(titleText, iconText, parent), m_parameters(parameters),
    m_settings(parameters, settingsText), m_isPlayBag(settingsText == "play_bag")
{
    m_sourceLineEdit->setText(m_parameters.sourceDirectory);

    m_unselectLabel = new QLabel(unselectLabelText);
    m_unselectLabel->setVisible(false);
    auto labelFont = m_unselectLabel->font();
    labelFont.setBold(true);
    m_unselectLabel->setFont(labelFont);

    m_treeWidget = new BagTreeWidget;
    m_treeWidget->setMinimumWidth(380);

    m_okButton->setEnabled(true);

    auto* const okShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);

    connect(m_findSourceButton, &QPushButton::clicked, this, &BasicBagWidget::findSourceButtonPressed);
    connect(m_treeWidget, &QTreeWidget::itemChanged, this, &BasicBagWidget::itemCheckStateChanged);
    connect(m_okButton, &QPushButton::clicked, this, &BasicBagWidget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &BasicBagWidget::okButtonPressed);
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

    auto* labelName = qobject_cast<QLabel*>(m_treeWidget->itemWidget(item, COL_TOPIC_NAME));
    auto* labelType = qobject_cast<QLabel*>(m_treeWidget->itemWidget(item, COL_TOPIC_TYPE));
    // Need to know which vector we have to adjust
    auto& vector = labelType->text().contains("/msg/") ? m_parameters.topics : m_parameters.services;
    // Find the right index
    auto it = std::find_if(vector.begin(), vector.end(), [labelName] (const auto& element) {
        return QString::compare(element.name, labelName->text()) == 0;
    });
    const auto vectorIndex = std::distance(std::begin(vector), it);

    writeParameterToSettings(vector[vectorIndex].isSelected, item->checkState(COL_CHECKBOXES) == Qt::Checked, m_settings);
    enableOkButton();
}


void
BasicBagWidget::okButtonPressed() const
{
    if (!m_okButton->isEnabled()) {
        return;
    }

    emit okPressed();
}
