#include "DummyTopicWidget.hpp"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLineEdit>

DummyTopicWidget::DummyTopicWidget(const QString& topicTypeText, const QString& topicNameText, QWidget *parent) :
    QWidget(parent)
{
    m_topicTypeComboBox = new QComboBox;
    m_topicTypeComboBox->addItem("String", 0);
    m_topicTypeComboBox->addItem("Integer", 1);
    m_topicTypeComboBox->addItem("Image", 2);
    m_topicTypeComboBox->addItem("Point Cloud", 3);
    if (!topicTypeText.isEmpty()) {
        m_topicTypeComboBox->setCurrentText(topicTypeText);
    }

    m_topicNameLineEdit = new QLineEdit(topicNameText);
    m_topicNameLineEdit->setPlaceholderText("Enter Topic Name...");

    auto* const mainLayout = new QHBoxLayout;
    mainLayout->addWidget(m_topicTypeComboBox);
    mainLayout->addWidget(m_topicNameLineEdit);
    setLayout(mainLayout);
    // This widget will be integrated into other widgets where extra space would look bad
    // So remove all space around the widget itself
    mainLayout->setSpacing(0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    connect(m_topicTypeComboBox, &QComboBox::currentTextChanged, this, [this] (const QString& text) {
        emit topicTypeChanged(text);
    });
    connect(m_topicNameLineEdit, &QLineEdit::textChanged, this, [this] (const QString& text) {
        emit topicNameChanged(text);
    });
}
