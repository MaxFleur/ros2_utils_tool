#include "TopicWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QComboBox>
#include <QCompleter>
#include <QEvent>
#include <QHBoxLayout>
#include <QPushButton>
#include <QToolButton>

TopicWidget::TopicWidget(bool isDummyWidget, bool addRemoveButton, const QString& topicTypeText,
                         const QString& topicNameText, QWidget *parent) :
    QWidget(parent)
{
    m_topicNameLineEdit = new QLineEdit(topicNameText);
    m_topicNameLineEdit->setPlaceholderText("Enter Topic Name...");

    m_removeTopicButton = new QToolButton;
    m_removeTopicButton->setToolTip("Remove the topic on the left.");
    auto sizePolicy = m_removeTopicButton->sizePolicy();
    sizePolicy.setRetainSizeWhenHidden(true);
    m_removeTopicButton->setSizePolicy(sizePolicy);

    auto* const mainLayout = new QHBoxLayout;
    if (isDummyWidget) {
        auto* const topicTypeComboBox = new QComboBox;
        topicTypeComboBox->addItem("String", 0);
        topicTypeComboBox->addItem("Integer", 1);
        topicTypeComboBox->addItem("Image", 2);
        topicTypeComboBox->addItem("Point Cloud", 3);
        topicTypeComboBox->addItem("TF2", 4);
        if (!topicTypeText.isEmpty()) {
            topicTypeComboBox->setCurrentText(topicTypeText);
        }

        mainLayout->addWidget(topicTypeComboBox);

        connect(topicTypeComboBox, &QComboBox::currentTextChanged, this, [this] (const QString& text) {
            emit topicTypeChanged(text);
        });
    } else {
        const auto& currentTopicsAndTypes = Utils::ROS::getTopicInformation();
        QStringList wordList;
        for (const auto& currentTopic : currentTopicsAndTypes) {
            wordList.append(QString::fromStdString(currentTopic.first));
        }

        auto* const completer = new QCompleter(wordList);
        completer->setCaseSensitivity(Qt::CaseInsensitive);
        m_topicNameLineEdit->setCompleter(completer);
    }

    mainLayout->addWidget(m_topicNameLineEdit);
    mainLayout->addWidget(m_removeTopicButton);
    m_removeTopicButton->setVisible(addRemoveButton);

    setLayout(mainLayout);
    // This widget will be integrated into other widgets where extra space would look bad
    // So remove all space around the widget itself
    mainLayout->setSpacing(0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    connect(m_removeTopicButton, &QPushButton::clicked, this, [this] {
        emit topicRemoveButtonClicked();
    });
    connect(m_topicNameLineEdit, &QLineEdit::textChanged, this, [this] (const QString& text) {
        emit topicNameChanged(text);
    });

    setPixmapLabelIcon();
}


void
TopicWidget::setPixmapLabelIcon() const
{
    if (!m_removeTopicButton) {
        return;
    }

    const auto isDarkMode = Utils::UI::isDarkMode();
    m_removeTopicButton->setIcon(QIcon(isDarkMode ? ":/icons/widgets/minus_white.svg" : ":/icons/widgets/minus_black.svg"));
}


bool
TopicWidget::event(QEvent *event)
{
    [[unlikely]] if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setPixmapLabelIcon();
    }
    return QWidget::event(event);
}
