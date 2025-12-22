#include "SendTF2Widget.hpp"

#include "UtilsROS.hpp"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGraphicsEffect>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QPropertyAnimation>
#include <QShortcut>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>

SendTF2Widget::SendTF2Widget(Parameters::SendTF2Parameters& parameters, QWidget *parent) :
    BasicInputWidget("Send TF2 Message", ":/icons/send_tf2", parent),
    m_parameters(parameters), m_settings(parameters, "send_tf2")
{
    m_nodeWrapper = std::make_shared<NodeWrapper>("tf_node");

    m_okButton->setText("Send");
    m_okButton->setEnabled(true);

    const auto createDoubleSpinBox = [this] (double& inputValue) {
        auto* const spinBox = new QDoubleSpinBox;
        spinBox->setDecimals(NUMBER_OF_DECIMALS);
        spinBox->setRange(SPINBOX_LOWER_RANGE, SPINBOX_UPPER_RANGE);
        spinBox->setValue(inputValue);

        connect(spinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this, &inputValue] (double newValue) {
            writeParameterToSettings(inputValue, newValue, m_settings);
        });

        return spinBox;
    };

    auto* const translationXSpinBox = createDoubleSpinBox(m_parameters.translation[0]);
    auto* const translationYSpinBox = createDoubleSpinBox(m_parameters.translation[1]);
    auto* const translationZSpinBox = createDoubleSpinBox(m_parameters.translation[2]);

    auto* const rotationXSpinBox = createDoubleSpinBox(m_parameters.rotation[0]);
    auto* const rotationYSpinBox = createDoubleSpinBox(m_parameters.rotation[1]);
    auto* const rotationZSpinBox = createDoubleSpinBox(m_parameters.rotation[2]);
    auto* const rotationWSpinBox = createDoubleSpinBox(m_parameters.rotation[3]);

    auto* const nameLineEdit = new QLineEdit(m_parameters.childFrameName);

    m_isStaticCheckBox = new QCheckBox;
    m_isStaticCheckBox->setChecked(m_parameters.isStatic);

    m_formLayout = new QFormLayout;
    m_formLayout->addRow("Translation X:", translationXSpinBox);
    m_formLayout->addRow("Translation Y:", translationYSpinBox);
    m_formLayout->addRow("Translation Z:", translationZSpinBox);
    m_formLayout->addRow("", new QLabel(""));
    m_formLayout->addRow("Rotation X:", rotationXSpinBox);
    m_formLayout->addRow("Rotation Y:", rotationYSpinBox);
    m_formLayout->addRow("Rotation Z:", rotationZSpinBox);
    m_formLayout->addRow("Rotation W:", rotationWSpinBox);
    m_formLayout->addRow("", new QLabel(""));
    m_formLayout->addRow("Child Frame Name", nameLineEdit);
    m_formLayout->addRow("Is static:", m_isStaticCheckBox);

    m_transformSentLabel = new QLabel("Transformation sent!");
    auto boldFont = m_transformSentLabel->font();
    boldFont.setBold(true);
    m_transformSentLabel->setFont(boldFont);
    m_transformSentLabel->setVisible(false);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(m_formLayout);
    controlsLayout->addSpacing(5);
    controlsLayout->addWidget(m_transformSentLabel);
    controlsLayout->addStretch();
    controlsLayout->setAlignment(m_transformSentLabel, Qt::AlignCenter);

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(controlsLayout);
    controlsSqueezedLayout->addStretch();

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    m_timer = new QTimer(this);
    m_timer->setSingleShot(true);

    auto* const okShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);

    connect(m_isStaticCheckBox, &QCheckBox::stateChanged, this, &SendTF2Widget::staticCheckBoxPressed);
    connect(nameLineEdit, &QLineEdit::textChanged, this, [this, nameLineEdit] {
        writeParameterToSettings(m_parameters.childFrameName, nameLineEdit->text(), m_settings);
    });
    connect(m_dialogButtonBox, &QDialogButtonBox::accepted, this, &SendTF2Widget::okButtonPressed);
    connect(okShortCut, &QShortcut::activated, this, &SendTF2Widget::okButtonPressed);

    connect(m_timer, &QTimer::timeout, this, [this] {
        static constexpr int LABEL_FADEOUT = 1000;

        // Create a small fading text if a transformation has been sent
        auto *const effect = new QGraphicsOpacityEffect;
        m_transformSentLabel->setGraphicsEffect(effect);

        auto *const animation = new QPropertyAnimation(effect, "opacity");
        animation->setDuration(LABEL_FADEOUT);
        animation->setStartValue(1.0);
        animation->setEndValue(0.0);
        animation->setEasingCurve(QEasingCurve::OutQuad);
        animation->start(QAbstractAnimation::DeleteWhenStopped);
    });

    staticCheckBoxPressed(m_parameters.isStatic);
}


void
SendTF2Widget::staticCheckBoxPressed(int state)
{
    // Partially checked value can still count for this case
    writeParameterToSettings(m_parameters.isStatic, state != Qt::Unchecked, m_settings);

    if (state == Qt::Unchecked) {
        m_rateSpinBox = new QSpinBox;
        m_rateSpinBox->setRange(1, 100);
        m_rateSpinBox->setValue(m_parameters.rate);

        m_formLayout->addRow("Rate:", m_rateSpinBox);

        connect(m_rateSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
            writeParameterToSettings(m_parameters.rate, value, m_settings);
        });
    } else if (m_rateSpinBox) {
        m_formLayout->removeRow(m_rateSpinBox);
    }
}


void
SendTF2Widget::okButtonPressed() const
{
    if (m_isStaticCheckBox->checkState() == Qt::Checked) {
        Utils::ROS::sendStaticTransformation(m_parameters.translation, m_parameters.rotation, m_nodeWrapper);
        m_transformSentLabel->setVisible(true);
        m_transformSentLabel->setGraphicsEffect(0);
        m_timer->start(LABEL_SHOWN_DURATION);

        return;
    }

    emit okPressed();
}
