#include "SendTF2Widget.hpp"

#include "UtilsGeneral.hpp"
#include "UtilsIO.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFileDialog>
#include <QFormLayout>
#include <QGraphicsEffect>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPropertyAnimation>
#include <QPushButton>
#include <QShortcut>
#include <QSpinBox>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

SendTF2Widget::SendTF2Widget(Parameters::SendTF2Parameters& parameters, QWidget *parent) :
    BasicInputWidget("Send TF2 Message", ":/icons/tools/send_tf2", parent),
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

    m_translationXSpinBox = createDoubleSpinBox(m_parameters.translation[0]);
    m_translationYSpinBox = createDoubleSpinBox(m_parameters.translation[1]);
    m_translationZSpinBox = createDoubleSpinBox(m_parameters.translation[2]);
    m_rotationXSpinBox = createDoubleSpinBox(m_parameters.rotation[0]);
    m_rotationYSpinBox = createDoubleSpinBox(m_parameters.rotation[1]);
    m_rotationZSpinBox = createDoubleSpinBox(m_parameters.rotation[2]);
    m_rotationWSpinBox = createDoubleSpinBox(m_parameters.rotation[3]);

    m_childFrameNameLineEdit = new QLineEdit(m_parameters.childFrameName);

    m_isStaticCheckBox = new QCheckBox;
    m_isStaticCheckBox->setChecked(m_parameters.isStatic);

    m_saveToFileButton = new QToolButton;
    m_saveToFileButton->setToolTip("Save the current config to json/yaml.");
    m_loadFromFileButton = new QToolButton;
    m_loadFromFileButton->setToolTip("Set the config via a stored json/yaml file.");

    auto* const loadFileButtonLayout = new QHBoxLayout;
    loadFileButtonLayout->addStretch();
    loadFileButtonLayout->addWidget(m_saveToFileButton);
    loadFileButtonLayout->addWidget(m_loadFromFileButton);

    auto* const spacerWidget = new QWidget;
    spacerWidget->setFixedHeight(5);
    auto* const secondSpacerWidget = new QWidget;
    secondSpacerWidget->setFixedHeight(5);

    m_formLayout = new QFormLayout;
    m_formLayout->addRow("Translation X:", m_translationXSpinBox);
    m_formLayout->addRow("Translation Y:", m_translationYSpinBox);
    m_formLayout->addRow("Translation Z:", m_translationZSpinBox);
    m_formLayout->addRow("", spacerWidget);
    m_formLayout->addRow("Rotation X:", m_rotationXSpinBox);
    m_formLayout->addRow("Rotation Y:", m_rotationYSpinBox);
    m_formLayout->addRow("Rotation Z:", m_rotationZSpinBox);
    m_formLayout->addRow("Rotation W:", m_rotationWSpinBox);
    m_formLayout->addRow("", secondSpacerWidget);
    m_formLayout->addRow("Child Frame Name:", m_childFrameNameLineEdit);
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
    controlsLayout->addSpacing(25);
    controlsLayout->addLayout(m_formLayout);
    controlsLayout->addLayout(loadFileButtonLayout);
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
    connect(m_saveToFileButton, &QToolButton::clicked, this, &SendTF2Widget::saveToFileButtonPressed);
    connect(m_loadFromFileButton, &QToolButton::clicked, this, &SendTF2Widget::loadFromFileButtonPressed);
    connect(m_childFrameNameLineEdit, &QLineEdit::textChanged, this, [this] {
        writeParameterToSettings(m_parameters.childFrameName, m_childFrameNameLineEdit->text(), m_settings);
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
    setPixmapLabelIcon();
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
SendTF2Widget::saveToFileButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Save File", "/home", "File (*.json *.yaml)");
    if (fileName.isEmpty()) {
        return;
    }

    if (const auto valid = Utils::General::getFileExtension(fileName) == "json" ? Utils::IO::writeTF2ToJson(fileName, m_parameters)
                                                                                : Utils::IO::writeTF2ToYAML(fileName, m_parameters); !valid) {
        Utils::UI::createCriticalMessageBox("Writing failed!",
                                            "Failed writing file! Please make sure that you have permissions to access the drive and write the file.");
        return;
    }

    animateInfoLabel("File successfully saved.");
}


void
SendTF2Widget::loadFromFileButtonPressed()
{
    const auto fileName = QFileDialog::getOpenFileName(this, "Open File", "/home", "File (*.json *.yaml)");
    if (fileName.isEmpty()) {
        return;
    }

    if (const auto valid = Utils::General::getFileExtension(fileName) == "json" ? Utils::IO::readTF2FromJson(fileName, m_parameters)
                                                                                : Utils::IO::readTF2FromYAML(fileName, m_parameters); !valid) {
        Utils::UI::createCriticalMessageBox("Loading failed!",
                                            "The input config file seems to be invalid or could not be read.\n"
                                            "Save a valid config file using this tool first and then try again using this saved config.");
        return;
    }

    m_translationXSpinBox->setValue(m_parameters.translation[0]);
    m_translationYSpinBox->setValue(m_parameters.translation[1]);
    m_translationZSpinBox->setValue(m_parameters.translation[2]);
    m_rotationXSpinBox->setValue(m_parameters.rotation[0]);
    m_rotationYSpinBox->setValue(m_parameters.rotation[1]);
    m_rotationZSpinBox->setValue(m_parameters.rotation[2]);
    m_rotationWSpinBox->setValue(m_parameters.rotation[3]);
    m_childFrameNameLineEdit->setText(m_parameters.childFrameName);

    animateInfoLabel("File successfully loaded.");
}


void
SendTF2Widget::okButtonPressed()
{
    if (m_isStaticCheckBox->checkState() == Qt::Checked) {
        Utils::ROS::sendStaticTransformation(m_parameters.translation, m_parameters.rotation, m_nodeWrapper);
        animateInfoLabel("Transformation sent!");
        return;
    }

    emit okPressed();
}


void
SendTF2Widget::animateInfoLabel(const QString& labelText)
{
    m_transformSentLabel->setVisible(true);
    m_transformSentLabel->setGraphicsEffect(0);
    m_transformSentLabel->setText(labelText);
    m_timer->start(LABEL_SHOWN_DURATION);
}


void
SendTF2Widget::setPixmapLabelIcon() const
{
    const auto isDarkMode = Utils::UI::isDarkMode();
    m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? m_iconPath + "_white.svg" : m_iconPath + "_black.svg").pixmap(QSize(100, 45)));

    m_loadFromFileButton->setIcon(QIcon(isDarkMode ? ":/icons/widgets/load_from_file_white.svg" : ":/icons/widgets/load_from_file_black.svg"));
    m_saveToFileButton->setIcon(QIcon(isDarkMode ? ":/icons/widgets/save_to_file_white.svg" : ":/icons/widgets/save_to_file_black.svg"));
}


bool
SendTF2Widget::event(QEvent *event)
{
    [[unlikely]] if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setPixmapLabelIcon();
    }
    return QWidget::event(event);
}
