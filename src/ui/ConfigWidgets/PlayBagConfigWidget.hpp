#pragma once

#include "BasicConfigWidget.hpp"
#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QLineEdit;

/**
 * @brief The widget used to play a ROSBag
 */
class PlayBagConfigWidget : public BasicConfigWidget
{
    Q_OBJECT

public:
    PlayBagConfigWidget(Utils::UI::PlayBagParameters& bagParameters,
                        QWidget*                      parent = 0);

private slots:
    void
    bagLocationButtonPressed();

    void
    okButtonPressed();

private:
    QPointer<QLineEdit> m_bagLineEdit;

    Utils::UI::PlayBagParameters& m_bagParameters;
};
