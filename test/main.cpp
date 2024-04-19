#define CATCH_CONFIG_RUNNER
#include "catch_ros2/catch_ros2.hpp"

#include <QApplication>

int
main(int argc, char **argv)
{
    setenv("QT_QPA_PLATFORM", "offscreen", 0);

    QApplication app(argc, argv);

    return Catch::Session().run(argc, argv);
}
