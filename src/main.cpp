#include "ui/MainWindow.hpp"

#include "rclcpp/rclcpp.hpp"

#include <signal.h>

#include <QApplication>

int
main(int argc, char* argv[])
{
    // Initialize ROS and Qt
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    MainWindow mainWindow;
    mainWindow.show();

    while (rclcpp::ok()) {
        app.processEvents();
    }

    // Allow keyboard interrupts
    signal(SIGINT, SIG_DFL);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
