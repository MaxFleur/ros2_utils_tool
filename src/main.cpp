#include "ui/MainWindow.hpp"

#include "rclcpp/rclcpp.hpp"

#include <QApplication>

int
main(int argc, char* argv[])
{
    // Initialize ROS and Qt
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/icons/main.svg"));

    MainWindow mainWindow;
    mainWindow.show();

    while (rclcpp::ok()) {
        app.processEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
