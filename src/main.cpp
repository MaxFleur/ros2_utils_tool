#include "ui/MainWindow.hpp"

#include "UtilsROS.hpp"

#include <QApplication>

int
main(int argc, char* argv[])
{
    // Initialize ROS and Qt
    rclcpp::init(argc, argv);
    // We don't want any ROS log stuff to appear in the CLI
    Utils::ROS::disableROSLogging();

    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/icons/tools/main.svg"));

    MainWindow mainWindow;
    mainWindow.show();

    while (rclcpp::ok()) {
        app.processEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
