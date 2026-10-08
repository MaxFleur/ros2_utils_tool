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
    app.setOrganizationName("ros2_utils_tool");
    app.setApplicationName("ros2_utils_tool");

    MainWindow mainWindow;
    mainWindow.show();

    rclcpp::on_shutdown([] {
        if (!qApp) {
            return;
        }

        qApp->quit();
    });

    const auto returnValue = app.exec();
    rclcpp::shutdown();

    return returnValue;
}
