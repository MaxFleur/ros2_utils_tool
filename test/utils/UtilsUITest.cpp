#include "catch_ros2/catch_ros2.hpp"

#include "UtilsUI.hpp"

#include <QComboBox>
#include <QWidget>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp/writer.hpp"

#include "sensor_msgs/msg/image.hpp"

#include <filesystem>

TEST_CASE("Utils UI Testing", "[utils]") {
    SECTION("Font size test") {
        auto* const widget = new QWidget;

        Utils::UI::setWidgetFontSize(widget);
        auto font = widget->font();
        REQUIRE(font.pointSize() == 16);

        Utils::UI::setWidgetFontSize(widget, true);
        font = widget->font();
        REQUIRE(font.pointSize() == 14);
    }
    SECTION("Number of topics test") {
        const auto bagDirectory = std::filesystem::path("test_bag_file");
        std::filesystem::remove_all(bagDirectory);

        rosbag2_cpp::Writer writer;
        writer.open(bagDirectory);

        for (auto i = 0; i < 5; i++) {
            sensor_msgs::msg::Image imageMessage;
            imageMessage.width = 1;
            imageMessage.height = 1;
            writer.write(imageMessage, "/topic_image_one", rclcpp::Clock().now());
            writer.write(imageMessage, "/topic_image_two", rclcpp::Clock().now());
        }
        writer.close();

        auto* const comboBox = new QComboBox;
        Utils::UI::fillComboBoxWithTopics(comboBox, QString::fromStdString(bagDirectory), "sensor_msgs/msg/Image");

        REQUIRE(comboBox->count() == 2);
        std::filesystem::remove_all(bagDirectory);
    }
    SECTION("Checkbox test") {
        auto* checkBox = Utils::UI::createCheckBox("This is a tooltip", true);
        REQUIRE(checkBox->toolTip() == "This is a tooltip");
        REQUIRE(checkBox->checkState() == Qt::Checked);

        checkBox = Utils::UI::createCheckBox("Another tooltip", false);
        REQUIRE(checkBox->toolTip() == "Another tooltip");
        REQUIRE(checkBox->checkState() == Qt::Unchecked);
    }
}
