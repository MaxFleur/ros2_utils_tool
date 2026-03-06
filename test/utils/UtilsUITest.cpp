#include "catch_ros2/catch_ros2.hpp"

#include "UtilsUI.hpp"

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
    SECTION("Checkbox test") {
        auto* checkBox = Utils::UI::createCheckBox("This is a tooltip", true);
        REQUIRE(checkBox->toolTip() == "This is a tooltip");
        REQUIRE(checkBox->checkState() == Qt::Checked);

        checkBox = Utils::UI::createCheckBox("Another tooltip", false);
        REQUIRE(checkBox->toolTip() == "Another tooltip");
        REQUIRE(checkBox->checkState() == Qt::Unchecked);
    }
    SECTION("New appendix test") {
        const auto textSingleAppendix = "text_with_a_format.mp4";
        const auto& newTextSingleAppendix = Utils::UI::replaceTextAppendix(textSingleAppendix, "mkv");

        REQUIRE(QString::compare(newTextSingleAppendix, "text_with_a_format.mkv") == 0);

        const auto textMultipleAppendices = "text_with_a_format.mp4.mkv";
        const auto& newTextMultipleAppendices = Utils::UI::replaceTextAppendix(textMultipleAppendices, "avi");
        REQUIRE(QString::compare(newTextMultipleAppendices, "text_with_a_format.mp4.avi") == 0);
    }
}
