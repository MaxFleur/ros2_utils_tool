#include "catch_ros2/catch_ros2.hpp"

#include "BagTreeWidget.hpp"

#include <QLabel>

TEST_CASE("Bag Tree Widget Testing", "[bag_tree_widget]") {
    auto* const bagTreeWidget = new BagTreeWidget;

    for (auto i = 0; i < 3; ++i) {
        bagTreeWidget->blockSignals(true);
        bagTreeWidget->createItemWithTopicNameAndType("name_" + QString::number(i), "topic_" + QString::number(i), true);
        bagTreeWidget->blockSignals(false);
    }

    REQUIRE(bagTreeWidget->topLevelItemCount() == 3);
    for (auto i = 0; i < bagTreeWidget->topLevelItemCount(); ++i) {
        REQUIRE(bagTreeWidget->topLevelItem(i)->checkState(0) == Qt::Checked);

        auto* const nameLabel = static_cast<QLabel*>(bagTreeWidget->itemWidget(bagTreeWidget->topLevelItem(i), 1));
        auto* const topicLabel = static_cast<QLabel*>(bagTreeWidget->itemWidget(bagTreeWidget->topLevelItem(i), 2));
        REQUIRE(nameLabel->text() == "name_" + QString::number(i));
        REQUIRE(topicLabel->text() == "topic_" + QString::number(i));
    }

    auto* const lastItem = bagTreeWidget->topLevelItem(2);
    lastItem->setCheckState(0, Qt::Unchecked);

    auto* const lastNameLabel = static_cast<QLabel*>(bagTreeWidget->itemWidget(lastItem, 1));
    auto* const lastTopicLabel = static_cast<QLabel*>(bagTreeWidget->itemWidget(lastItem, 2));
    REQUIRE(!lastNameLabel->isEnabled());
    REQUIRE(!lastTopicLabel->isEnabled());
}
