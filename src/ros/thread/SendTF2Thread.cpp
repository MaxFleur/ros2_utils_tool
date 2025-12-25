#include "SendTF2Thread.hpp"

SendTF2Thread::SendTF2Thread(const Parameters::SendTF2Parameters& parameters,
                             QObject*                             parent) :
    BasicThread(parameters.sourceDirectory, "", parent),
    m_parameters(parameters)
{
    m_node = std::make_shared<rclcpp::Node>("tf_node");
    m_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(m_node);
}


void
SendTF2Thread::run()
{
    const int rate = ((1000 / static_cast<float>(m_parameters.rate)) * 1000);
    geometry_msgs::msg::TransformStamped transformStamped;

    // Send the transformation using the specified rate
    auto timer = m_node->create_wall_timer(std::chrono::microseconds(rate), [this, &transformStamped] {
        if (isInterruptionRequested()) {
            return;
        }

        transformStamped.header.stamp = m_node->get_clock()->now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = m_parameters.childFrameName.toStdString();

        transformStamped.transform.translation.x = m_parameters.translation[0];
        transformStamped.transform.translation.y = m_parameters.translation[1];
        transformStamped.transform.translation.z = m_parameters.translation[2];

        transformStamped.transform.rotation.x = m_parameters.rotation[0];
        transformStamped.transform.rotation.y = m_parameters.rotation[1];
        transformStamped.transform.rotation.z = m_parameters.rotation[2];
        transformStamped.transform.rotation.w = m_parameters.rotation[3];

        m_broadcaster->sendTransform(transformStamped);
    });

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(m_node);

    while (!isInterruptionRequested()) {
        executor->spin_once();
    }

    timer->cancel();
    emit finished();
}
