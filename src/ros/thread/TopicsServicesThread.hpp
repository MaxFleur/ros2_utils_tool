#pragma once

#include "UtilsROS.hpp"

#include <QThread>

// Thread gathering current ROS topic and service information
// Used to enable synchronous UI interaction in the Configure Record Bag and Topic Services Info Widget
class TopicsServicesThread : public QThread
{
    Q_OBJECT
public:
    explicit
    TopicsServicesThread(QObject* parent = nullptr) : QThread(parent)
    {
    }

    void
    run() override
    {
        m_topicInformation = Utils::ROS::getTopicInformation();
        m_serviceNamesAndTypes = Utils::ROS::getServiceNamesAndTypes();
    }

    [[nodiscard]] const std::vector<std::pair<std::string, std::array<std::string, 3> > >&
    getTopicInformation() const
    {
        return m_topicInformation;
    }

    [[nodiscard]] const std::map<std::string, std::vector<std::string> >&
    getServiceNamesAndTypes() const
    {
        return m_serviceNamesAndTypes;
    }

private:
    std::vector<std::pair<std::string, std::array<std::string, 3> > > m_topicInformation;

    std::map<std::string, std::vector<std::string> > m_serviceNamesAndTypes;
};
