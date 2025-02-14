#include "UtilsCLI.hpp"

#include "UtilsROS.hpp"

#include <iostream>

namespace Utils::CLI
{
bool
containsArguments(const QStringList& argumentsList, const QString& shortArg, const QString& longArg)
{
    return argumentsList.contains(shortArg) || argumentsList.contains(longArg);
}


int
getArgumentsIndex(const QStringList& argumentsList, const QString& shortArg, const QString& longArg)
{
    return std::max(argumentsList.lastIndexOf(shortArg), argumentsList.lastIndexOf(longArg));
}


bool
checkArgumentValidity(const QStringList& argumentsList, const QString& shortArg, const QString& longArg,
                      int& parameter, int lowerRange, int higherRange, int argumentListOffset)
{
    if (!containsArguments(argumentsList, shortArg, longArg)) {
        return true;
    }

    const auto argumentIndex = std::max(argumentsList.lastIndexOf(shortArg), argumentsList.lastIndexOf(longArg));
    if (argumentsList.at(argumentIndex) == argumentsList.last() ||
        (argumentListOffset == 2 && argumentsList.at(argumentIndex + 1) == argumentsList.last())) {
        return false;
    }
    if (parameter = argumentsList.at(argumentIndex + argumentListOffset).toInt(); parameter < lowerRange || parameter > higherRange) {
        return false;
    }

    return true;
}


bool
shouldContinue(const std::string& message)
{
    std::string input;

    while (true) {
        std::cout << message << std::endl;
        std::cin >> input;

        if (input == "y") {
            return true;
        } else if (input == "n") {
            return false;
        }
    }
}


bool
isTopicNameValid(const QStringList& argumentsList, QString& parameterTopicName)
{
    if (containsArguments(argumentsList, "-t", "--topic_name")) {
        const auto topicNameIndex = Utils::CLI::getArgumentsIndex(argumentsList, "-t", "--topic_name");
        if (argumentsList.at(topicNameIndex) == argumentsList.last()) {
            std::cerr << "Please enter a valid topic name!" << std::endl;
            return false;
        }

        const auto& topicName = argumentsList.at(topicNameIndex + 1);
        if (!Utils::ROS::isNameROS2Conform(topicName)) {
            const auto errorString = "The topic name does not follow the ROS2 naming convention! More information on ROS2 naming convention is found here:\n"
                                     "https://design.ros2.org/articles/topic_and_service_names.html\n"
                                     "Do you want to continue anyways? [y/n]";
            if (!shouldContinue(errorString)) {
                return false;
            }
        }
        parameterTopicName = topicName;
    }

    return true;
}


std::string
drawProgressString(int progress)
{
    const int numberOfHashtags = ((float) progress / 100.0f) * 50;
    const auto numberOfDashes = 50 - numberOfHashtags;

    const auto progressString = std::string(numberOfHashtags, '#') + std::string(numberOfDashes, '-');
    return progressString;
}


void
runThread(QThread* thread, volatile sig_atomic_t& signalStatus)
{
    thread->start();

    // Look for SIGINT
    while (!thread->isFinished()) {
        if (signalStatus == SIGINT) {
            thread->requestInterruption();
            thread->wait();
            std::cout << "" << std::endl;
            std::cout << "Interrupted" << std::endl;
        }
    }
}
}
