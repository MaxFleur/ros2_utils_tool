#include "UtilsCLI.hpp"

#include "UtilsROS.hpp"

#include <iostream>

namespace Utils::CLI
{
bool
containsInvalidParameters(const QStringList& argumentsList, const QStringList& checkList)
{
    for (const auto& argument : argumentsList) {
        if ((argument.startsWith("-") || argument.startsWith("--")) && !checkList.contains(argument)) {
            std::cerr << "Unrecognized argument '" << argument.toStdString() << "'!" << std::endl;
            return true;
        }
    }
    return false;
}


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
isTopicParameterAtValidPosition(const QStringList& argumentsList)
{
    const auto topicNameIndex = Utils::CLI::getArgumentsIndex(argumentsList, "-t", "--topic_name");
    if (argumentsList.at(topicNameIndex) == argumentsList.last()) {
        std::cerr << "Please enter a valid topic name!" << std::endl;
        return false;
    }
    return true;
}


bool
isTopicNameValid(const QStringList& argumentsList, const QString& bagDirectory, const QString& topicType, QString& topicNameToSet)
{
    if (Utils::CLI::containsArguments(argumentsList, "-t", "--topic_name")) {
        if (!isTopicParameterAtValidPosition(argumentsList)) {
            return false;
        }

        const auto& topicName = argumentsList.at(Utils::CLI::getArgumentsIndex(argumentsList, "-t", "--topic_name") + 1);
        if (!Utils::ROS::doesBagContainTopicName(bagDirectory, topicName)) {
            std::cerr << "Topic '" + topicName.toStdString() + "' has not been found in the bag file!" << std::endl;
            return false;
        }
        if (Utils::ROS::getTopicType(bagDirectory, topicName) != topicType) {
            std::cerr << "Topic '" + topicName.toStdString() + "' doesn't have the correct type!" << std::endl;
            return false;
        }
        topicNameToSet = topicName;
    }
    return true;
}


bool
continueWithInvalidROS2Name(const QStringList& argumentsList, QString& parameterTopicName)
{
    if (Utils::CLI::containsArguments(argumentsList, "-t", "--topic_name")) {
        if (!isTopicParameterAtValidPosition(argumentsList)) {
            return false;
        }

        const auto& topicName = argumentsList.at(Utils::CLI::getArgumentsIndex(argumentsList, "-t", "--topic_name") + 1);
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
showIsProcessingString(bool& isProcessing, bool isCompressing)
{
    auto processingChar = '\\';

    isProcessing = true;
    while (isProcessing) {
        switch (processingChar) {
        case '|':
            processingChar = '/';
            break;
        case '/':
            processingChar = '-';
            break;
        case '-':
            processingChar = '\\';
            break;
        case '\\':
            processingChar = '|';
            break;
        default:
            break;
        }

        const auto isProcessingString = isCompressing ? "Writing and compressing target file, please wait... "
                                                      : "Merging bags, please wait... ";
        std::cout << isProcessingString << processingChar << "\r" << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
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
