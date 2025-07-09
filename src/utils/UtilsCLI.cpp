#include "UtilsCLI.hpp"

#include "UtilsGeneral.hpp"
#include "UtilsROS.hpp"

#include <filesystem>

namespace Utils::CLI
{
std::optional<std::string>
containsInvalidParameters(const QStringList& argumentsList, const QStringList& checkList)
{
    for (const auto& argument : argumentsList) {
        if ((argument.startsWith("-") || argument.startsWith("--")) && !checkList.contains(argument)) {
            return argument.toStdString();
        }
    }
    return std::nullopt;
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


void
checkTopicParameterPosition(const QStringList& argumentsList)
{
    if (const auto topicNameIndex = Utils::CLI::getArgumentsIndex(argumentsList, "-t", "--topic_name");
        argumentsList.at(topicNameIndex) == argumentsList.last()) {
        throw std::runtime_error("Please enter a valid topic name!");
    }
}


void
checkTopicNameValidity(const QStringList& argumentsList, const QString& bagDirectory, const QString& topicType, QString& topicNameToSet)
{
    if (Utils::CLI::containsArguments(argumentsList, "-t", "--topic_name")) {
        checkTopicParameterPosition(argumentsList);

        const auto& topicName = argumentsList.at(Utils::CLI::getArgumentsIndex(argumentsList, "-t", "--topic_name") + 1);
        if (!Utils::ROS::doesBagContainTopicName(bagDirectory, topicName)) {
            throw std::runtime_error("Topic '" + topicName.toStdString() + "' has not been found in the bag file!");
        }
        if (Utils::ROS::getTopicType(bagDirectory, topicName) != topicType) {
            throw std::runtime_error("Topic '" + topicName.toStdString() + "' doesn't have the correct type!");
        }
        topicNameToSet = topicName;
    }
}


void
checkBagSourceDirectory(const QString& bagDirectory)
{
    if (!std::filesystem::exists(bagDirectory.toStdString())) {
        throw std::runtime_error("Bag file not found. Make sure that the bag file exists!");
    }
    if (const auto doesDirContainBag = Utils::ROS::doesDirectoryContainBagFile(bagDirectory); !doesDirContainBag) {
        throw std::runtime_error("The directory does not contain a bag file!");
    }
}


void
checkParentDirectory(const QString& directory, bool isTarget)
{
    auto parentDirectory = directory;
    parentDirectory.truncate(parentDirectory.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(parentDirectory.toStdString())) {
        throw std::runtime_error(isTarget ? "Invalid target directory. Please enter a valid one!"
                                          : "Invalid source directory. Please enter a valid one!");
    }
}


void
checkForTargetTopic(const QString& directory, QString& parameterTopicName, bool isTopicOfImageType)
{
    const auto targetTopicName = Utils::ROS::getFirstTopicWithCertainType(directory, isTopicOfImageType ? "sensor_msgs/msg/Image"
                                                                                                        : "sensor_msgs/msg/PointCloud2");
    if (targetTopicName == std::nullopt) {
        throw std::runtime_error(isTopicOfImageType ? "The bag file does not contain any image topics!"
                                                    : "The bag file does not contain any point cloud topics!");
    }

    parameterTopicName = *targetTopicName;
}


bool
shouldContinue(const std::string& message)
{
    std::string input;

    while (true) {
        std::cout << message << std::endl;
        std::getline(std::cin, input);

        // 'Y' or 'Enter' key will accept
        if (input == "y" || input.length() == 0) {
            return true;
        } else if (input == "n") {
            return false;
        }
    }
}


bool
continueWithInvalidROS2Name(const QStringList& argumentsList, QString& parameterTopicName)
{
    if (Utils::CLI::containsArguments(argumentsList, "-t", "--topic_name")) {
        checkTopicParameterPosition(argumentsList);

        const auto& topicName = argumentsList.at(Utils::CLI::getArgumentsIndex(argumentsList, "-t", "--topic_name") + 1);
        if (!Utils::ROS::isNameROS2Conform(topicName)) {
            const auto errorString = "The topic name does not follow the ROS2 naming convention! More information on ROS2 naming convention is found here:\n"
                                     "https://design.ros2.org/articles/topic_and_service_names.html\n"
                                     "Do you want to continue anyways? [y]/n";
            if (!shouldContinue(errorString)) {
                return false;
            }
        }
        parameterTopicName = topicName;
    }
    return true;
}


bool
continueExistingTargetLowDiskSpace(const QString& directory)
{
    if (const auto diskSpace = Utils::General::getAvailableDriveSpace(directory); diskSpace < Utils::General::MINIMUM_RECOMMENDED_DRIVE_SPACE) {
        if (!Utils::CLI::shouldContinue("Available disk space is very small (" + std::to_string(diskSpace) + " GB). Do you want to continue? [y]/n")) {
            return false;
        }
    }
    if (std::filesystem::exists(directory.toStdString())) {
        if (!Utils::CLI::shouldContinue("The target directory already exists. Continue and overwrite the target? [y]/n")) {
            return false;
        }
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
showProcessingString(bool& isProcessing, int toolOperation)
{
    auto processingChar = '\\';
    std::string isProcessingString;

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

        switch (toolOperation) {
        case 0:
            isProcessingString = "Writing and compressing target file, please wait... ";
            break;
        case 1:
            isProcessingString = "Decompressing and writing target file, please wait... ";
            break;
        case 2:
            isProcessingString = "Merging bags, please wait... ";
            break;
        }

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
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
}
