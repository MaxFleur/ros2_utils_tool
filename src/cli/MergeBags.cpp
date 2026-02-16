#include "MergeBagsThread.hpp"

#include "Parameters.hpp"
#include "UtilsCLI.hpp"
#include "UtilsGeneral.hpp"
#include "UtilsROS.hpp"

#include <QCoreApplication>
#include <QObject>
#include <QSet>

#include <filesystem>
#include <iostream>

volatile sig_atomic_t signalStatus = 0;

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_merge_bags path/to/first/bag path/to/second/bag -t1 (...) -t2 (...) path/to/target/bag\n\n";
    std::cout << "Topic names after '-t1' are those contained in the first bag file, names after '-t2' in the second file.\n";
    std::cout << "Note that duplicate topics (equal topics contained in both bags) will be merged if both are specified.\n\n";
    std::cout << "Additional parameters:\n";
    std::cout << "-d or --delete: Delete the source bag files.\n\n";
    std::cout << "-th or --threads: Number of threads, must be at least 1 (maximum is " << std::thread::hardware_concurrency() << ").\n\n";
    std::cout << "-s or --suppress: Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run mediassist4_ros_tools tool_merge_bags /home/usr/first_bag /home/usr/second_bag -t1 /lidar -t2 /video /sensor /home/usr/target_bag -th 4\n\n";
    std::cout << "-h or --help: Show this help.\n";
}


int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);

    const auto& arguments = app.arguments();
    if (arguments.size() < 8 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    const QVector<QString> checkList{ "-t1", "-t2", "-d", "-th", "-s", "--delete", "--threads", "--suppress" };
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList); argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::MergeBagsParameters parameters;

    // Bag directories
    parameters.sourceDirectory = arguments.at(1);
    parameters.secondSourceDirectory = arguments.at(2);

    if (!std::filesystem::exists(parameters.sourceDirectory.toStdString()) ||
        !std::filesystem::exists(parameters.secondSourceDirectory.toStdString())) {
        throw std::runtime_error("One or more bag files do not exist. Please specify correct directories!");
    }
    if (std::filesystem::equivalent(parameters.sourceDirectory.toStdString(), parameters.secondSourceDirectory.toStdString())) {
        throw std::runtime_error("Please enter different files for the input bags!");
    }

    if (arguments.at(3) != "-t1") {
        throw std::runtime_error("Please specify '-t1' correctly!");
    }

    // Topics
    QSet<QString> topicNameSet;
    const auto addTopicsToParameters = [&arguments, &parameters, &topicNameSet] (const auto& bagDirectory, int& bagIndex) {
        if (!Utils::ROS::doesBagContainTopicName(bagDirectory, arguments.at(bagIndex))) {
            throw std::runtime_error("The specified topic '" + arguments.at(bagIndex).toStdString() + "' does not exist!");
        }

        parameters.topics.push_back({ arguments.at(bagIndex), bagDirectory, true });
        topicNameSet.insert(arguments.at(bagIndex));
        bagIndex++;
        return true;
    };

    // First bag
    auto topicsFirstBagIndex = 4;
    while (topicsFirstBagIndex <= arguments.size() && arguments.at(topicsFirstBagIndex) != "-t2") {
        if (!addTopicsToParameters(parameters.sourceDirectory, topicsFirstBagIndex)) {
            return 0;
        }
    }

    // Handle source deletion here because it might affect topic and target name handling
    parameters.deleteSource = Utils::CLI::containsArguments(arguments, "-d", "--delete");

    // Second bag
    if (!Utils::CLI::containsArguments(arguments, "-t2", "--topic2")) {
        throw std::runtime_error("Please specify '-t2' correctly!");
    }

    // Thread count
    auto numberOfThreads = 1;
    if (!Utils::CLI::checkArgumentValidity(arguments, "-th", "--threads", numberOfThreads, 1, std::thread::hardware_concurrency())) {
        throw std::runtime_error("Please enter a thread count value in the range of 1 to " + std::to_string(std::thread::hardware_concurrency()) + "!");
    }

    // Boundary used to check for the second bags topics (parameters usually come after those)
    auto boundary = arguments.size() - 1;
    if (parameters.deleteSource) {
        boundary--;
    }
    if (Utils::CLI::containsArguments(arguments, "-s", "--suppress")) {
        boundary--;
    }
    if (Utils::CLI::containsArguments(arguments, "-th", "--threads")) {
        boundary -= 2;
    }

    auto topicsSecondBagIndex = Utils::CLI::getArgumentsIndex(arguments, "-t2", "--topic2") + 1;
    while (topicsSecondBagIndex != boundary) {
        if (!addTopicsToParameters(parameters.secondSourceDirectory, topicsSecondBagIndex)) {
            return 0;
        }
    }

    // Target file
    parameters.targetDirectory = arguments.at(boundary);
    Utils::CLI::checkParentDirectory(parameters.targetDirectory);

    if (parameters.targetDirectory == parameters.sourceDirectory || parameters.targetDirectory == parameters.secondSourceDirectory) {
        throw std::runtime_error("The target file must have a different name than both input bag files!");
    }

    if (topicNameSet.size() != parameters.topics.size() && !Utils::CLI::containsArguments(arguments, "-s", "--suppress")) {
        if (!Utils::CLI::shouldContinue("Duplicate topic names detected. These would be merged into one topic. Do you want to continue? [y]/n")) {
            return 0;
        }
    }
    if (!Utils::CLI::continueExistingTargetLowDiskSpace(arguments, parameters.targetDirectory)) {
        return 0;
    }

    // Create thread and connect to its informations
    auto* const mergeBagsThread = new MergeBagsThread(parameters, numberOfThreads);
    auto isMerging = false;
    std::thread processingThread;

    QObject::connect(mergeBagsThread, &MergeBagsThread::processing, [&processingThread, &isMerging] {
        processingThread = std::thread(&Utils::CLI::showProcessingString, std::ref(isMerging));

        return EXIT_SUCCESS;
    });
    QObject::connect(mergeBagsThread, &MergeBagsThread::finished, [&isMerging, &processingThread] {
        isMerging = false;
        processingThread.join();

        std::cout << "\n"; // Extra line to stop flushing
        std::cout << "Merging bags finished!\n";
        return EXIT_SUCCESS;
    });
    QObject::connect(mergeBagsThread, &MergeBagsThread::finished, mergeBagsThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Source bag file 1: " << std::filesystem::absolute(parameters.sourceDirectory.toStdString()) << "\n";
    std::cout << "Source bag file 2: " << std::filesystem::absolute(parameters.secondSourceDirectory.toStdString()) << "\n";
    std::cout << "Target bag file: " << std::filesystem::absolute(parameters.targetDirectory.toStdString()) << "\n";
    std::cout << "Topics to merge:\n";
    for (const auto& topic : parameters.topics) {
        std::cout << "    " << topic.name.toStdString() << "\n";
    }
    std::cout << "Number of used threads: " << numberOfThreads << "\n\n";
    Utils::CLI::runThread(mergeBagsThread, signalStatus);

    return EXIT_SUCCESS;
}
