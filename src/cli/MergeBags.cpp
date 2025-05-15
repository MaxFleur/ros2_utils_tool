#include "MergeBagsThread.hpp"

#include "UtilsCLI.hpp"
#include "Parameters.hpp"
#include "UtilsROS.hpp"

#include <QCoreApplication>
#include <QObject>
#include <QSet>

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_merge_bags path/to/first/bag path/to/second/bag -t1 (...) -t2 (...) path/to/target/bag\n" << std::endl;
    std::cout << "Topic names after '-t1' are those contained in the first bag file, names after '-t2' in the second file." << std::endl;
    std::cout << "Note that duplicate topics (equal topics contained in both bags) will be merged if both are specified.\n" << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-d or --delete: Delete the source bag files." << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


volatile sig_atomic_t signalStatus = 0;

int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);

    const auto arguments = app.arguments();
    if (arguments.size() < 8 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }
    const QStringList checkList{ "-h", "--help", "-t1", "-t2", "-d", "--delete" };
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList); argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::MergeBagsParameters parameters;
    parameters.topicName = "";

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
    auto topicsSecondBagIndex = Utils::CLI::getArgumentsIndex(arguments, "-t2", "--topic2") + 1;
    const auto boundary = parameters.deleteSource ? arguments.size() - 2 : arguments.size() - 1;
    while (topicsSecondBagIndex != boundary) {
        if (!addTopicsToParameters(parameters.secondSourceDirectory, topicsSecondBagIndex)) {
            return 0;
        }
    }

    // Target file
    parameters.targetDirectory = parameters.deleteSource ? arguments.at(arguments.size() - 2) : arguments.back();
    Utils::CLI::checkParentDirectory(parameters.targetDirectory);

    if (parameters.targetDirectory == parameters.sourceDirectory || parameters.targetDirectory == parameters.secondSourceDirectory) {
        throw std::runtime_error("The target file must have a different name than both input bag files!");
    }

    if (topicNameSet.size() != parameters.topics.size()) {
        if (!Utils::CLI::shouldContinue("Duplicate topic names detected. These would be merged into one topic. Do you want to continue? [y/n]")) {
            return 0;
        }
    }
    if (std::filesystem::exists(parameters.targetDirectory.toStdString())) {
        if (!Utils::CLI::shouldContinue("The target directory already exists. Continue and overwrite the target? [y/n]")) {
            return 0;
        }
    }

    // Create thread and connect to its informations
    auto* const mergeBagsThread = new MergeBagsThread(parameters, std::thread::hardware_concurrency());
    auto isMerging = false;
    std::thread processingThread;

    QObject::connect(mergeBagsThread, &MergeBagsThread::processing, [&processingThread, &isMerging] {
        processingThread = std::thread(&Utils::CLI::showProcessingString, std::ref(isMerging), Utils::CLI::CLI_MERGE);

        return EXIT_SUCCESS;
    });
    QObject::connect(mergeBagsThread, &MergeBagsThread::finished, [&isMerging, &processingThread] {
        isMerging = false;
        processingThread.join();

        std::cout << "" << std::endl; // Extra line to stop flushing
        std::cout << "Merging bags finished!" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(mergeBagsThread, &MergeBagsThread::finished, mergeBagsThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    Utils::CLI::runThread(mergeBagsThread, signalStatus);

    return EXIT_SUCCESS;
}
