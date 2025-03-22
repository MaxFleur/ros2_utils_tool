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
    std::cout << "Note that duplicate topics (equal topics contained in both bags) will be merged if both are specified." << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


volatile sig_atomic_t signalStatus = 0;

int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);

    const auto arguments = app.arguments();
    if (Utils::CLI::containsInvalidParameters(arguments, { "-h", "--help", "-t1", "-t2" }) ||
        arguments.size() < 8 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    Parameters::MergeBagsParameters parameters;
    parameters.topicName = "";

    // Bag directories
    parameters.sourceDirectory = arguments.at(1);
    parameters.secondSourceDirectory = arguments.at(2);

    if (!std::filesystem::exists(parameters.sourceDirectory.toStdString()) ||
        !std::filesystem::exists(parameters.secondSourceDirectory.toStdString())) {
        std::cerr << "One or more bag files do not exist. Please specify correct directories!" << std::endl;
        return 0;
    }
    if (std::filesystem::equivalent(parameters.sourceDirectory.toStdString(), parameters.secondSourceDirectory.toStdString())) {
        std::cerr << "Please enter different files for the input bags!" << std::endl;
        return 0;
    }

    if (arguments.at(3) != "-t1") {
        std::cerr << "Please specify '-t1' correctly!" << std::endl;
        return 0;
    }

    // Topics
    QSet<QString> topicNameSet;
    const auto addTopicsToParameters = [&arguments, &parameters, &topicNameSet] (const auto& bagDirectory, int& bagIndex) {
        if (!Utils::ROS::doesBagContainTopicName(bagDirectory, arguments.at(bagIndex))) {
            std::cerr << "The specified topic '" << arguments.at(bagIndex).toStdString() << "' does not exist!" << std::endl;
            return false;
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

    // Second bag
    if (!Utils::CLI::containsArguments(arguments, "-t2", "--topic2")) {
        std::cerr << "Please specify '-t2' correctly!" << std::endl;
        return 0;
    }
    auto topicsSecondBagIndex = Utils::CLI::getArgumentsIndex(arguments, "-t2", "--topic2") + 1;
    while (topicsSecondBagIndex != arguments.size() - 1) {
        if (!addTopicsToParameters(parameters.secondSourceDirectory, topicsSecondBagIndex)) {
            return 0;
        }
    }

    // Target file
    parameters.targetDirectory = arguments.back();
    auto dirPath = parameters.targetDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        std::cerr << "Invalid target directory. Please enter a valid one!" << std::endl;
        return 0;
    }
    if (parameters.targetDirectory == parameters.sourceDirectory || parameters.targetDirectory == parameters.secondSourceDirectory) {
        std::cerr << "The target file must have a different name then both input bag files!" << std::endl;
        return 0;
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
        processingThread = std::thread(&Utils::CLI::showIsProcessingString, std::ref(isMerging), false);

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

    std::cout << "Merging bags. Please wait..." << std::endl;
    Utils::CLI::runThread(mergeBagsThread, signalStatus);

    return EXIT_SUCCESS;
}
