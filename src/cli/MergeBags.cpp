#include "MergeBagsThread.hpp"

#include "Parameters.hpp"
#include "UtilsCLI.hpp"
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
    std::cout << "Usage: ros2 run ros2_utils_tool tool_merge_bags [-h] [bag_path_first] [bag_path_second] [-t1 Topic [Topic...]]\n";
    std::cout << "                                                [-t2 Topic [Topic...]] [output_merged_bag]\n";
    std::cout << "                                                [--thread-count THREAD_COUNT] [-d]\n";
    std::cout << "                                                [--compression-mode {file,message}] [-s]\n\n";
    std::cout << "Merge topics from two bag files into a new bag file.\n";
    std::cout << "Note that duplicate specified topics (equal topics contained in both bags) will be merged to one.\n\n";
    std::cout << "positional arguments:\n";
    std::cout << "  bag_path_first        First bag file to merge.\n";
    std::cout << "  bag_path_second       Second bag file to merge.\n";
    std::cout << "  [-t1 Topic [Topic...]]\n";
    std::cout << "                        List of topics from the first bag file to be merged.\n";
    std::cout << "  [-t2 Topic [Topic...]]\n";
    std::cout << "                        List of topics from the second bag file to be merged.\n";
    std::cout << "  output_merged_bag     Output merged bag file.\n\n";
    std::cout << "options:\n";
    std::cout << "  -h, --help            Show this help message and exit.\n";
    std::cout << "  --thread-count THREAD_COUNT\n";
    std::cout << "                        Number of threads used to merge. Minimum is 1, maximum is " << std::thread::hardware_concurrency() << ", defaults to 1.\n";
    std::cout << "  -d, --delete          Delete the source files after completion.\n";
    std::cout << "  --compression-mode {file,message}\n";
    std::cout << "                        Choose the compression mode for the generated output file, defaults to None.\n";
    std::cout << "  -s, --suppress        Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run ros2_utils_tool tool_merge_bags /home/usr/first_bag /home/usr/second_bag -t1 /lidar -t2 /video /sensor /home/usr/target_bag --thread-count 4" << std::endl;
}


int
main(int argc, char* argv[])
{
    // Don't want ROS logging to break our progress info
    Utils::ROS::disableROSLogging();
    // Create application
    QCoreApplication app(argc, argv);

    const auto& arguments = app.arguments();
    if (arguments.size() < 8 || Utils::CLI::containsArguments(arguments, "-h", "--help")) {
        showHelp();
        return 0;
    }

    const QVector<QString> checkList{ "-t1", "-t2", "-d", "-s", "--delete", "--suppress", "--thread-count", "--compression-mode" };
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

        parameters.topics.push_back({ { { arguments.at(bagIndex) }, true }, bagDirectory });
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
    if (!Utils::CLI::checkArgumentValidity(arguments, "", "--thread-count", numberOfThreads, 1, std::thread::hardware_concurrency())) {
        throw std::runtime_error("Please enter a thread count value in the range of 1 to " + std::to_string(std::thread::hardware_concurrency()) + "!");
    }

    // Compression mode
    if (arguments.contains("--compression-mode")) {
        const auto modeIndex = arguments.indexOf("--compression-mode");
        if (arguments.at(modeIndex) == arguments.last() || (arguments.at(modeIndex + 1) != "file" && arguments.at(modeIndex + 1) != "message")) {
            throw std::runtime_error("Please enter either 'file' or 'message' for the compression mode!");
        }

        parameters.compressTarget = true;
        parameters.compressPerMessage = arguments.at(modeIndex + 1) == "message";
    }

    // Boundary used to check for the second bags topics (parameters usually come after those)
    auto boundary = arguments.size() - 1;
    if (parameters.deleteSource) {
        boundary--;
    }
    if (Utils::CLI::containsArguments(arguments, "-s", "--suppress")) {
        boundary--;
    }
    if (arguments.contains("--thread-count")) {
        boundary -= 2;
    }
    if (arguments.contains("--compression-mode")) {
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
    // Start operation
    Utils::CLI::runThread(mergeBagsThread, signalStatus);

    return EXIT_SUCCESS;
}
