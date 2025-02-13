#include "MergeBagsThread.hpp"

#include "UtilsCLI.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCoreApplication>
#include <QObject>
#include <QSet>

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_merge_bags path/to/First/Bag path/to/SecondBag -t1 (...) -t2 (...) path/To/Target\n" << std::endl;
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
    if (arguments.size() < 8 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    Utils::UI::MergeBagsInputParameters inputParameters;
    inputParameters.topicName = "";

    // Bag directories
    inputParameters.sourceDirectory = arguments.at(1);
    inputParameters.secondSourceDirectory = arguments.at(2);

    if (!std::filesystem::exists(inputParameters.sourceDirectory.toStdString()) ||
        !std::filesystem::exists(inputParameters.secondSourceDirectory.toStdString())) {
        std::cerr << "One or more bag files do not exist. Please specify correct directories!" << std::endl;
        return 0;
    }
    if (std::filesystem::equivalent(inputParameters.sourceDirectory.toStdString(), inputParameters.secondSourceDirectory.toStdString())) {
        std::cerr << "Please enter different files for the input bags!" << std::endl;
        return 0;
    }

    if (arguments.at(3) != "-t1") {
        std::cerr << "Please specify '-t1' correctly!" << std::endl;
        return 0;
    }

    // Topics
    QSet<QString> topicNameSet;
    const auto addTopicsToParameters = [&arguments, &inputParameters, &topicNameSet] (const auto& bagDirectory, int& bagIndex) {
        if (!Utils::ROS::doesBagContainTopicName(bagDirectory, arguments.at(bagIndex))) {
            std::cerr << "The specified topic '" << arguments.at(bagIndex).toStdString() << "' does not exist!" << std::endl;
            return false;
        }

        inputParameters.topics.push_back({ arguments.at(bagIndex), bagDirectory, true });
        topicNameSet.insert(arguments.at(bagIndex));
        bagIndex++;
        return true;
    };

    // First bag
    auto topicsFirstBagIndex = 4;
    while (topicsFirstBagIndex <= arguments.size() && arguments.at(topicsFirstBagIndex) != "-t2") {
        if (!addTopicsToParameters(inputParameters.sourceDirectory, topicsFirstBagIndex)) {
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
        if (!addTopicsToParameters(inputParameters.secondSourceDirectory, topicsSecondBagIndex)) {
            return 0;
        }
    }

    // Target file
    inputParameters.targetDirectory = arguments.back();
    if (inputParameters.targetDirectory == inputParameters.sourceDirectory || inputParameters.targetDirectory == inputParameters.secondSourceDirectory) {
        std::cerr << "The target file must have a different name then both input bag files!" << std::endl;
        return 0;
    }

    if (topicNameSet.size() != inputParameters.topics.size()) {
        if (!Utils::CLI::shouldContinue("Duplicate topic names detected. These would be merged into one topic. Do you want to continue? [y/n]")) {
            return 0;
        }
    }
    if (std::filesystem::exists(inputParameters.targetDirectory.toStdString())) {
        if (!Utils::CLI::shouldContinue("The target directory already exists. Continue and overwrite the target? [y/n]")) {
            return 0;
        }
    }

    // Create thread and connect to its informations
    auto* const mergeBagsThread = new MergeBagsThread(inputParameters);

    QObject::connect(mergeBagsThread, &MergeBagsThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(mergeBagsThread, &MergeBagsThread::finished, [] {
        // This signal is thrown even if SIGINT is called, but we haven't finished, only interrupted
        if (signalStatus != SIGINT) {
            std::cout << "" << std::endl; // Extra line to stop flushing
            std::cout << "Merging bags finished!" << std::endl;
        }
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
