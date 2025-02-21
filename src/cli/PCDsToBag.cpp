#include "PCDsToBagThread.hpp"

#include "UtilsCLI.hpp"
#include "UtilsUI.hpp"

#include <QCoreApplication>
#include <QObject>

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_pcds_to_bag path/to/pcd/dir path/of/stored/ros_bag\n" << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-t or --topic_name: Topic name in the bag file. If this is empty, the name '/topic_point_cloud' will be used.\n" << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


volatile sig_atomic_t signalStatus = 0;

int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);
    const auto arguments = app.arguments();
    if (arguments.size() < 3 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    Utils::UI::AdvancedInputParameters inputParameters;

    // PCDs directory
    inputParameters.sourceDirectory = arguments.at(1);
    auto dirPath = inputParameters.sourceDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        std::cerr << "The entered directory for the pcd files does not exist. Please specify a correct directory!" << std::endl;
        return 0;
    }

    auto containsPCDFiles = false;
    for (auto const& entry : std::filesystem::directory_iterator(inputParameters.sourceDirectory.toStdString())) {
        if (entry.path().extension() == ".pcd") {
            containsPCDFiles = true;
            break;
        }
    }
    if (!containsPCDFiles) {
        std::cerr << "The entered directory for the pcd files does not contain any pcd files!" << std::endl;
        return 0;
    }

    // Handle bag directory
    inputParameters.targetDirectory = arguments.at(2);
    dirPath = inputParameters.targetDirectory;
    dirPath.truncate(dirPath.lastIndexOf(QChar('/')));
    if (!std::filesystem::exists(dirPath.toStdString())) {
        std::cerr << "Invalid target directory. Please enter a valid one!" << std::endl;
        return 0;
    }

    // Check for optional arguments
    if (arguments.size() > 3) {
        // Topic name
        if (!Utils::CLI::continueWithInvalidROS2Name(arguments, inputParameters.topicName)) {
            return 0;
        }
    }
    // Apply default topic name if not assigned
    if (inputParameters.topicName.isEmpty()) {
        inputParameters.topicName = "/topic_point_cloud";
    }

    if (std::filesystem::exists(inputParameters.targetDirectory.toStdString())) {
        if (!Utils::CLI::shouldContinue("The bag file already exists. Continue? [y/n]")) {
            return 0;
        }
    }

    // Create thread and connect to its informations
    auto* const pcdsToBagThread = new PCDsToBagThread(inputParameters);

    QObject::connect(pcdsToBagThread, &PCDsToBagThread::openingCVInstanceFailed, [] {
        std::cerr << "The bag creation failed. Please make sure that all inputParameters are set correctly." << std::endl;
        return 0;
    });
    QObject::connect(pcdsToBagThread, &PCDsToBagThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(pcdsToBagThread, &PCDsToBagThread::finished, [] {
        std::cout << "" << std::endl; // Extra line to stop flushing
        std::cout << "Writing finished!" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(pcdsToBagThread, &PCDsToBagThread::finished, pcdsToBagThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Writing pcd files to bag. Please wait..." << std::endl;
    Utils::CLI::runThread(pcdsToBagThread, signalStatus);

    return EXIT_SUCCESS;
}
