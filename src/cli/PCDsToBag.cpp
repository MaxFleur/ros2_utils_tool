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
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_pcds_to_bag path/to/pcds/dir path/to/bag\n" << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-t or --topic_name: Topic name in the bag file. If this is empty, the name '/topic_point_cloud' will be used.\n" << std::endl;
    std::cout << "-r or --rate: Rate ('clouds per second'). Must be between 1 and 30, default is 5.\n" << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


volatile sig_atomic_t signalStatus = 0;

int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);

    const auto arguments = app.arguments();
    const QStringList checkList{ "-h", "--help", "-t", "--topic_name", "-r", "--rate" };
    if (arguments.size() < 3 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList); argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::PCDsToBagParameters parameters;

    // PCDs directory
    parameters.sourceDirectory = arguments.at(1);
    Utils::CLI::checkParentDirectory(parameters.sourceDirectory, false);

    auto containsPCDFiles = false;
    for (auto const& entry : std::filesystem::directory_iterator(parameters.sourceDirectory.toStdString())) {
        if (entry.path().extension() == ".pcd") {
            containsPCDFiles = true;
            break;
        }
    }
    if (!containsPCDFiles) {
        throw std::runtime_error("The entered directory for the pcd files does not contain any pcd files!");
    }

    // Handle bag directory
    parameters.targetDirectory = arguments.at(2);
    Utils::CLI::checkParentDirectory(parameters.targetDirectory);

    // Check for optional arguments
    if (arguments.size() > 3) {
        // Topic name
        if (!Utils::CLI::continueWithInvalidROS2Name(arguments, parameters.topicName)) {
            return 0;
        }
        // Rate
        if (!Utils::CLI::checkArgumentValidity(arguments, "-r", "--rate", parameters.rate, 1, 30)) {
            throw std::runtime_error("Please enter a rate in the range of 1 to 30!");
        }
    }
    // Apply default topic name if not assigned
    if (parameters.topicName.isEmpty()) {
        parameters.topicName = "/topic_point_cloud";
    }

    if (std::filesystem::exists(parameters.targetDirectory.toStdString())) {
        if (!Utils::CLI::shouldContinue("The bag file already exists. Continue? [y/n]")) {
            return 0;
        }
    }

    // Create thread and connect to its informations
    auto* const pcdsToBagThread = new PCDsToBagThread(parameters);

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
