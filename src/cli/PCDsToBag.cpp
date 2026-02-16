#include "PCDsToBagThread.hpp"

#include "UtilsCLI.hpp"
#include "UtilsUI.hpp"

#include <QCoreApplication>
#include <QObject>

#include <filesystem>
#include <iostream>

volatile sig_atomic_t signalStatus = 0;

void
showHelp()
{
    std::cout << "Usage: ros2 run ros2_utils_tool tool_pcds_to_bag path/to/pcds/dir path/to/bag\n\n";
    std::cout << "Additional parameters:\n";
    std::cout << "-t or --topic_name: Topic name in the bag file. If no topic name is specified, the name '/topic_point_cloud' will be used.\n";
    std::cout << "-r or --rate: Number of messages per second. Minimum is 1, maximum is 30, default is 5.\n\n";
    std::cout << "-s or --suppress: Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run ros2_utils_tool tool_pcds_to_bag /home/usr/pcd_dir /home/usr/output_bag -t /scanner_pcd -r 2\n\n";
    std::cout << "-h or --help: Show this help.\n";
}


int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);

    const auto& arguments = app.arguments();
    if (arguments.size() < 3 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    const QVector<QString> checkList{ "-t", "-r", "-s", "--topic_name", "--rate", "--suppress" };
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

    if (!Utils::CLI::continueExistingTargetLowDiskSpace(arguments, parameters.targetDirectory)) {
        return 0;
    }

    // Create thread and connect to its informations
    auto* const pcdsToBagThread = new PCDsToBagThread(parameters);

    QObject::connect(pcdsToBagThread, &PCDsToBagThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(pcdsToBagThread, &PCDsToBagThread::finished, [] {
        std::cout << "\n"; // Extra line to stop flushing
        std::cout << "Writing finished!\n";
        return EXIT_SUCCESS;
    });
    QObject::connect(pcdsToBagThread, &PCDsToBagThread::finished, pcdsToBagThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Source pcd directory: " << std::filesystem::absolute(parameters.sourceDirectory.toStdString()) << "\n";
    std::cout << "Target bag file: " << std::filesystem::absolute(parameters.targetDirectory.toStdString()) << "\n";
    std::cout << "Topic name: " << parameters.topicName.toStdString() << "\n";
    std::cout << "Rate: " << parameters.rate << " point clouds per second\n\n";
    std::cout << "Please wait...\n";
    Utils::CLI::runThread(pcdsToBagThread, signalStatus);

    return EXIT_SUCCESS;
}
