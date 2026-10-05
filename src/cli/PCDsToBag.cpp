#include "PCDsToBagThread.hpp"

#include "UtilsCLI.hpp"

#include <QCoreApplication>

#include <filesystem>
#include <iostream>

void
helpFunction()
{
    std::cout << "Usage: ros2 run ros2_utils_tool tool_pcds_to_bag [-h] [files_dir] [output_bag_dir] [-t TOPIC] [-r RATE] [-s]\n\n";
    std::cout << "Convert a dir of pcd files to a bag file.\n\n";
    std::cout << "positional arguments:\n";
    std::cout << "  files_dir             Source files directory.\n";
    std::cout << "  output_bag_dir        Output bag file.\n\n";
    std::cout << "options:\n";
    std::cout << "  -h, --help            Show this help message and exit.\n";
    std::cout << "  -t TOPIC, --topic TOPIC\n";
    std::cout << "                        Point cloud messages topic name, defaults to '/topic_point_cloud'.\n";
    std::cout << "  -r RATE, --rate RATE  Number of messages per second. Minimum is 1, maximum is 30, defaults to 5.\n";
    std::cout << "  -s, --suppress        Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run ros2_utils_tool tool_pcds_to_bag /home/usr/pcd_files_dir /home/usr/output_bag -t /scanner_pcd -r 2" << std::endl;
}


int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);

    const auto& arguments = app.arguments();
    if (Utils::CLI::showHelpAndExitEarly(arguments, helpFunction, 3)) {
        return 0;
    }

    const QVector<QString> checkList{ "-t", "-r", "-s", "--topic", "--rate", "--suppress" };
    Utils::CLI::checkForInvalidParameters(arguments, checkList, helpFunction);

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

    // Create thread and run the operation
    auto* const pcdsToBagThread = new PCDsToBagThread(parameters);
    std::cout << "Source pcd directory: " << std::filesystem::absolute(parameters.sourceDirectory.toStdString()) << "\n";
    std::cout << "Target bag file: " << std::filesystem::absolute(parameters.targetDirectory.toStdString()) << "\n";
    std::cout << "Topic name: " << parameters.topicName.toStdString() << "\n";
    std::cout << "Rate: " << parameters.rate << " point clouds per second\n\n";
    std::cout << "Please wait...\n";
    Utils::CLI::runThread(pcdsToBagThread, "Writing finished!");

    return EXIT_SUCCESS;
}
