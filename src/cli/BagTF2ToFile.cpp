#include "TF2ToFileThread.hpp"

#include "Parameters.hpp"
#include "UtilsCLI.hpp"
#include "UtilsGeneral.hpp"
#include "UtilsROS.hpp"

#include <QCoreApplication>
#include <QFileInfo>
#include <QObject>

#include <filesystem>
#include <iostream>

volatile sig_atomic_t signalStatus = 0;

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_tf2_to_file path/to/bag path/to/output/file\n\n";
    std::cout << "Accepted file formats are json or yaml.\n\n";
    std::cout << "Additional parameters:\n";
    std::cout << "-t or --topic_name: tf2 topic inside the bag. If no topic name is specified, the first found tf2 topic in the bag is taken.\n\n";
    std::cout << "-k or --keep_timestamps: Keep the message's timestamp in the output file.\n";
    std::cout << "-i or --indent: Indent the output file (json only).\n\n";
    std::cout << "-s or --suppress: Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run mediassist4_ros_tools tool_tf2_to_file /home/usr/input_bag /home/usr/output_file.json -k -i\n\n";
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

    const QVector<QString> checkList{ "-t", "-i", "-k", "-s", "--topic_name", "--indent", "--keep_timestamps", "--suppress" };
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList); argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::TF2ToFileParameters parameters;

    // Source bag directory
    parameters.sourceDirectory = arguments.at(1);
    Utils::CLI::checkBagSourceDirectory(parameters.sourceDirectory);
    // Target file directory
    parameters.targetDirectory = arguments.at(2);
    Utils::CLI::checkParentDirectory(parameters.targetDirectory);
    if (const auto fileExtension = Utils::General::getFileExtension(parameters.targetDirectory); fileExtension != "json" && fileExtension != "yaml") {
        throw std::runtime_error("Please enter either 'json' or 'yaml' for the format!");
    }

    // Check for optional arguments
    if (arguments.size() > 3) {
        // Topic name
        Utils::CLI::checkTopicNameValidity(arguments, parameters.sourceDirectory, "tf2_msgs/msg/TFMessage", parameters.topicName);
        // Timestamps
        parameters.keepTimestamps = Utils::CLI::containsArguments(arguments, "-k", "--keep_timestamps");
        // Indenting
        parameters.compactOutput = !Utils::CLI::containsArguments(arguments, "-i", "--indent");
    }

    // Search for topic name in bag file if not specified
    if (parameters.topicName.isEmpty()) {
        Utils::CLI::checkForTargetTopic(parameters.sourceDirectory, parameters.topicName, "tf2_msgs/msg/TFMessage");
    }

    if (!Utils::CLI::continueExistingTargetLowDiskSpace(arguments, parameters.targetDirectory)) {
        return 0;
    }

    // Create thread and connect to its informations
    auto* const tf2ToFileThread = new TF2ToFileThread(parameters);
    QObject::connect(tf2ToFileThread, &TF2ToFileThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(tf2ToFileThread, &TF2ToFileThread::finished, [] {
        std::cout << "\n"; // Extra line to stop flushing
        std::cout << "Writing finished!\n";
        return EXIT_SUCCESS;
    });
    QObject::connect(tf2ToFileThread, &TF2ToFileThread::finished, tf2ToFileThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Source bag file: " << std::filesystem::absolute(parameters.sourceDirectory.toStdString()) << "\n";
    std::cout << "Target file: " << std::filesystem::absolute(parameters.targetDirectory.toStdString()) << "\n";
    std::cout << "Topic name: " << parameters.topicName.toStdString() << "\n\n";
    std::cout << "Please wait...\n";
    Utils::CLI::runThread(tf2ToFileThread, signalStatus);

    return EXIT_SUCCESS;
}
