#include "TF2ToJsonThread.hpp"

#include "Parameters.hpp"
#include "UtilsCLI.hpp"
#include "UtilsROS.hpp"

#include <QCoreApplication>
#include <QObject>

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_tf2_to_json path/to/bag path/to/output.json\n" << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-t or --topic_name: tf2 topic inside the bag. If no topic name is specified, the first found tf2 topic in the bag is taken.\n" << std::endl;
    std::cout << "-i or --indent: Indent the output file." << std::endl;
    std::cout << "-k or --keep_timestamps: Keep the message's timestamp in the json file." << std::endl;
    std::cout << "-s or --suppress: Suppress any warnings.\n" << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


volatile sig_atomic_t signalStatus = 0;

int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);

    const auto& arguments = app.arguments();
    const QStringList checkList{ "-t", "-i", "-k", "-s", "-h", "--topic_name", "--indent", "--keep_timestamps", "--suppress", "--help" };
    if (arguments.size() < 3 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList); argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::TF2ToJsonParameters parameters;

    // Source bag directory
    parameters.sourceDirectory = arguments.at(1);
    Utils::CLI::checkBagSourceDirectory(parameters.sourceDirectory);
    // json file directory
    parameters.targetDirectory = arguments.at(2);
    Utils::CLI::checkParentDirectory(parameters.targetDirectory);

    // Check for optional arguments
    if (arguments.size() > 3) {
        // Topic name
        Utils::CLI::checkTopicNameValidity(arguments, parameters.sourceDirectory, "tf2_msgs/msg/TFMessage", parameters.topicName);

        // Formatting
        parameters.compactOutput = !Utils::CLI::containsArguments(arguments, "-i", "--indent");
        // Timestamps
        parameters.keepTimestamps = Utils::CLI::containsArguments(arguments, "-k", "--keep_timestamps");
    }

    // Search for topic name in bag file if not specified
    if (parameters.topicName.isEmpty()) {
        Utils::CLI::checkForTargetTopic(parameters.sourceDirectory, parameters.topicName, "tf2_msgs/msg/TFMessage");
    }

    if (!Utils::CLI::continueExistingTargetLowDiskSpace(arguments, parameters.targetDirectory)) {
        return 0;
    }

    // Create thread and connect to its informations
    auto* const tf2ToJsonThread = new TF2ToJsonThread(parameters);
    QObject::connect(tf2ToJsonThread, &TF2ToJsonThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(tf2ToJsonThread, &TF2ToJsonThread::finished, [] {
        std::cout << "" << std::endl; // Extra line to stop flushing
        std::cout << "Writing json finished!" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(tf2ToJsonThread, &TF2ToJsonThread::finished, tf2ToJsonThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Writing json. Please wait..." << std::endl;
    Utils::CLI::runThread(tf2ToJsonThread, signalStatus);

    return EXIT_SUCCESS;
}
