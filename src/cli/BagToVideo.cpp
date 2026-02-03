#include "BagToVideoThread.hpp"

#include "UtilsCLI.hpp"
#include "Parameters.hpp"
#include "UtilsROS.hpp"

#include <QCoreApplication>
#include <QObject>

#include <filesystem>
#include <iostream>

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_bag_to_video path/to/bag path/to/video\n\n";
    std::cout << "Accepted video formats are mp4, mkv or avi.\n\n";
    std::cout << "Additional parameters:\n";
    std::cout << "-t or --topic_name: Video topic inside the bag. If no topic name is specified, the first found video topic in the bag is taken.\n";
    std::cout << "-r or --rate: Framerate for the encoded video. Must be from 10 to 60.\n\n";
    std::cout << "-a or --accelerate: Use hardware acceleration.\n";
    std::cout << "-e or --exchange: Exchange red and blue values.\n";
    std::cout << "-c or --colorless: Use colorless images.\n";
    std::cout << "-l or --lossless (mkv only): Use lossless images.\n\n";
    std::cout << "-s or --suppress: Suppress any warnings.\n\n";
    std::cout << "-h or --help: Show this help.\n";
}


volatile sig_atomic_t signalStatus = 0;

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

    const QVector<QString> checkList{ "-t", "-r", "-a", "-e", "-c", "-l", "-s",
                                      "--topic_name", "--rate", "--accelerate", "--exchange", "--colorless", "--lossless", "--suppress" };
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList); argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::BagToVideoParameters parameters;

    // Handle bag directory
    parameters.sourceDirectory = arguments.at(1);
    Utils::CLI::checkBagSourceDirectory(parameters.sourceDirectory);

    // Video directory
    parameters.targetDirectory = arguments.at(2);
    Utils::CLI::checkParentDirectory(parameters.targetDirectory);

    if (const QVector<QString> acceptedFormats { "mp4", "mkv", "avi" }; !acceptedFormats.contains(parameters.targetDirectory.right(3))) {
        throw std::runtime_error("The entered video name is in invalid format. Please make sure that the video has the ending 'mp4', 'mkv' or 'avi'!");
    }

    // Check for optional arguments
    auto useHardwareAcceleration = false;

    if (arguments.size() > 3) {
        // Topic name
        Utils::CLI::checkTopicNameValidity(arguments, parameters.sourceDirectory, "sensor_msgs/msg/Image", parameters.topicName);
        // Framerate
        if (!Utils::CLI::checkArgumentValidity(arguments, "-r", "--rate", parameters.fps, 10, 60)) {
            throw std::runtime_error("Please enter a framerate in the range of 10 to 60!");
        }

        // Hardware acceleration
        useHardwareAcceleration = Utils::CLI::containsArguments(arguments, "-a", "--accelerate");
        // Exchange red and blue values
        parameters.exchangeRedBlueValues = Utils::CLI::containsArguments(arguments, "-e", "--exchange");
        // Colorless
        parameters.useBWImages = Utils::CLI::containsArguments(arguments, "-c", "--colorless");
        // Lossless
        parameters.lossless = Utils::CLI::containsArguments(arguments, "-l", "--lossless");
    }

    // Search for topic name in bag file if not specified
    if (parameters.topicName.isEmpty()) {
        Utils::CLI::checkForTargetTopic(parameters.sourceDirectory, parameters.topicName, "sensor_msgs/msg/Image");
    }

    if (!Utils::CLI::continueExistingTargetLowDiskSpace(arguments, parameters.targetDirectory)) {
        return 0;
    }

    // Create thread and connect to its informations
    auto* const encodingThread = new BagToVideoThread(parameters, useHardwareAcceleration);
    QObject::connect(encodingThread, &BagToVideoThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(encodingThread, &BagToVideoThread::finished, [] {
        std::cout << "\n"; // Extra line to stop flushing
        std::cout << "Encoding finished!\n";
        return EXIT_SUCCESS;
    });
    QObject::connect(encodingThread, &BagToVideoThread::finished, encodingThread, &QObject::deleteLater);
    QObject::connect(encodingThread, &BagToVideoThread::failed, [] {
        throw std::runtime_error("The video writing failed. Please make sure that all parameters are set correctly and disable the hardware acceleration, if necessary.");
    });

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Encoding video. Please wait...\n";
    Utils::CLI::runThread(encodingThread, signalStatus);

    return EXIT_SUCCESS;
}
