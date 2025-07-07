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
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_bag_to_video path/to/bag path/to/video\n" << std::endl;
    std::cout << "Accepted video formats are mp4, mkv or avi." << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-t or --topic_name: Video topic inside the bag. If no topic name is specified, the first found video topic in the bag is taken.\n" << std::endl;
    std::cout << "-r or --rate: Framerate for the encoded video. Must be from 10 to 60." << std::endl;
    std::cout << "-a or --accelerate: Use hardware acceleration." << std::endl;
    std::cout << "-e or --exchange: Exchange red and blue values." << std::endl;
    std::cout << "-c or --colorless: Use colorless images." << std::endl;
    std::cout << "-l or --lossless (mkv only): Use lossless images.\n" << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


volatile sig_atomic_t signalStatus = 0;

int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);

    const auto arguments = app.arguments();
    const QStringList checkList{ "-t", "-r", "-a", "-e", "-c", "-l", "-h",
                                 "--topic_name", "--rate", "--accelerate", "--exchange", "--colorless", "--lossless", "--help" };
    if (arguments.size() < 3 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }
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

    parameters.format = parameters.targetDirectory.right(3);
    if (parameters.format != "mp4" && parameters.format != "mkv" && parameters.format != "avi") {
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
        Utils::CLI::checkForTargetTopic(parameters.sourceDirectory, parameters.topicName, true);
    }

    if (std::filesystem::exists(parameters.targetDirectory.toStdString())) {
        if (!Utils::CLI::shouldContinue("The video already exists. Continue? [y/n]")) {
            return 0;
        }
    }

    // Create thread and connect to its informations
    auto* const encodingThread = new BagToVideoThread(parameters, useHardwareAcceleration);
    QObject::connect(encodingThread, &BagToVideoThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(encodingThread, &BagToVideoThread::finished, [] {
        std::cout << "" << std::endl; // Extra line to stop flushing
        std::cout << "Encoding finished!" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(encodingThread, &BagToVideoThread::finished, encodingThread, &QObject::deleteLater);
    QObject::connect(encodingThread, &BagToVideoThread::failed, [] {
        throw std::runtime_error("The video writing failed. Please make sure that all parameters are set correctly and disable the hardware acceleration, if necessary.");
    });

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Encoding video. Please wait..." << std::endl;
    Utils::CLI::runThread(encodingThread, signalStatus);

    return EXIT_SUCCESS;
}
