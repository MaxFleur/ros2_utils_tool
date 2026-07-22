#include "BagToVideoThread.hpp"

#include "UtilsCLI.hpp"
#include "Parameters.hpp"

#include <QCoreApplication>
#include <QObject>

#include <filesystem>
#include <iostream>

volatile sig_atomic_t signalStatus = 0;

void
showHelp()
{
    std::cout << "Usage: ros2 run ros2_utils_tool tool_bag_to_video [-h] [bag_path] [output_video_path.{mp4,mkv,avi}] [-r RATE]\n";
    std::cout << "                                                  [-t TOPIC_NAME] [-a] [-c] [-e] [-l] [-s]\n\n";
    std::cout << "Convert bag image messages to a video.\n\n";
    std::cout << "positional arguments:\n";
    std::cout << "  bag_path              Source bag file.\n";
    std::cout << "  output_video_path.{mp4,mkv,avi}\n";
    std::cout << "                        Output video file.\n\n";
    std::cout << "options:\n";
    std::cout << "  -h, --help            Show this help message and exit.\n";
    std::cout << "  -r RATE, --rate RATE  Framerate for the encoded video. Minimum is 10, maximum is 60, default is 30.\n";
    std::cout << "  -t TOPIC_NAME, --topic_name TOPIC_NAME\n";
    std::cout << "                        Bag image topic to convert. If no topic name is specified, the first found topic with type image is taken.\n";
    std::cout << "  -a, --accelerate      Use hardware acceleration.\n";
    std::cout << "  -c, --colorless       Encode images without color.\n";
    std::cout << "  -e, --exchange        Exchange red and blue values.\n";
    std::cout << "  -l, --lossless        Use lossless images. mkv only.\n";
    std::cout << "  -s, --suppress        Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run ros2_utils_tool tool_bag_to_video /home/usr/input_bag /home/usr/target_video.mkv -t /endoscope_video -r 20 -a -c -l" << std::endl;
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

    const QVector<QString> checkList{ "-r", "-t", "-a", "-c", "-e", "-l", "-s",
                                      "--rate", "--topic_name", "--accelerate", "--colorless", "--exchange", "--lossless", "--suppress" };
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
        Utils::CLI::checkTopicNameValidity(arguments, parameters.sourceDirectory, { "sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage" }, parameters.topicName);
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
        Utils::CLI::checkForTargetTopic(parameters.sourceDirectory, parameters.topicName, { "sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage" });
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

    std::cout << "Source bag file: " << std::filesystem::absolute(parameters.sourceDirectory.toStdString()) << "\n";
    std::cout << "Target video file: " << std::filesystem::absolute(parameters.targetDirectory.toStdString()) << "\n";
    std::cout << "Rate: " << parameters.fps << "\n";
    std::cout << "Hardware acceleration " << (useHardwareAcceleration ? "enabled" : "disabled") << "\n\n";
    std::cout << "Encoding video. Please wait...\n";
    // Start operation
    Utils::CLI::runThread(encodingThread, signalStatus);

    return EXIT_SUCCESS;
}
