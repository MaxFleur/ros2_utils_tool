#include "BagToImagesThread.hpp"

#include "Parameters.hpp"
#include "UtilsCLI.hpp"
#include "UtilsROS.hpp"

#include <QCoreApplication>
#include <QObject>

#include <filesystem>
#include <iostream>

volatile sig_atomic_t signalStatus = 0;

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_bag_to_images path/to/bag path/to/images\n\n";
    std::cout << "Additional parameters:\n";
    std::cout << "-t or --topic_name: Video topic inside the bag. If no topic name is specified, the first found video topic in the bag is taken.\n";
    std::cout << "-f or --format: Must be jpg, png or bmp (jpg is default).\n";
    std::cout << "-q or --quality (jpg and png only): Image quality, must be between 0 and 9 (0 is lowest, 9 is highest, 8 is default).\n\n";
    std::cout << "-e or --exchange: Exchange red and blue values.\n";
    std::cout << "-o or --optimize (jpg only): Optimize jpg file size.\n";
    std::cout << "-c or --colorless: Encode images without color.\n";
    std::cout << "-b or --binary (png only): Write images with only black and white pixels.\n\n";
    std::cout << "-th or --threads: Number of threads, must be at least 1 (maximum is " << std::thread::hardware_concurrency() << ").\n\n";
    std::cout << "-s or --suppress: Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run mediassist4_ros_tools tool_bag_to_images /home/usr/input_bag /home/usr/images_dir -t /endoscope_video -f png -q 8 -th 4 -e\n\n";
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

    const QVector<QString> checkList{ "-t", "-f", "-e", "-c", "-q", "-o", "-b", "-th", "-s",
                                      "--topic_name", "--format", "--exchange", "--colorless", "--quality", "--optimize", "--binary", "--threads", "--suppress" };
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList); argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::BagToImagesParameters parameters;

    // Handle bag directory
    parameters.sourceDirectory = arguments.at(1);
    Utils::CLI::checkBagSourceDirectory(parameters.sourceDirectory);

    // Images directory
    parameters.targetDirectory = arguments.at(2);
    Utils::CLI::checkParentDirectory(parameters.targetDirectory);

    // Check for optional arguments
    if (arguments.size() > 3) {
        // Topic name
        Utils::CLI::checkTopicNameValidity(arguments, parameters.sourceDirectory, "sensor_msgs/msg/Image", parameters.topicName);
        // Quality
        if (!Utils::CLI::checkArgumentValidity(arguments, "-q", "--quality", parameters.quality, 0, 9)) {
            throw std::runtime_error("Please enter a quality value in the range of 0 to 9!");
        }
        // Format
        if (Utils::CLI::containsArguments(arguments, "-f", "--format")) {
            const auto qualityFormatIndex = Utils::CLI::getArgumentsIndex(arguments, "-f", "--format");
            const QVector<QString> acceptedFormats { "jpg", "png", "bmp" };
            if (arguments.at(qualityFormatIndex) == arguments.last() || !acceptedFormats.contains(arguments.at(qualityFormatIndex + 1))) {
                throw std::runtime_error("Please enter either 'jpg', 'png' or 'bmp' for the format!");
            }
            parameters.format = arguments.at(qualityFormatIndex + 1);
        }

        // Exchange red and blue values
        parameters.exchangeRedBlueValues = Utils::CLI::containsArguments(arguments, "-e", "--exchange");
        // Black white images
        parameters.useBWImages = Utils::CLI::containsArguments(arguments, "-c", "--colorless");
        // Optimize for jpeg
        parameters.jpgOptimize = parameters.format == "jpg" && Utils::CLI::containsArguments(arguments, "-o", "--optimize");
        // Bilevel for png images
        parameters.pngBilevel = parameters.format == "png" && Utils::CLI::containsArguments(arguments, "-b", "--binary");
    }

    // Thread count
    auto numberOfThreads = 1;
    if (!Utils::CLI::checkArgumentValidity(arguments, "-th", "--threads", numberOfThreads, 1, std::thread::hardware_concurrency())) {
        throw std::runtime_error("Please enter a thread count value in the range of 1 to " + std::to_string(std::thread::hardware_concurrency()) + "!");
    }

    // Search for topic name in bag file if not specified
    if (parameters.topicName.isEmpty()) {
        Utils::CLI::checkForTargetTopic(parameters.sourceDirectory, parameters.topicName, "sensor_msgs/msg/Image");
    }

    if (!Utils::CLI::continueExistingTargetLowDiskSpace(arguments, parameters.targetDirectory)) {
        return 0;
    }

    // Create thread and connect to its informations
    auto* const bagToImagesThread = new BagToImagesThread(parameters, numberOfThreads);
    QObject::connect(bagToImagesThread, &BagToImagesThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(bagToImagesThread, &BagToImagesThread::finished, [] {
        std::cout << "\n"; // Extra line to stop flushing
        std::cout << "Writing images finished!\n";
        return EXIT_SUCCESS;
    });
    QObject::connect(bagToImagesThread, &BagToImagesThread::finished, bagToImagesThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Source bag file: " << std::filesystem::absolute(parameters.sourceDirectory.toStdString()) << "\n";
    std::cout << "Target images dir: " << std::filesystem::absolute(parameters.targetDirectory.toStdString()) << "\n";
    std::cout << "Topic name: " << parameters.topicName.toStdString() << "\n";
    std::cout << "Format: " << parameters.format.toStdString() << "\n";
    std::cout << "Number of used threads: " << numberOfThreads << "\n\n";
    std::cout << "Writing images. Please wait...\n";
    Utils::CLI::runThread(bagToImagesThread, signalStatus);

    return EXIT_SUCCESS;
}
