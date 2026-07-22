#include "BagToImagesThread.hpp"

#include "Parameters.hpp"
#include "UtilsCLI.hpp"

#include <QCoreApplication>
#include <QObject>

#include <filesystem>
#include <iostream>

volatile sig_atomic_t signalStatus = 0;

void
showHelp()
{
    std::cout << "Usage: ros2 run ros2_utils_tool tool_bag_to_images [-h] [bag_path] [output_files_path] [-f {jpg,png,bmp}] [-t TOPIC_NAME]\n";
    std::cout << "                                                   [-th NUMBER_OF_THREADS] [-c] [-e] [-b] [-o] [-q QUALITY] [-s]\n\n";
    std::cout << "Convert bag image messages to a list of files\n\n";
    std::cout << "positional arguments:\n";
    std::cout << "  bag_path              Source bag file.\n";
    std::cout << "  output_files_path     Directory containing the output image files.\n\n";
    std::cout << "options:\n";
    std::cout << "  -h, --help            Show this help message and exit.\n";
    std::cout << "  -f {jpg,png,bmp}, --format {jpg,png,bmp}\n";
    std::cout << "                        File format, defaults to jpg.\n";
    std::cout << "  -t TOPIC_NAME, --topic_name TOPIC_NAME\n";
    std::cout << "                        Bag image topic to convert. If no topic name is specified, the first found image topic is taken.\n";
    std::cout << "  -th NUMBER_OF_THREADS, --threads NUMBER_OF_THREADS\n";
    std::cout << "                        Number of threads used for writing. Minimum is 1, maximum is " << std::thread::hardware_concurrency() << ", defaults to 1.\n";
    std::cout << "  -c, --colorless       Encode images without color.\n";
    std::cout << "  -e, --exchange        Exchange red and blue values.\n";
    std::cout << "  -b, --binary          Write images with only black and white pixels, png only.\n";
    std::cout << "  -o, --optimize        Optimize jpg file size, jpg only.\n";
    std::cout << "  -q QUALITY, --quality QUALITY\n";
    std::cout << "                        Image quality. Minimum is 0, maximum is 9, defaults to 8. jpg and png only.\n";
    std::cout << "  -s, --suppress        Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run ros2_utils_tool tool_bag_to_images /home/usr/input_bag /home/usr/images_dir -t /endoscope_video -f png -q 8 -th 4 -e" << std::endl;
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

    const QVector<QString> checkList{ "-f", "-t", "-th", "-c", "-e", "-b", "-o", "-q", "-s",
                                      "--format", "--topic_name", "--threads", "--colorless", "--exchange", "--binary", "--optimize", "--quality", "--suppress" };
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
        Utils::CLI::checkTopicNameValidity(arguments, parameters.sourceDirectory, { "sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage" }, parameters.topicName);
        // Quality
        if (!Utils::CLI::checkArgumentValidity(arguments, "-q", "--quality", parameters.quality, 0, 9)) {
            throw std::runtime_error("Please enter a quality value in the range of 0 to 9!");
        }
        // Format
        if (Utils::CLI::containsArguments(arguments, "-f", "--format")) {
            parameters.format = arguments.at(Utils::CLI::getFormatIndex(arguments, { "jpg", "png", "bmp" }));
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
        Utils::CLI::checkForTargetTopic(parameters.sourceDirectory, parameters.topicName, { "sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage" });
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
    // Start operation
    Utils::CLI::runThread(bagToImagesThread, signalStatus);

    return EXIT_SUCCESS;
}
