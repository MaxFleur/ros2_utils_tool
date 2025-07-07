#include "BagToImagesThread.hpp"

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
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_bag_to_images path/to/bag path/to/images\n" << std::endl;
    std::cout << "Additional parameters:" << std::endl;
    std::cout << "-t or --topic_name: Video topic inside the bag. If no topic name is specified, the first found video topic in the bag is taken.\n" << std::endl;
    std::cout << "-f or --format: Must be jpg, png or bmp (jpg is default)." << std::endl;
    std::cout << "-e or --exchange: Exchange red and blue values." << std::endl;
    std::cout << "-c or --colorless: Encode images without color.\n" << std::endl;
    std::cout << "-q 0-9 or --quality 0-9 (jpg and png only): Image quality, must be between 0 and 9 (9 is highest, 8 is default)." << std::endl;
    std::cout << "-o or --optimize (jpg only): Optimize jpg file size." << std::endl;
    std::cout << "-b or --binary (png only): Write images with only black and white pixels.\n" << std::endl;
    std::cout << "-h or --help: Show this help." << std::endl;
}


volatile sig_atomic_t signalStatus = 0;

int
main(int argc, char* argv[])
{
    // Create application
    QCoreApplication app(argc, argv);

    const auto& arguments = app.arguments();
    const QStringList checkList{ "-t", "-f", "-e", "-c", "-q", "-o", "-b", "-h",
                                 "--topic_name", "--format", "--exchange", "--colorless", "--quality", "--optimize", "--binary", "--help" };
    if (arguments.size() < 3 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }
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
            if (arguments.at(qualityFormatIndex) == arguments.last() ||
                (arguments.at(qualityFormatIndex + 1) != "jpg" && arguments.at(qualityFormatIndex + 1) != "png" && arguments.at(qualityFormatIndex + 1) != "bmp")) {
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

    // Search for topic name in bag file if not specified
    if (parameters.topicName.isEmpty()) {
        Utils::CLI::checkForTargetTopic(parameters.sourceDirectory, parameters.topicName, true);
    }

    if (std::filesystem::exists(parameters.targetDirectory.toStdString())) {
        if (!Utils::CLI::shouldContinue("The image directory already exists. Continue? [y]/n")) {
            return 0;
        }
    }

    // Create thread and connect to its informations
    auto* const bagToImagesThread = new BagToImagesThread(parameters, std::thread::hardware_concurrency());
    QObject::connect(bagToImagesThread, &BagToImagesThread::progressChanged, [] (const QString& progressString, int progress) {
        const auto progressStringCMD = Utils::CLI::drawProgressString(progress);
        // Always clear the last line for a nice "progress bar" feeling
        std::cout << progressString.toStdString() << " " << progressStringCMD << " " << progress << "%" << "\r" << std::flush;
    });
    QObject::connect(bagToImagesThread, &BagToImagesThread::finished, [] {
        std::cout << "" << std::endl; // Extra line to stop flushing
        std::cout << "Writing images finished!" << std::endl;
        return EXIT_SUCCESS;
    });
    QObject::connect(bagToImagesThread, &BagToImagesThread::finished, bagToImagesThread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    std::cout << "Writing images. Please wait..." << std::endl;
    Utils::CLI::runThread(bagToImagesThread, signalStatus);

    return EXIT_SUCCESS;
}
