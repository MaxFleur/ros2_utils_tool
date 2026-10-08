#include "BagTF2ToFileThread.hpp"

#include "Parameters.hpp"
#include "UtilsCLI.hpp"
#include "UtilsGeneral.hpp"

#include <QCoreApplication>

#include <filesystem>
#include <iostream>

void
helpFunction()
{
    std::cout << "Usage: ros2 run ros2_utils_tool tool_bag_tf2_to_file [-h] [bag_path] [output_file_path.{json,yaml}] [-t TOPIC] [--keep-timestamps] [-i] [-s]\n\n";
    std::cout << "Convert bag transformations to file.\n\n";
    std::cout << "positional arguments:\n";
    std::cout << "  bag_path              Source bag file.\n";
    std::cout << "  output_file_path.{json,yaml}\n";
    std::cout << "                        The file containing the transformation(s). Accepted file formats are json or yaml.\n\n";
    std::cout << "options:\n";
    std::cout << "  -h, --help            Show this help message and exit.\n";
    std::cout << "  -t TOPIC, --topic TOPIC\n";
    std::cout << "                        Bag tf2 topic to convert. If no topic name is specified, the first found topic with type tf2 is taken.\n";
    std::cout << "  --keep-timestamps\n";
    std::cout << "                        Keep the message's timestamp in the output file.\n";
    std::cout << "  -i, --indent          Indent the output file. json only.\n";
    std::cout << "  -s, --suppress        Suppress any warnings.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run ros2_utils_tool tool_bag_tf2_to_file /home/usr/input_bag /home/usr/output_file.json --keep-timestamps -i" << std::endl;
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

    const QVector<QString> checkList{ "-t", "-i", "-s", "--topic", "--indent", "--suppress", "--keep-timestamps" };
    Utils::CLI::checkForInvalidParameters(arguments, checkList, helpFunction);

    Parameters::BagTF2ToFileParameters parameters;

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
        Utils::CLI::checkTopicNameValidity(arguments, parameters.sourceDirectory, { "tf2_msgs/msg/TFMessage" }, parameters.topicName);
        // Timestamps
        parameters.keepTimestamps = arguments.contains("--keep-timestamps");
        // Indenting
        parameters.compactOutput = !Utils::CLI::containsArguments(arguments, "-i", "--indent");
    }

    // Search for topic name in bag file if not specified
    if (parameters.topicName.isEmpty()) {
        Utils::CLI::checkForTargetTopic(parameters.sourceDirectory, parameters.topicName, { "tf2_msgs/msg/TFMessage" });
    }

    if (!Utils::CLI::continueExistingTargetLowDiskSpace(arguments, parameters.targetDirectory)) {
        return 0;
    }

    // Create thread and run the operation
    auto* const bagTF2ToFileThread = new BagTF2ToFileThread(parameters);
    std::cout << "Source bag file: " << std::filesystem::absolute(parameters.sourceDirectory.toStdString()) << "\n";
    std::cout << "Target file: " << std::filesystem::absolute(parameters.targetDirectory.toStdString()) << "\n";
    std::cout << "Topic name: " << parameters.topicName.toStdString() << "\n\n";
    std::cout << "Please wait...\n";
    Utils::CLI::runThread(bagTF2ToFileThread, "Writing finished!");

    return EXIT_SUCCESS;
}
