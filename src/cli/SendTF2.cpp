#include "SendTF2Thread.hpp"

#include "UtilsCLI.hpp"
#include "UtilsGeneral.hpp"
#include "UtilsIO.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCoreApplication>
#include <QObject>

#include <filesystem>
#include <iostream>

volatile sig_atomic_t signalStatus = 0;

void
showHelp()
{
    std::cout << "Usage: ros2 run mediassist4_ros_tools tool_send_tf2\n\n";
    std::cout << "Additional parameters:\n";
    std::cout << "-t or --translation: Translation. Must be in format \"value,value,value\", default is \"0.0,0.0,0.0\".\n";
    std::cout << "-ro or --rotation: Rotation. Must be in format \"value,value,value,value\", default is \"0.0,0.0,0.0,1.0\".\n";
    std::cout << "-c or --child_frame_name: Child frame name.\n\n";
    std::cout << "-r or --rate: Rate (only for nonstatic transformations). Must be between 1 and 100. If this is not specified, the transformation is static.\n\n";
    std::cout << "-f or --file: Provide an input file containing data for translation, rotation and the child frame name. Supported formats are json and yaml.\n";
    std::cout << "-s or --save: Save the input translation to a json or yaml file. Cannot be used together with -f.\n\n";
    std::cout << "Example usages:\n";
    std::cout << "ros2 run mediassist4_ros_tools tool_send_tf2 -t \"0.1,0.2,0.3\" -ro \"1.0,2.0,3.0,1.5\" -c base_link -s save_for_later.json\n";
    std::cout << "ros2 run mediassist4_ros_tools tool_send_tf2 -f save_for_later.json\n\n";
    std::cout << "-h or --help: Show this help.\n";
}


void
showInfo(const Parameters::SendTF2Parameters& parameters, bool isStatic)
{
    const auto staticInfoString = isStatic ? "static " : "nonstatic ";

    std::cout << "Sending " << staticInfoString << "transformation with following parameters:\n"
              << "translation:\n"
              << "    x: " << parameters.translation[0] << "\n" << "    y: " << parameters.translation[1] << "\n"
              << "    z: " << parameters.translation[2] << "\n"
              << "rotation:\n"
              << "    x: " << parameters.rotation[0] << "\n" << "    y: " << parameters.rotation[1] << "\n"
              << "    z: " << parameters.rotation[2] << "\n" << "    w: " << parameters.rotation[3] << "\n"
              << "child frame name: " << parameters.childFrameName.toStdString() << "\n";
    if (isStatic) {
        return;
    }

    std::cout << "Rate: " << parameters.rate << " transformations per second\n\n";
    std::cout << "Sending...\n";
}


int
main(int argc, char* argv[])
{
    // Initialize ROS and Qt
    rclcpp::init(argc, argv);
    QCoreApplication app(argc, argv);

    const auto& arguments = app.arguments();
    if (arguments.size() < 1 || arguments.contains("--help") || arguments.contains("-h")) {
        showHelp();
        return 0;
    }

    const QVector<QString> checkList{ "-t", "-ro", "-c", "-r", "-f", "-s",
                                      "--translation", "--rotation", "--child_frame_name", "--rate", "--file", "--save" };
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList); argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::SendTF2Parameters parameters;
    if (Utils::CLI::containsArguments(arguments, "-f", "--file")) {
        const auto filePath = arguments.at(Utils::CLI::getArgumentsIndex(arguments, "-f", "--file") + 1);

        const auto isJson = Utils::General::getFileExtension(filePath) == "json";
        const auto readSuccessful = isJson ? Utils::IO::readTF2FromJson(filePath, parameters) : Utils::IO::readTF2FromYAML(filePath, parameters);
        if (!readSuccessful) {
            throw std::runtime_error(isJson ? "Invalid json input file! Please make sure to save a valid file first using '-s'."
                                            : "Invalid yaml input file! Please make sure to save a valid file first using '-s'.");
        }
    } else {
        // A char array means something like 0.12345. We have to convert this to a double value
        const auto charArrayToDouble = [arguments] (const std::string& transformationType, int argumentsIndex, int expectedArraySize) {
            std::stringstream ss(arguments.at(argumentsIndex).toStdString());
            std::string temp;
            std::vector<double> doubleValues;
            auto loopCounter = 0;

            while (getline(ss, temp, ',')) {
                loopCounter++;

                try {
                    doubleValues.emplace_back(std::stod(temp));
                } catch (...) {
                    throw std::runtime_error("Invalid arguments for " + transformationType + "!");
                }
            }
            // xyz for translations and xyzw for rotation
            if (loopCounter != expectedArraySize) {
                throw std::runtime_error("Please insert the correct number of values for " + transformationType +
                                         " (expected " + std::to_string(expectedArraySize) + ")!");
            }

            return doubleValues;
        };

        // Translation
        if (Utils::CLI::containsArguments(arguments, "-t", "--translation")) {
            const auto translationIndex = Utils::CLI::getArgumentsIndex(arguments, "-t", "--translation") + 1;
            const auto& translationValues = charArrayToDouble("translation", translationIndex, 3);

            parameters.translation[0] = translationValues.at(0);
            parameters.translation[1] = translationValues.at(1);
            parameters.translation[2] = translationValues.at(2);
        }
        // Rotation
        if (Utils::CLI::containsArguments(arguments, "-ro", "--rotation")) {
            const auto rotationIndex = Utils::CLI::getArgumentsIndex(arguments, "-ro", "--rotation") + 1;
            const auto& rotationValues = charArrayToDouble("rotation", rotationIndex, 4);

            parameters.rotation[0] = rotationValues.at(0);
            parameters.rotation[1] = rotationValues.at(1);
            parameters.rotation[2] = rotationValues.at(2);
            parameters.rotation[3] = rotationValues.at(3);
        }

        // Check for optional arguments
        // Child frame name
        if (Utils::CLI::containsArguments(arguments, "-c", "--child_frame_name")) {
            parameters.childFrameName = arguments.at(Utils::CLI::getArgumentsIndex(arguments, "-c", "--child_frame_name") + 1);
        }
        // Save called
        if (Utils::CLI::containsArguments(arguments, "-s", "--save")) {
            const auto filePath = arguments.at(Utils::CLI::getArgumentsIndex(arguments, "-s", "--save") + 1);

            if (Utils::General::getFileExtension(filePath) == "json") {
                if (!Utils::IO::writeTF2ToJson(filePath, parameters)) {
                    throw std::runtime_error("Failed writing json file!");
                }
            } else if (!Utils::IO::writeTF2ToYAML(filePath, parameters)) {
                throw std::runtime_error("Failed writing yaml file!");
            }
            std::cout << "Saved file " << filePath.toStdString() << "\n";
        }
    }

    auto nodeWrapper = std::make_shared<NodeWrapper>("ros2_utils_tool_tf_node");
    if (!Utils::CLI::containsArguments(arguments, "-r", "--rate")) {
        showInfo(parameters, true);
        Utils::ROS::sendStaticTransformation(parameters.translation, parameters.rotation, nodeWrapper);
        std::cout << "\nTF sent!\n";
        return EXIT_SUCCESS;
    }

    // Rate
    parameters.isStatic = false;
    if (!Utils::CLI::checkArgumentValidity(arguments, "-r", "--rate", parameters.rate, 1, 100)) {
        throw std::runtime_error("Please enter a rate in the range of 1 to 100!");
    }

    // Create thread and connect to its informations
    auto* const sendTF2Thread = new SendTF2Thread(parameters);

    QObject::connect(sendTF2Thread, &SendTF2Thread::finished, sendTF2Thread, &QObject::deleteLater);

    signal(SIGINT, [] (int signal) {
        signalStatus = signal;
    });

    showInfo(parameters, false);
    Utils::CLI::runThread(sendTF2Thread, signalStatus);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
