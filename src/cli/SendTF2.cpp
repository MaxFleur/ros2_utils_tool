#include "SendTF2Thread.hpp"

#include "UtilsCLI.hpp"
#include "UtilsGeneral.hpp"
#include "UtilsIO.hpp"
#include "UtilsROS.hpp"

#include <QCoreApplication>
#include <QObject>

#include <iostream>

volatile sig_atomic_t signalStatus = 0;

void
showHelp()
{
    std::cout << "Usage: ros2 run ros2_utils_tool tool_send_tf2 [-h] [--child-frame-name NAME]\n";
    std::cout << "                                              [--rotation \"VALUE,VALUE,VALUE,VALUE\"] [--translation \"VALUE,VALUE,VALUE\"]\n";
    std::cout << "                                              [-i FILE_NAME.{json,yaml}] [--save FILE_NAME.{json,yaml}] [-r RATE]\n\n";
    std::cout << "Publish a static or nonstatic transformation as a tf2 message.\n\n";
    std::cout << "options:\n";
    std::cout << "  -h, --help            Show this help message and exit.\n";
    std::cout << "  --child-frame-name NAME\n";
    std::cout << "                        Child frame name.\n";
    std::cout << "  --rotation \"VALUE,VALUE,VALUE,VALUE\"\n";
    std::cout << "                        Rotation, defaults to \"0.0,0.0,0.0,1.0\".\n";
    std::cout << "  --translation \"VALUE,VALUE,VALUE\"\n";
    std::cout << "                        Translation, defaults to \"0.0,0.0,0.0\".\n";
    std::cout << "  -i FILE_NAME.{json,yaml}, --input FILE_NAME.{json,yaml}\n";
    std::cout << "                        Provide an input file containing transformation data. Cannot be used together with -s.\n";
    std::cout << "  --save FILE_NAME.{json,yaml}\n";
    std::cout << "                        Save the input translation to a json or yaml file. Cannot be used together with -i.\n";
    std::cout << "  -r RATE, --rate RATE  Number of transformations per second. Minimum is 1, maximum is 100. TF is static if unspecified.\n\n";
    std::cout << "Example usage:\n";
    std::cout << "ros2 run ros2_utils_tool tool_send_tf2 --child-frame-name base_link --translation \"0.1,0.2,0.3\" --rotation \"1.0,2.0,3.0,1.5\" --save save_for_later.json\n";
    std::cout << "ros2 run ros2_utils_tool tool_send_tf2 -i save_for_later.json" << std::endl;
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
    if (arguments.size() < 1 || Utils::CLI::containsArguments(arguments, "-h", "--help")) {
        showHelp();
        return 0;
    }

    const QVector<QString> checkList{ "-i", "-r", "--input", "--rate", "--child-frame-name", "--rotation", "--translation", "--save" };
    if (const auto& argument = Utils::CLI::containsInvalidParameters(arguments, checkList); argument != std::nullopt) {
        showHelp();
        throw std::runtime_error("Unrecognized argument '" + *argument + "'!");
    }

    Parameters::SendTF2Parameters parameters;
    if (Utils::CLI::containsArguments(arguments, "-i", "--input")) {
        const auto filePath = arguments.at(Utils::CLI::getArgumentsIndex(arguments, "-i", "--input") + 1);

        const auto isJson = Utils::General::getFileExtension(filePath) == "json";
        const auto readSuccessful = isJson ? Utils::IO::readTF2FromJson(filePath, parameters) : Utils::IO::readTF2FromYAML(filePath, parameters);
        if (!readSuccessful) {
            const std::string format = isJson ? "json" : "yaml";
            throw std::runtime_error("Invalid " + format + " input file! Please make sure to save a valid file first using '-s'.");
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
        if (arguments.contains("--translation")) {
            const auto& translationValues = charArrayToDouble("translation", arguments.indexOf("--translation") + 1, 3);

            parameters.translation[0] = translationValues.at(0);
            parameters.translation[1] = translationValues.at(1);
            parameters.translation[2] = translationValues.at(2);
        }
        // Rotation
        if (arguments.contains("--rotation")) {
            const auto& rotationValues = charArrayToDouble("rotation", arguments.indexOf("--rotation") + 1, 4);

            parameters.rotation[0] = rotationValues.at(0);
            parameters.rotation[1] = rotationValues.at(1);
            parameters.rotation[2] = rotationValues.at(2);
            parameters.rotation[3] = rotationValues.at(3);
        }

        // Check for optional arguments
        // Child frame name
        if (arguments.contains("--child-frame-name")) {
            parameters.childFrameName = arguments.at(arguments.indexOf("--child-frame-name") + 1);
        }
        // Save called
        if (arguments.contains("--save")) {
            const auto filePath = arguments.at(arguments.indexOf("--save") + 1);

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
    // Start operation
    Utils::CLI::runThread(sendTF2Thread, signalStatus);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
