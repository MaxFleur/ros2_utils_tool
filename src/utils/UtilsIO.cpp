#include "UtilsIO.hpp"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include "yaml-cpp/yaml.h"

#include <fstream>

namespace Utils::IO
{
bool
readTF2FromJson(const QString& path, Parameters::SendTF2Parameters& parameters)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return false;
    }

    const auto rootObject = QJsonDocument::fromJson(file.readAll()).object();
    file.close();

    const auto transformsArray = rootObject["transforms"].toArray();
    if (transformsArray.isEmpty()) {
        return false;
    }

    const auto dataObject = transformsArray.at(0).toObject();
    if (!dataObject.contains("child_frame_id") || !dataObject.contains("transform")) {
        return false;
    }

    const auto transformObject = dataObject["transform"].toObject();
    if (!transformObject.contains("translation") || !transformObject.contains("rotation")) {
        return false;
    }

    const auto translationObject = transformObject["translation"].toObject();
    if (!translationObject.contains("x") || !translationObject.contains("y") || !translationObject.contains("z")) {
        return false;
    }

    const auto rotationObject = transformObject["rotation"].toObject();
    if (!rotationObject.contains("x") || !rotationObject.contains("y") || !rotationObject.contains("z") || !rotationObject.contains("w")) {
        return false;
    }

    parameters.childFrameName = dataObject["child_frame_id"].toString();
    parameters.translation[0] = translationObject["x"].toDouble();
    parameters.translation[1] = translationObject["y"].toDouble();
    parameters.translation[2] = translationObject["z"].toDouble();
    parameters.rotation[0] = rotationObject["x"].toDouble();
    parameters.rotation[1] = rotationObject["y"].toDouble();
    parameters.rotation[2] = rotationObject["z"].toDouble();
    parameters.rotation[3] = rotationObject["w"].toDouble();
    return true;
}


bool
readTF2FromYAML(const QString& path, Parameters::SendTF2Parameters& parameters)
{
    const auto rootNode = YAML::LoadFile(path.toStdString());
    if (!rootNode["transforms"]) {
        return false;
    }
    if (!rootNode["transforms"]["child_frame_id"] || !rootNode["transforms"]["transform"]) {
        return false;
    }
    if (!rootNode["transforms"]["transform"]["translation"] || !rootNode["transforms"]["transform"]["rotation"]) {
        return false;
    }

    const auto& translationNode = rootNode["transforms"]["transform"]["translation"];
    if (!translationNode["x"] || !translationNode["y"] || !translationNode["z"]) {
        return false;
    }
    const auto& rotationNode = rootNode["transforms"]["transform"]["rotation"];
    if (!rotationNode["x"] || !rotationNode["y"] || !rotationNode["z"] || !rotationNode["w"]) {
        return false;
    }

    parameters.childFrameName = QString::fromStdString(rootNode["transforms"]["child_frame_id"].as<std::string>());
    parameters.translation[0] = translationNode["x"].as<double>();
    parameters.translation[1] = translationNode["y"].as<double>();
    parameters.translation[2] = translationNode["z"].as<double>();
    parameters.rotation[0] = rotationNode["x"].as<double>();
    parameters.rotation[1] = rotationNode["y"].as<double>();
    parameters.rotation[2] = rotationNode["z"].as<double>();
    parameters.rotation[3] = rotationNode["w"].as<double>();
    return true;
}


bool
writeTF2ToJson(const QString& path, Parameters::SendTF2Parameters& parameters)
{
    QJsonObject translationObject;
    translationObject["x"] = parameters.translation[0];
    translationObject["y"] = parameters.translation[1];
    translationObject["z"] = parameters.translation[2];
    QJsonObject rotationObject;
    rotationObject["x"] = parameters.rotation[0];
    rotationObject["y"] = parameters.rotation[1];
    rotationObject["z"] = parameters.rotation[2];
    rotationObject["w"] = parameters.rotation[3];

    QJsonObject transformObject;
    transformObject["translation"] = translationObject;
    transformObject["rotation"] = rotationObject;

    QJsonObject dataObject;
    dataObject["child_frame_id"] = parameters.childFrameName;
    dataObject["transform"] = transformObject;
    QJsonArray array;
    array.append(dataObject);

    QJsonObject rootObject;
    rootObject["transforms"] = array;
    QJsonDocument document(rootObject);

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        return false;
    }
    if (!file.write(document.toJson(QJsonDocument::Indented))) {
        return false;
    }

    file.close();
    return true;
}


bool
writeTF2ToYAML(const QString& path, Parameters::SendTF2Parameters& parameters)
{
    YAML::Node node;
    node["transforms"]["child_frame_id"] = parameters.childFrameName.toStdString();
    node["transforms"]["transform"]["translation"]["x"] = parameters.translation[0];
    node["transforms"]["transform"]["translation"]["y"] = parameters.translation[1];
    node["transforms"]["transform"]["translation"]["z"] = parameters.translation[2];
    node["transforms"]["transform"]["rotation"]["x"] = parameters.rotation[0];
    node["transforms"]["transform"]["rotation"]["y"] = parameters.rotation[1];
    node["transforms"]["transform"]["rotation"]["z"] = parameters.rotation[2];
    node["transforms"]["transform"]["rotation"]["w"] = parameters.rotation[3];

    std::ofstream fout(path.toStdString());
    try {
        fout << node;
    } catch (std::ofstream::failure& /* exeption */) {
        return false;
    }

    return true;
}
}
