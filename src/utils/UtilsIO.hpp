#pragma once

#include "Parameters.hpp"

#include <QString>

// Util functions for writing data to and reading data from files
namespace Utils::IO
{
// Reads tf2 values stored in a json file into a parameter instance
[[nodiscard]] bool
readTF2FromJson(const QString&                 path,
                Parameters::SendTF2Parameters& parameters);

// Reads tf2 values stored in a yaml file into a parameter instance
[[nodiscard]] bool
readTF2FromYAML(const QString&                 path,
                Parameters::SendTF2Parameters& parameters);

// Writes send tf2 parameter values to json
[[nodiscard]] bool
writeTF2ToJson(const QString&                 path,
               Parameters::SendTF2Parameters& parameters);

// Writes send tf2 parameter values to yaml
[[nodiscard]] bool
writeTF2ToYAML(const QString&                 path,
               Parameters::SendTF2Parameters& parameters);
}
