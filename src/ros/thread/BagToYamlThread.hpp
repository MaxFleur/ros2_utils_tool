#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to write one or multiple bag topics to a yaml file
class BagToYamlThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    BagToYamlThread(const Parameters::BagToYamlParameters& parameters,
                    QObject*                               parent = nullptr);

    void
    run() override;

private:
    const Parameters::BagToYamlParameters& m_parameters;
};
