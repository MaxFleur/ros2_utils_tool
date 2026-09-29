#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to write one or multiple bag topics to a yaml or json file
class BagMessageToFileThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    BagMessageToFileThread(const Parameters::BagMessageToFileParameters& parameters,
                           QObject*                                      parent = nullptr);

    void
    run() override;

private:
    const Parameters::BagMessageToFileParameters& m_parameters;
};
