#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to write images out of a ROS bag
class BagToImagesThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    BagToImagesThread(const Parameters::BagToImagesParameters& parameters,
                      QObject*                                 parent = nullptr);

    void
    run() override;

private:
    const Parameters::BagToImagesParameters& m_parameters;
};
