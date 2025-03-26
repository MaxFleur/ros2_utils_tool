#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to write a bag images topic to a set of images
class BagToImagesThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    BagToImagesThread(const Parameters::BagToImagesParameters& parameters,
                      unsigned int                             numberOfThreads,
                      QObject*                                 parent = nullptr);

    void
    run() override;

private:
    const Parameters::BagToImagesParameters& m_parameters;

    const unsigned int m_numberOfThreads;
};
