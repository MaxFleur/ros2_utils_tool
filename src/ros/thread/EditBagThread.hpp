#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

// Thread used to create a new, edited bag file out of an existing one
class EditBagThread : public BasicThread {
    Q_OBJECT
public:
    explicit
    EditBagThread(const Parameters::EditBagParameters& parameters,
                  unsigned int                         numberOfThreads,
                  QObject*                             parent = nullptr);

    void
    run() override;

private:
    const Parameters::EditBagParameters& m_parameters;

    const unsigned int m_numberOfThreads;
};
