// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/common/Common.h>
#include <carbon/common/Pimpl.h>

#include <ostream>
#include <memory>


namespace epic {
namespace carbon {

/**
    Abstract class to represent progress reporting implementation.

    Simple utility that can be implemented to act as a CLI progress
    reporter, or maybe part of a more complex GUI application.
*/
class AbstractProgressReporter {
    public:
        /**
            Update progress.

            Make a single step toward finishing the progress that is
            defined using StartProgress method.

            Should change the internal state of the implementation,
            to account for progress update with respect to the process
            length defined with parameter numSteps in StartProgress
            method.
        */
        virtual void Update() = 0;

        /**
            Start tracking progress for a particular process.

            @param message Message that should describe the process.
            @param numSteps Defines how many steps / iterations this progress will contain.

            @note Based on numSteps update method calculates how much to advance the
                progress counters.
        */
        virtual void StartProgress(const char* message, std::size_t numSteps) = 0;

        /**
            Mark the ending of a progress.
        */
        virtual void EndProgress() = 0;
};


/**
    A simple command-line interface progress reporter implementation.

    An example of a fully printed out CMD progress line:
    Reading images   |==================================================| 100%, done in 44.3064 seconds.

    It includes a single line progress update (equal signs '=' in the example above though the character can be changed),
    estimated time printout while progress is running, and elapsed time took to finish the process once it is done.
*/
class CLIProgressReporter : public AbstractProgressReporter {
    public:
        /**
            Default Constructor.

            @param barLength How many character will there be to represent the 100% process.
            @param barStart The offset character count, acting as a TAB character to denote
                the offset between the message and progress bar.
            @param barPin Character that is used to represent a single pin inside the progress bar.
        */
        CLIProgressReporter(unsigned int barLength = 50, unsigned int barStart = 40, char barPin = '=');

        ~CLIProgressReporter();

        /// Silence the reporter output (ignore it's update to the stream)
        void Silence();

        /// Continue pushing printouts to the stream.
        void Unsilence();

        /**
            Set custom output stream to which the utility prints progress.

            By default, constructor sets this field to std::cout.

            @warning Stream instance must live as long as it is the lifetime of
                this instance. CLIProgressReporter does not take ownership of
                the stream instance.
        */
        void SetStream(std::ostream* stream);

        void StartProgress(const char* message, std::size_t numSteps) override;
        void EndProgress() override;
        void Update() override;

    private:
        struct Impl;
        Pimpl<Impl> m_pimpl;

        void PrintProgress();
        void PrintProgressEnd();
};
}
}
