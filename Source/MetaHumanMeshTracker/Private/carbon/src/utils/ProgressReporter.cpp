// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/utils/ProgressReporter.h>

#include <carbon/utils/Stopwatch.h>
#include<cmath>

#include <iostream>


namespace epic::carbon {

struct CLIProgressReporter::Impl {

    float progressUpdate;  // how many steps are there to complete the progress;
    float progress;  // where is the updater currently;

    std::string message;
    bool isSilent;
    unsigned int barLength;
    unsigned int barStart;
    char barPin;
    Stopwatch<std::chrono::steady_clock> stopwatch;
    std::ostream* stream;
};

CLIProgressReporter::CLIProgressReporter(unsigned int barLength, unsigned int barStart, char barPin)
    : m_pimpl(new CLIProgressReporter::Impl) {
    m_pimpl->isSilent = false;
    m_pimpl->barLength = barLength;
    m_pimpl->barStart = barStart;
    m_pimpl->barPin = barPin;
    m_pimpl->stream = &std::cout;
}

CLIProgressReporter::~CLIProgressReporter() {
}

void CLIProgressReporter::StartProgress(const char* message, std::size_t numSteps) {
    m_pimpl->message = message;
    m_pimpl->progressUpdate = 1.f / static_cast<float>(numSteps);
    m_pimpl->progress = 0.f;

    this->PrintProgress();  // initial printing (empty bar)

    // tick the stopwatch to initiate the progress time tracking
    m_pimpl->stopwatch.Tick();
}

void CLIProgressReporter::EndProgress() {
    m_pimpl->stopwatch.Tick();

    m_pimpl->progress = 1.f;

    this->PrintProgress();  // final printing (full bar)
    this->PrintProgressEnd();  // and add the time elapsed report, plus newline.
}

void CLIProgressReporter::Silence() {
    m_pimpl->isSilent = true;
}

void CLIProgressReporter::Unsilence() {
    m_pimpl->isSilent = false;
}

void CLIProgressReporter::SetStream(std::ostream* stream) {
    CARBON_PRECONDITION(stream, "Given stream must not be nullptr.");
    m_pimpl->stream = stream;
}

void CLIProgressReporter::PrintProgressEnd() {
    if (!m_pimpl->isSilent) {
        *(m_pimpl->stream) << ", done in " << m_pimpl->stopwatch.LastTime().count() << " seconds." << std::endl;
    }
}

std::string ClockFromSeconds(int seconds) {
    int minutes = seconds / 60;
    seconds %= 60;
    int hours = minutes / 60;
    minutes %= 60;

    std::string clockStr;
    clockStr.reserve(8);  // hh:mm:ss

    if (hours < 10) {
        clockStr += "0";
    }
    clockStr += std::to_string(hours);
    clockStr += ":";
    if (minutes < 10) {
        clockStr += "0";
    }
    clockStr += std::to_string(minutes);
    clockStr += ":";
    if (seconds < 10) {
        clockStr += "0";
    }
    clockStr += std::to_string(seconds);

    return clockStr;
}

void CLIProgressReporter::PrintProgress() {
    if (m_pimpl->isSilent) {
        return;
    }

    int barFilled = static_cast<int>(m_pimpl->barLength * m_pimpl->progress);
    int barEmpty = m_pimpl->barLength - barFilled;

    std::string bar(barFilled, m_pimpl->barPin);
    std::string empty(barEmpty, ' ');

    std::string percentStr = std::to_string(static_cast<int>(std::ceil(m_pimpl->progress * 100.f)));

    if (percentStr.length() != 3) {  // not a 100%, add some spaces in front
        percentStr = std::string(3 - static_cast<int>(percentStr.length()), ' ') + percentStr;
    }

    int barSpacing = static_cast<int>(m_pimpl->barStart) - static_cast<int>(m_pimpl->message.length());
    std::string barSpacingStr;
    if (barSpacing > 0) {
        barSpacingStr = std::string(barSpacing, ' ');
    }

    *(m_pimpl->stream) << "\r" <<
    m_pimpl->message << barSpacingStr << "|" << bar << empty << "| " << percentStr << "%";

    // print elapsed time if relevant
    const double elapsedTime = m_pimpl->stopwatch.ElapsedTime().count();
    if ((m_pimpl->progress > 1e-3f) && (m_pimpl->progress < 1.f) && (elapsedTime > 1.0)) {
        double timePerUpdate = elapsedTime / m_pimpl->progress;
        int timeRemaining = static_cast<int>((1.f - m_pimpl->progress) * timePerUpdate);
        std::string clockStr = ClockFromSeconds(timeRemaining);
        *(m_pimpl->stream) << " (ETA " << clockStr << ")";
    }

    m_pimpl->stream->flush();
}

void CLIProgressReporter::Update() {
    m_pimpl->progress += m_pimpl->progressUpdate;
    this->PrintProgress();
}

}
