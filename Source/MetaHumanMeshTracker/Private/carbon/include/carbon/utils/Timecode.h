// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/Common.h>
#include <carbon/common/Format.h>

#include <math.h>
#include <string>

namespace epic::carbon {

/**
 * Class to calculate Timecode
 */
class Timecode {
public:
    Timecode() = default;

    Timecode(int hours, int minutes, int seconds, int frames, double fps)
        : m_hours(hours)
        , m_minutes(minutes)
        , m_seconds(seconds)
        , m_frames(frames)
        , m_fps(fps)
    {
        MakeCanonical();
    }

    void AddSeconds(double seconds)
    {
        m_frames += seconds * m_fps;
        MakeCanonical();
    }

    void Round()
    {
        m_frames = double(static_cast<int>(m_frames + 0.5));
        MakeCanonical();
    }

    void MakeCanonical()
    {
        const int additionalSeconds = static_cast<int>(m_frames / m_fps);
        m_frames = fmod(m_frames, m_fps);
        m_seconds += additionalSeconds;
        const int additionalMinutes = m_seconds / 60;
        m_seconds = m_seconds % 60;
        m_minutes += additionalMinutes;
        const int additionalHours = m_minutes / 60;
        m_minutes = m_minutes % 60;
        m_hours += additionalHours;
    }

    std::string to_string() const
    {
        char str[64];
        snprintf(str, 64, "%02i:%02i:%02i:%02i", m_hours, m_minutes, m_seconds, static_cast<int>(m_frames));
        return std::string(str);
    }

    double GetFps() const { return m_fps; }

private:
    int m_hours;
    int m_minutes;
    int m_seconds;
    double m_frames;
    double m_fps;
};

}  // namespace epic::carbon
