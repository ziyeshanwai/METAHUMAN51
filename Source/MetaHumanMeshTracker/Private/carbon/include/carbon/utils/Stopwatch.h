// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <chrono>
#include <vector>
#include <numeric>  // for accumulate


namespace epic {
namespace carbon {

/**
    Simple, header-only utility for measuring time based on std::chrono.
 */
template<class Timer = std::chrono::steady_clock>
class Stopwatch {
    public:
        Stopwatch() : m_tick(Timer::now()) {
        }

        ~Stopwatch() {
        }

        /**
            Tick the stopwatch - start counting anew.

            @return Last counted time segment, before the tick.
        */
        std::chrono::duration<double> Tick() {
            auto const lastTick = m_tick;
            m_tick = Timer::now();

            // store the time
            m_allTimes.push_back(m_tick - lastTick);

            return this->LastTime();
        }

        /**
            Reset all so far collected times.
        */
        void Reset() {
            m_tick = Timer::now();
            m_allTimes.clear();
        }

        /// Get what is the last measured time.
        std::chrono::duration<double> LastTime() {
            return m_allTimes.back();
        }

        /// Get what is currently elapsed time since the last tick.
        std::chrono::duration<double> ElapsedTime() {
            auto const lastTick = this->m_tick;
            auto const nowTick = Timer::now();

            return nowTick - lastTick;
        }

        /// Get what is the total elapsed time since the first tick of the stopwatch instance.
        std::chrono::duration<double> TotalTime() {
            std::chrono::duration<double> duration{};  // zero duration
            return std::accumulate(this->m_allTimes.begin(), this->m_allTimes.end(), duration);
        }

        /// Get list of all times that is collected in this stopwatch instance.
        const std::vector<std::chrono::duration<double> >& AllTimes() const {
            return this->m_allTimes;
        }

    private:
        std::chrono::time_point<Timer> m_tick;
        std::vector<std::chrono::duration<double> > m_allTimes;
};

/// Create a stopwatch instance based on std::chrono::steady_clock.
Stopwatch<std::chrono::steady_clock> SteadyStopwatch() {
    return Stopwatch<std::chrono::steady_clock>();
}

/// Create a stopwatch instance based on std::chrono::high_resolution_clock.
Stopwatch<std::chrono::high_resolution_clock> HighResolutionStopwatch() {
    return Stopwatch<std::chrono::high_resolution_clock>();
}

/// Create a stopwatch instance based on std::chrono::system_clock.
Stopwatch<std::chrono::system_clock> SystemStopwatch() {
    return Stopwatch<std::chrono::system_clock>();
}

}  // carbon
}  // epic
