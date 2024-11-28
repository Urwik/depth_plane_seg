#ifndef TIMER_HPP
#define TIMER_HPP

#include <iostream>
#include <chrono>

namespace cr = std::chrono;


namespace utils {
    class Timer;
}

class Timer {
    public:
        Timer() : start_time_point(cr::high_resolution_clock::now()) {}

        void start() {
            start_time_point = cr::high_resolution_clock::now();
        }

        void end() {
            end_time_point = cr::high_resolution_clock::now();
        }

        long long currentElapsedTime() const {
            cr::time_point<cr::high_resolution_clock> current_time_point = cr::high_resolution_clock::now();
            return cr::duration_cast<cr::milliseconds>(current_time_point - start_time_point).count();
        }

        long long totalElapsedTime() const {
            return cr::duration_cast<cr::milliseconds>(end_time_point - start_time_point).count();
        }

    private:
        cr::time_point<cr::high_resolution_clock> start_time_point;
        cr::time_point<cr::high_resolution_clock> end_time_point;
};

#endif