#pragma once

#include "calibration/calibrator.hpp"
#include "io/rosbag_container.h"
#include "io/aprilgrid_container.h"

#include "BS_thread_pool.hpp"

#include <memory>
#include <string>

/*
 * Meyers Singleton class to hold the state of the application with thread-safe initialization.
 * Provide global access to the application state instance.
 * */
class AppState {
private:
    // Constructor and destructor should be private
    AppState() : rosbag_files(), selected_bag(0), thread_pool(/*std::thread::hardware_concurrency() - 1*/20) {}
    ~AppState() = default;

    int selected_bag = 0; // selected rosbag_file for display
protected:
    BS::thread_pool thread_pool;

public:
    // Delete copy and move constructors and assign operators
    AppState(AppState const&) = delete;             // Copy construct
    AppState(AppState&&) = delete;                  // Move construct
    AppState& operator=(AppState const&) = delete;  // Copy assign
    AppState& operator=(AppState &&) = delete;      // Move assign

    static AppState& get_instance() {
        static AppState instance;
        return instance;
    }

    /*
     * Push tasks to thread pool, does not return std::future
     * */
    template <typename F, typename... A>
    static void submit_task(F&& task, A&&... args) {
        auto &app_state = AppState::get_instance();
        app_state.thread_pool.push_task(std::forward<F>(task), std::forward<A>(args)...);

    }

    void load_dataset();
    void load_aprilgrid();

    /*
     * Made public since we have the [] operator overloaded. No point making a getter. (e.g app_state.get_selected(idx))
     * is bloated.
     * */
    RosbagContainer rosbag_files;
    AprilGridContainer aprilgrid_files;
};