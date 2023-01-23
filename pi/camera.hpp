#ifndef CAMERA_HPP
#define CAMERA_HPP
#include "global.hpp"
#ifndef VIRTUAL_TEST
#include <atomic>
#include <mutex>
#include <condition_variable>
#include "search.hpp"
enum class ThreadState : std::uint8_t
{
    INIT,
    STARTED,
    STOP
};
Color::color color_detect(const cv::Mat &frame);
Letter::letter letter_detect(cv::Mat &frame);
void detect(std::atomic<ThreadState> &state, Search **search, std::mutex &map_lock, std::condition_variable &map_cv);
#endif
#endif