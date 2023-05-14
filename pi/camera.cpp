#include "global.hpp"
#ifndef VIRTUAL_TEST
#include "camera.hpp"
#include <array>
#include <iostream>
#include <chrono>
#include <filesystem>
#include <numeric>
#include <thread>
#include <fstream>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>
#include "search.hpp"
#include "Serial.hpp"
namespace fs = std::filesystem;
constexpr auto SIZE_THRESH = 40;
constexpr auto SLICE_SIZE_THRESH = 10;
constexpr auto CONT_SIZE_THRESH = 20000;
void detect(std::atomic<ThreadState> &state, Search **search, std::mutex &map_lock, std::condition_variable &map_cv)
{
    std::string port;
    for (const auto &entry : fs::directory_iterator("/sys/class/tty"))
    {
        const auto &filename = entry.path().filename();
        if (filename.generic_string().rfind("ttyUSB0", 0) == 0)
        {
            port = "/dev/" / filename;
            break;
        }
    }
    Serial serial(port, 9600);
    std::array<cv::VideoCapture, 2> caps{cv::VideoCapture(0, cv::CAP_V4L2), cv::VideoCapture(1, cv::CAP_V4L2)};
    for (auto &cap : caps)
    {
        cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    }
    state = ThreadState::STARTED;
    cv::Mat frame;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    while (state != ThreadState::STOP)
    {
        std::unique_lock<std::mutex> cond_lock(map_lock);
        map_cv.wait(cond_lock, [&search, &state]
                    { return !(*search)->get_current_vic() || state == ThreadState::STOP; });
        cond_lock.unlock();
        for (std::uint8_t i = 0; i < caps.size() && state != ThreadState::STOP; i++)
        {
            caps[i] >> frame;
            std::uint8_t n_kits = 0;
            bool vic = false;
            switch (color_detect(frame))
            {
            case Color::RED:
                n_kits = 1;
                std::cout << "red" << std::endl;
                break;
            case Color::YELLOW:
                n_kits = 1;
                std::cout << "yellow" << std::endl;
                break;
            case Color::GREEN:
                vic = true;
                std::cout << "green" << std::endl;
                break;
            case Color::UNKNOWN:
                std::cout << "unknown color" << std::endl;
                break;
            }
            std::cout << "between" << std::endl;
            switch (letter_detect(frame))
            {
            case Letter::H:
                n_kits = 3;
                std::cout << "H" << std::endl;
                break;
            case Letter::S:
                n_kits = 2;
                std::cout << "S" << std::endl;
                break;
            case Letter::U:
                vic = true;
                std::cout << "U" << std::endl;
                break;
            case Letter::UNKNOWN:
                std::cout << "unknown letter" << std::endl;
                break;
            }
            /*
            if (n_kits || vic)
            {
                cond_lock.lock();
                (*search)->set_current_vic();
                cond_lock.unlock();
                serial.write(static_cast<std::uint8_t>(0));
                serial.write(n_kits);
                serial.write(i);
            }
            */
           if (i == 0)
            cv::imshow("fr0", frame);
        else
                    cv::imshow("fr1", frame);

        }
        if (cv::waitKey(50) & 0xFF == 'q') break;
    }
}
Color::color color_detect(const cv::Mat &frame)
{
    static const std::array<cv::Scalar, 6> bounds{cv::Scalar(100, 118, 88), cv::Scalar(200, 255, 255), cv::Scalar(14, 107, 129), cv::Scalar(40, 188, 255), cv::Scalar(55, 42, 25), cv::Scalar(90, 172, 80)};
    auto color_ratio = 1 / 10.;
    auto color = Color::UNKNOWN;
    cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);
    cv::Mat filt_frame;
    for (std::uint8_t i = 0; i < bounds.size() && color == Color::UNKNOWN; i += 2)
    {
        cv::inRange(frame, bounds[i], bounds[i + 1], filt_frame);
        auto nonzero = cv::countNonZero(filt_frame);
        auto size = filt_frame.cols * filt_frame.rows;
        if (double cur_ratio; (cur_ratio = nonzero / static_cast<double>(size)) > color_ratio)
        {
            color = static_cast<Color::color>(i / 2);
            color_ratio = cur_ratio;
        }
    }
    cv::cvtColor(frame, frame, cv::COLOR_HSV2BGR);
    return color;
}
Letter::letter letter_detect(cv::Mat &frame)
{
    std::array<int, 3> letterCount{};
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    //cv::convertScaleAbs(frame, frame, 1.5, 4);
    cv::medianBlur(frame, frame, 5);
    cv::GaussianBlur(frame, frame, cv::Size(7, 7), 0);
    cv::adaptiveThreshold(frame, frame, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 13, 5);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(frame, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(frame, contours, -1, cv::Scalar(255, 255, 255), -1);
    
    for (const auto &contour : contours)
    {
        auto rect = cv::boundingRect(contour);
        auto rw = rect.width;
        auto rh = rect.height;
        auto angle = cv::minAreaRect(contour).angle;
        if (rw < SIZE_THRESH || rh < SIZE_THRESH)
            continue;
        auto letter = frame(rect);
        auto mat = cv::getRotationMatrix2D(cv::Point(letter.cols / 2, letter.rows / 2), angle, 1);
        cv::warpAffine(letter, letter, mat, cv::Size(letter.cols, letter.rows), cv::INTER_CUBIC);
        auto area = cv::contourArea(contour);
        if (area <= 0)
            continue;
        std::vector<std::vector<cv::Point>> letterContours;
        cv::findContours(letter, letterContours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        if (letterContours.empty()) continue;
        std::vector<cv::Point> largestContour = letterContours[0];
        for (auto &c : letterContours)
            if (cv::contourArea(c) > cv::contourArea(largestContour))
                largestContour = c;
        letter = letter(cv::boundingRect(largestContour));
        if (letter.cols > letter.rows)
            cv::rotate(letter, letter, cv::ROTATE_90_CLOCKWISE);
        cv::imshow("img", letter);
        auto sliceSize = letter.rows / 3;
        std::array<cv::Mat, 3> slices{letter(cv::Rect(0, 0, letter.cols, sliceSize)), letter(cv::Rect(0, sliceSize, letter.cols, sliceSize)), letter(cv::Rect(0, 2 * sliceSize, letter.cols, sliceSize))};
        std::array<int, 3> sliceContours{};
        cv::imshow("sl0", slices[0]);
        cv::imshow("sl1", slices[1]);
        cv::imshow("sl2", slices[2]);
        constexpr std::array<int, 3> h{2, 1, 2};
        constexpr std::array<int, 3> s{1, 1, 1};
        constexpr std::array<int, 3> u{2, 2, 1};
        constexpr std::array<int, 3> u_rot{1, 2, 2};
        for (int i = 0; i < slices.size(); i++)
        {
            cv::findContours(slices[i], letterContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            int count = 0;
            for (auto &sliceContour : letterContours)
            {
                auto sliceRect = cv::minAreaRect(sliceContour);
                if (sliceRect.size.width > SLICE_SIZE_THRESH)
                    count++;
            }
            sliceContours[i] = count;
        }
        if (sliceContours == h)
            letterCount[Letter::H]++;
        else if (sliceContours == s)
            letterCount[Letter::S]++;
        else if (sliceContours == u || sliceContours == u_rot)
            letterCount[Letter::U]++;
    }
    auto max = std::max_element(letterCount.begin(), letterCount.end());
    if (*max == 0)
        return Letter::UNKNOWN;
    return static_cast<Letter::letter>(max - letterCount.begin());
}
#endif