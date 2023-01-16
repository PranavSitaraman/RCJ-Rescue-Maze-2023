#include "global.hpp"
#include "search.hpp"
#include "camera.hpp"
#include <thread>
#include <mutex>
#include <array>
#include <condition_variable>
#include <filesystem>
int main(int argc, char **argv)
{
#ifndef VIRTUAL_TEST
    std::atomic<ThreadState> thread_state = ThreadState::INIT;
#endif
    std::mutex map_lock;
    std::condition_variable map_cv;
    std::string port;
#ifndef VIRTUAL_TEST
    for (const auto &entry : std::filesystem::directory_iterator("/sys/class/tty"))
    {
        const auto &filename = entry.path().filename();
        if (filename.generic_string().rfind("ttyACM", 0) == 0)
            port = "/dev/" / filename;
    }
    Serial serial(port, 9600);
    std::array<Search, 2> searches{(!std::filesystem::exists("/home/pi/map1")) ? Search(serial, map_lock, map_cv, "/home/pi/map1") : Search(serial, "/home/pi/map1", map_lock, map_cv), (!std::filesystem::exists("/home/pi/map2")) ? Search(serial, map_lock, map_cv, "/home/pi/map1") : Search(serial, "/home/pi/map2", map_lock, map_cv)};
#else
    Serial serial(NULL);
    std::array<Search, 2> searches{(!std::filesystem::exists("/home/pranav/map1")) ? Search(serial, map_lock, map_cv, "/home/pranav/map1") : Search(serial, "/home/pranav/map1", map_lock, map_cv), (!std::filesystem::exists("/home/pranav/map2")) ? Search(serial, map_lock, map_cv, "/home/pranav/map1") : Search(serial, "/home/pranav/map2", map_lock, map_cv)};
#endif
    Search *search = &searches[0];
    std::uint8_t current = 0;
#ifndef VIRTUAL_TEST
    std::thread camera_thread(&detect, std::ref(thread_state), &search, std::ref(map_lock), std::ref(map_cv));
#endif
    std::stack<std::uint8_t> path;
    search->check_walls();
#ifdef VIRTUAL_TEST
    search->print_map();
#endif
#ifndef VIRTUAL_TEST
    while (thread_state == ThreadState::INIT)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif
restart:
    while (!(path = search->search()).empty())
    {
        if (!search->move(path))
        {
            current = !current;
            map_lock.lock();
            search = &searches[current];
            search->map[search->y][search->x][Dir::S] = true;
            map_lock.unlock();
        }
#ifdef VIRTUAL_TEST
        search->print_map();
#endif
        search->check_walls();
#ifdef VIRTUAL_TEST
        search->print_map();
#endif
    }
    if (search == &searches[1])
    {
        map_lock.lock();
        search->map[search->y][search->x][Dir::S] = false;
        search->unmark_start();
        map_lock.unlock();
        goto restart;
    }
    map_lock.lock();
    search->unmark_start();
    map_lock.unlock();
    search->move(search->search());
#ifdef VIRTUAL_TEST
    search->print_map();
#endif
#ifndef VIRTUAL_TEST
    thread_state = ThreadState::STOP;
    camera_thread.join();
#endif
}