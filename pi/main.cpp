#include "global.hpp"
#include "search.hpp"
#include "camera.hpp"
#include <thread>
#include <mutex>
#include <array>
#include <fstream>
#include <condition_variable>
#include <filesystem>
int main(int argc, char **argv)
{
    auto epoch = std::chrono::high_resolution_clock::from_time_t(0);
    std::cout << "start" << std::endl;
#ifndef VIRTUAL_TEST
    std::atomic<ThreadState> thread_state = ThreadState::INIT;
#endif
    std::mutex map_lock;
    std::condition_variable map_cv;
    std::string port;
    std::uint8_t current = 0;
#ifndef VIRTUAL_TEST
    for (const auto &entry : std::filesystem::directory_iterator("/sys/class/tty"))
    {
        const auto &filename = entry.path().filename();
        if (filename.generic_string().rfind("ttyS0", 0) == 0)
            port = "/dev/" / filename;
    }
    Serial serial(port, 9600);
    std::array<Search, 2> searches{(!std::filesystem::exists("/home/pi/map1")) ? Search(serial, map_lock, map_cv, "/home/pi/map1") : Search(serial, "/home/pi/map1", map_lock, map_cv), (!std::filesystem::exists("/home/pi/map2")) ? Search(serial, map_lock, map_cv, "/home/pi/map2") : Search(serial, "/home/pi/map2", map_lock, map_cv)};
    if (std::filesystem::exists("/home/pi/num"))
    {
        std::ifstream in("/home/pi/num", std::ios::binary);
        in.read(reinterpret_cast<char *>(&current), sizeof(current));
    }
#else
    Serial serial(0);
    std::array<Search, 2> searches{(!std::filesystem::exists("/home/pranav/map1")) ? Search(serial, map_lock, map_cv, "/home/pranav/map1") : Search(serial, "/home/pranav/map1", map_lock, map_cv), (!std::filesystem::exists("/home/pranav/map2")) ? Search(serial, map_lock, map_cv, "/home/pranav/map2") : Search(serial, "/home/pranav/map2", map_lock, map_cv)};
    if (std::filesystem::exists("/home/pranav/num"))
    {
        std::ifstream in("/home/pranav/num", std::ios::binary);
        in.read(reinterpret_cast<char *>(&current), sizeof(current));
    }
#endif
    Search *search = &searches[current];
    std::stack<std::uint8_t> path;
#ifndef VIRTUAL_TEST
    std::thread camera_thread(&detect, std::ref(thread_state), &search, std::ref(map_lock), std::ref(map_cv));
    while (thread_state == ThreadState::INIT)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif
    while (true)
    {
        if (serial.available())
        {
            if (serial.read() == (uint8_t)1) break;
        }
    }
    search->check_walls();
    search->print_map();
restart:
    while (!(path = search->search()).empty())
    {
        uint8_t val = search->move(path);
        switch (val)
        {
        case Result::result::BLACK:
        {
            std::cout << "black" << std::endl;
            search->print_map();
            break;
        }
        case Result::result::RAMP:
        {
            std::cout << "ramp" << std::endl;
            std::uint8_t oldcd = search->cd;
            current = !current;
            map_lock.lock();
            search = &searches[current];
            search->map[search->y][search->x][(search->cd + 2) % 4] = true;
            if (oldcd == 2 && current == 0)
                search->cd = (search->cd + 2) % 4;
            map_lock.unlock();
        }
        case Result::result::SILVER:
        {
            std::cout << "silver" << std::endl;
            search = &searches[!current];
            search->dump_map();
            search = &searches[current];
            search->dump_map();
        }
        case Result::result::SUCCESS:
        {
            std::cout << "success" << std::endl;
            search->print_map();
            search->check_walls();
            search->print_map();
            break;
        }
        }
    }
    if (search == &searches[1])
    {
        map_lock.lock();
        search->unmark_start();
        map_lock.unlock();
        search->move(search->search());
        map_lock.lock();
        search->map[search->y][search->x][Dir::S] = false;
        map_lock.unlock();
        search->print_map();
        search->check_walls();
        search->print_map();
        goto restart;
    }
    map_lock.lock();
    search->unmark_start();
    map_lock.unlock();
    search->move(search->search());
    search->print_map();
#ifndef VIRTUAL_TEST
    thread_state = ThreadState::STOP;
    camera_thread.join();
#endif
}
