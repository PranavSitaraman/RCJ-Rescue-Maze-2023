#ifndef SEARCH_HPP
#define SEARCH_HPP
#include "global.hpp"
#include "matrix.hpp"
#include "Serial.hpp"
#include <stack>
#include <mutex>
#include <condition_variable>
class Search
{
public:
    Search(Serial &ser, const char *map_dump, std::mutex &lock, std::condition_variable &cv);
    Search(Serial &ser, std::mutex &lock, std::condition_variable &cv, const char *path);
    bool move(std::stack<std::uint8_t> &);
    bool move(std::stack<std::uint8_t> &&);
    void check_walls();
    void print_map() const;
    [[nodiscard]] bool get_current_vic() const;
    void set_current_vic();
    [[nodiscard]] bool get_current_vis();
    void dump_map();
    void unmark_start();
    [[nodiscard]] std::stack<std::uint8_t> search() const;
    Matrix map;
    std::int32_t x;
    std::int32_t y;

private:
    std::uint8_t cd;
    std::mutex &map_lock;
    std::condition_variable &map_cv;
    std::uint32_t init_x;
    std::uint32_t init_y;
    Serial &serial;
    const char *filename;
};
#endif