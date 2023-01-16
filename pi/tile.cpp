#include "global.hpp"
#include "tile.hpp"
#include <iostream>
tile::tile() : props{} {}
bool &tile::operator[](std::uint8_t index)
{
    return props[index];
}
void tile::set_vis()
{
    props[Prop::VIS] = true;
}
bool tile::vis() const
{
    return props[Prop::VIS];
}
void tile::set_vic()
{
    props[Prop::VIC] = true;
}
bool tile::vic() const
{
    return props[Prop::VIC];
}
void tile::print() const
{
    std::cout << "Walls: ";
    for (std::uint32_t i = 0; i < 4; i++)
    {
        std::cout << props[i];
    }
    std::cout << "\nVisited: " << props[Prop::VIS] << '\n';
}