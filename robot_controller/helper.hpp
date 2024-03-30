#include <iostream>
#include <list>

#ifndef _HELPER_HPP_
#define _HELPER_HPP_

struct pair_hash_combiner
{
    std::size_t operator()(const std::pair<int, int>& p) const
    {
        std::string s = std::to_string(p.first) + " " + std::to_string(p.second);
        return std::hash<std::string>()(s);
    }
};

double r2d(double decimal);

#endif