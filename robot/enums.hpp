#pragma once

#include <iostream>
#include <map>

enum RobotMode
{
    off,
    idle,
    manual,
    automatic,
};

enum AutoMode
{
    none,

    learn = 10,
    learn_wait,
    learn_back,
    learn_search,
    learn_line,
    learn_end,

    follow = 20,
    route = 30,
};

inline std::ostream &operator<<(std::ostream &out, const RobotMode value)
{
    static std::map<RobotMode, std::string> strings;
    if (strings.size() == 0)
    {
#define INSERT_ELEMENT(p) strings[p] = #p
        INSERT_ELEMENT(off);
        INSERT_ELEMENT(idle);
        INSERT_ELEMENT(manual);
        INSERT_ELEMENT(automatic);
#undef INSERT_ELEMENT
    }
    return out << strings[value];
}

inline std::ostream &operator<<(std::ostream &out, const AutoMode value)
{
    static std::map<AutoMode, std::string> strings;
    if (strings.size() == 0)
    {
#define INSERT_ELEMENT(p) strings[p] = #p
        INSERT_ELEMENT(none);
        INSERT_ELEMENT(learn);
        INSERT_ELEMENT(learn_wait);
        INSERT_ELEMENT(learn_back);
        INSERT_ELEMENT(learn_search);
        INSERT_ELEMENT(learn_line);
        INSERT_ELEMENT(learn_end);
        INSERT_ELEMENT(follow);
        INSERT_ELEMENT(route);
#undef INSERT_ELEMENT
    }
    return out << strings[value];
}
