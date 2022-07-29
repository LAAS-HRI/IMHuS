#ifndef POSESCLASS
#define POSESCLASS

#include "map/pose.h"
#include <vector>
#include <string>
#include "ros/ros.h"


class Poses
{
    public:
        Poses(){};

        void add(Pose);
        Pose get(std::string);

    private:
        std::vector<Pose> poses;

        bool isAlreadyHere(Pose);
};

#endif