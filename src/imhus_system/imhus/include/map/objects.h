#include "map/pose.h"
#include "map/object.h"
#include <vector>
#include <string>
#include "ros/ros.h"


class Objects
{
    public:
        Objects(){};

        void add(Object);
        Object get(std::string);

    private:
        std::vector<Object> objects;

        bool isAlreadyHere(Object);
};