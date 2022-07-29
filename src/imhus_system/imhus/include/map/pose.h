#ifndef POSE_
#define POSE_

#include <string>
#include "tf/tf.h"

class Pose
{
    public:
        Pose();
        Pose(std::string _id){id=_id;};

        void setID(std::string _id){id=_id;};
        void setX(float _x){x=_x;};
        void setY(float _y){y=_y;};
        void setTheta(float);
        void setXYTheta(float _x, float _y, float _t){x=_x, y=_y, theta=_t;};
        void setQuat(float _qx, float _qy, float _qz, float _qw);

        std::string getID(){return id;};
        float getX(){return x;};
        float getY(){return y;};
        float getTheta(){return theta;};
        tf::Quaternion getQuat(){return q;};

    private:
        std::string id;
        float x, y, theta;
        tf::Quaternion q;
};

#endif