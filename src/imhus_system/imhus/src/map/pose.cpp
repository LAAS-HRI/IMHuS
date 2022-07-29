#include "map/pose.h"

Pose::Pose()
{}

void Pose::setQuat(float _qx, float _qy, float _qz, float _qw)
{
    tf::Quaternion _q(_qx, _qy, _qz, _qw);
    q = _q;
    //update theta
    tf::Matrix3x3 m(q);
    double _r, _p, _theta; //we just need theta
    m.getRPY(_r, _p, _theta);
    theta = _theta;
}

void Pose::setTheta(float th)
{
    theta = th;
    q.setRPY(0, 0, theta);
}