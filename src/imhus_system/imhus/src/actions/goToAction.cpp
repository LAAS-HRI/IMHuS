#include "actions/goToAction.h"

using namespace actions;

GoToAction::GoToAction(Pose pose)
	:pose_(pose)
{
	socialspace_ = false;
	this->init();
}
GoToAction::GoToAction(SocialSpace* sp)
	:socialspace_ptr_(sp)
{
	socialspace_ = true;
	this->init();
}

void GoToAction::init()
{
	// Publishers
	pub_marker_rviz_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 100);

	// Init
	marker_rviz_.header.frame_id = "map";
	marker_rviz_.type = 1;
	marker_rviz_.pose.position.x = 0;
	marker_rviz_.pose.position.y = 0;
	marker_rviz_.pose.position.z = 0.25;
	marker_rviz_.scale.x = 0.3;
	marker_rviz_.scale.y = 0.3;
	marker_rviz_.scale.z = 0.5;
	marker_rviz_.color.r = 1;
	marker_rviz_.color.g = 1;
	marker_rviz_.color.b = 0;
	marker_rviz_.color.a = 0;

}

void GoToAction::execute()
{
	if(socialspace_) entity_->goTo(this->getPose(), this->socialspace_ptr_);	
	else entity_->goTo(this->getPose(), nullptr);
	this->updateMarkerPose(0, 0, 0);
}

void GoToAction::updateMarkerPose(float x, float y, float alpha)
{
	marker_rviz_.pose.position.x = x;
	marker_rviz_.pose.position.y = y;
	marker_rviz_.color.a = alpha;

	pub_marker_rviz_.publish(marker_rviz_);
}

Pose GoToAction::getPose()
{
	//check that the socialspace exists through the boolean, then request it a pose
	if(socialspace_)
	{ 
		return socialspace_ptr_->requestPose();
	}
	
	return pose_;
}