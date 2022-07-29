#include "imhus.h"

#define XML "goals.xml"

Imhus::Imhus(std::string root_id)
{
    agents::Group entity{root_id};

    ros::NodeHandle private_nh("~");
    private_nh.param(std::string("map_name"), map_name_, std::string("elevator"));
    ROS_INFO("Imhus: map_name=%s", map_name_.c_str());

    // INIT GOALS
    //read humans' choreography
    std::string goal_file = ros::package::getPath("imhus") + "/config/goals/" + map_name_ + "/" + XML;
    this->readXml(goal_file);

    //read robot's one
    // std::string robot_file_path = ros::package::getPath("imhus") + ;
    // this->readXml(robot_file_path);
   
    // // Check if file is corresponding with map_name
    // TiXmlHandle docHandle(doc_);
    // TiXmlElement *l_map = docHandle.FirstChild("map_name").ToElement();
    // std::string map_name_read = "";
    // if (NULL != l_map->Attribute("name"))
    //     map_name_read = l_map->Attribute("name");
    // ROS_INFO("Imhus: map_name_read=%s", map_name_read.c_str());
    // if (map_name_read != map_name_)
    //     ROS_ERROR("Imhus: Goals file mismatches the map_name");
}

void Imhus::execute(std::string id)
{
    std::vector<std::thread*> threads;
    for(auto s : this->scenarios.getScenarios())
    {
        if(s->id == id || s->id == id+"_robot")
        {
           threads.push_back(new std::thread(&scenario::Scenario::execute, s));         
        }
    }
    for(auto &thread: threads)
    {
        thread->join();
    }
    for(auto &thread: threads) 
    {
        delete thread; //not sure its needed
    }

    // bool found=false;
    // for(auto s : this->scenarios.getScenarios())
    // {
    //     if(s->id == id)
    //     {
    //         found=true;
    //         s->execute();
    //     }
    // }
    // if(!found) ROS_ERROR("Scenario %s has not been found!",id.c_str());
}
void Imhus::stop(std::string id)
{
    bool found=false;
    for(auto s : this->scenarios.getScenarios())
    {
        if(s->id == id)
        {
            found=true;
            s->stopping_request=true;
            ROS_WARN("The scenario %s will be terminated at next step.", id);
        }
    }
    if(!found) ROS_ERROR("Scenario %s has not been found!",id.c_str());
}



void Imhus::readXml(std::string file)
{
    doc_ = new TiXmlDocument(file);
    if (!doc_->LoadFile())
    {
        ROS_ERROR("Imhus: Failed to load %s. Error : %s", file.c_str(), doc_->ErrorDesc());
        return;
    }
    TiXmlHandle docHandle(doc_);

    // READ AGENTS
    TiXmlElement *agent = docHandle.FirstChild("agents").FirstChild().ToElement();
    while (agent)
    {
        std::string type = agent->Value(); // this line is bcs of a weird bug...
        if (type == "human")
        {
            if (NULL != agent->Attribute("id"))
            {
                this->createAgent(agent->Attribute("id"), "human");
            }
            else
            {
                ROS_ERROR("You forgot to specify id of a human in the xml, cannot create it.");
            }
        }
        if (type == "robot")
        {
            if (NULL != agent->Attribute("id"))
            {
                this->createAgent(agent->Attribute("id"), "robot");
            }
            else
            {
                ROS_ERROR("You forgot to specify id of a robot in the xml, cannot create it.");
            }
        }
        agent = agent->NextSiblingElement();
    }

    // READ GROUPS
    TiXmlElement *group_xml = docHandle.FirstChild("groups").FirstChild().ToElement();
    while (group_xml)
    {
        if (NULL != group_xml->Attribute("id"))
        {
            // CAREFUL need to declare global pointers otherwise they are delete when fct is finished
            agents::Group *group_ptr = new agents::Group{group_xml->Attribute("id")};
            if (NULL != group_xml->Attribute("members"))
            {
                std::vector<std::string> members = this->string_split(group_xml->Attribute("members"));
                for (auto memberID : members)
                {
                    group_ptr->add(this->entity.getEntityPtr(memberID));
                }
            }
            this->entity.add(group_ptr);
        }
        else
        {
            ROS_ERROR("A group in xml is missing id. Cannot create it.");
        }

        group_xml = group_xml->NextSiblingElement();
    }

    // READ POSES
    //normal pose
    TiXmlElement *pose_xml = docHandle.FirstChild("poses").FirstChild("pose").FirstChild().ToElement();
    while (pose_xml)
    {
        Pose pose;
        if (NULL != pose_xml->Attribute("id"))
            pose.setID(pose_xml->Attribute("id"));
        if (NULL != pose_xml->Attribute("x"))
            pose.setX(std::stof(pose_xml->Attribute("x")));
        if (NULL != pose_xml->Attribute("y"))
            pose.setY(std::stof(pose_xml->Attribute("y")));
        if (NULL != pose_xml->Attribute("theta"))
            pose.setTheta(std::stof(pose_xml->Attribute("theta")));
        this->map.poses.add(pose);

        pose_xml = pose_xml->NextSiblingElement();
    }
    //socialpose
    TiXmlElement *sc_space_xml = docHandle.FirstChild("poses").FirstChild("social_space").FirstChild().ToElement();
    while(sc_space_xml)
    {
        if (NULL != sc_space_xml->Attribute("id") && NULL != sc_space_xml->Attribute("x") && NULL != sc_space_xml->Attribute("y"))
        {
            SocialSpace *sc_space = new SocialSpace{sc_space_xml->Attribute("id"), &entity};

            float x=std::stof(sc_space_xml->Attribute("x"));
            float y=std::stof(sc_space_xml->Attribute("y"));

            sc_space->setX(x);
            sc_space->setY(y);
           
            TiXmlElement *sc_pose_xml = sc_space_xml->FirstChildElement();
            while(sc_pose_xml)
            {
                SocialPose sc_pose;

                if (NULL != sc_pose_xml->Attribute("id")) 
                    sc_pose.setID(sc_pose_xml->Attribute("id"));
                if (NULL != sc_pose_xml->Attribute("theta")) 
                    sc_pose.setTheta(std::stof(sc_pose_xml->Attribute("theta")));
                if(NULL != sc_pose_xml->Attribute("relativeX") && NULL != sc_pose_xml->Attribute("relativeY"))
                {
                    sc_pose.setX(x + std::stof(sc_pose_xml->Attribute("relativeX")));
                    sc_pose.setY(y + std::stof(sc_pose_xml->Attribute("relativeY")));
                    sc_space->add(sc_pose);
                }
                else
                {
                    ROS_ERROR("A Pose inside the SocialSpace %s is missing relativeX and/or relativeY attribute", sc_space_xml->Attribute("id"));
                }

                sc_pose_xml = sc_pose_xml->NextSiblingElement();
            }

            this->map.social_spaces.add(sc_space);
        }
        else
        {
            ROS_ERROR("A social space is missing the attribute : id and/or x and/or y ! \n It cannot be created.");
        } 
        sc_space_xml = sc_space_xml->NextSiblingElement();
    }


    /////////////////  GOALS  ///////////////////////////////
    // READ COMPOUND TASK (short set of actions)
    TiXmlElement *ct_xml = docHandle.FirstChild("goals").FirstChild("compound_tasks").FirstChild().ToElement();

    while (ct_xml)
    {
        std::string value = ct_xml->Value();
        actions::CompoundTask *ct = new actions::CompoundTask{value};
        TiXmlElement *action_xml = ct_xml->FirstChildElement();

        while (action_xml)
        {
            std::string action_type = action_xml->Value();
            if (action_type == "goto_action")
            {
                std::string pose_id;

                if (NULL != action_xml->Attribute("poseID"))
                {
                    if(NULL != action_xml->Attribute("relativeX") || NULL != action_xml->Attribute("relativeY") || NULL != action_xml->Attribute("relativeTheta"))
                    { //Relative pose
                        pose_id = action_xml->Attribute("poseID");
                        Pose target = this->map.poses.get(pose_id);

                        if(NULL != action_xml->Attribute("relativeX")) target.setX(target.getX() + std::stof(action_xml->Attribute("relativeX")));
                        if(NULL != action_xml->Attribute("relativeY")) target.setY(target.getY() + std::stof(action_xml->Attribute("relativeY")));
                        if(NULL != action_xml->Attribute("relativeTheta")) target.setTheta(target.getTheta() + std::stof(action_xml->Attribute("relativeTheta")));

                        actions::GoToAction *new_action = new actions::GoToAction{target};
                        ct->add(new_action);
                    }
                    else
                    {   //Predefined pose       
                        pose_id = action_xml->Attribute("poseID");
                        Pose target = this->map.poses.get(pose_id);
                        actions::GoToAction *new_action = new actions::GoToAction{target};
                        ct->add(new_action);
                    }
                }
                else if(NULL != action_xml->Attribute("socialSpaceID"))
                {
                    if(std::get<0>(map.social_spaces.get(action_xml->Attribute("socialSpaceID"))))
                    {   //the pose will be assigned by a socialspace later on, so its only purpose here is to carry the ID
                        actions::GoToAction *new_action = new actions::GoToAction{std::get<1>(map.social_spaces.get(action_xml->Attribute("socialSpaceID")))};
                        ct->add(new_action);
                    }
                    else ROS_ERROR("SocialSpace %s does not exist ! Cannot create gotoaction.", action_xml->Attribute("socialSpaceID"));   
                }
                else
                {
                    Pose new_pose;
                    if(NULL != action_xml->Attribute("x")) new_pose.setX(std::stof(action_xml->Attribute("x")));
                    if(NULL != action_xml->Attribute("y")) new_pose.setY(std::stof(action_xml->Attribute("y")));
                    if(NULL != action_xml->Attribute("theta")) new_pose.setTheta(std::stof(action_xml->Attribute("theta")));
                    actions::GoToAction *new_action = new actions::GoToAction{new_pose};
                    ct->add(new_action);
                }
            }
            else if (action_type == "wait_action")
            {
                if (NULL != action_xml->Attribute("duration"))
                {
                    actions::WaitAction *new_action = new actions::WaitAction{std::stof(action_xml->Attribute("duration"))};
                    ct->add(new_action);
                }
                else if (NULL != action_xml->Attribute("event") && NULL != action_xml->Attribute("topic"))
                {
                    actions::WaitAction *new_action = new actions::WaitAction{action_xml->Attribute("topic"), action_xml->Attribute("event")};
                    ct->add(new_action);
                }
                else
                {
                    ROS_ERROR("A wait action is ill declared with empty attributes. Skipping it");
                }
            }
            else if (action_type == "publish_action")
            {
                if(NULL != action_xml->Attribute("topic") && NULL != action_xml->Attribute("msg"))
                {
                    actions::PublishAction *new_action = new actions::PublishAction{action_xml->Attribute("topic"), action_xml->Attribute("msg")};
                    ct->add(new_action);
                }else{
                    ROS_ERROR("A publish action is missing either topic attribute or msg attribute.");
                } 
            }
            else if(action_type == "async_action_req")
            {
                if(NULL != action_xml->Attribute("taskID"))
                {
                    actions::AsyncActionReq *new_action = new actions::AsyncActionReq{action_xml->Attribute("taskID")};
                    ct->add(new_action);
                }else{
                    ROS_ERROR("A async_action_req is missing attribute.");
                } 
            }
            else
            {
                ROS_ERROR("Action type : %s, is not read by InHuS, this action will be ignored.", action_type.c_str());
            }
            action_xml = action_xml->NextSiblingElement();
        }
        // register this compoundtask and read following one
        this->tasks.add(ct);
        ct_xml = ct_xml->NextSiblingElement();
    }

    //READ SCENARIOS
    TiXmlElement *scenario_xml = docHandle.FirstChild("goals").FirstChild("scenarios").FirstChild().ToElement();

    int make_unique=1001; //this nb is to avoid redundancy when we ask a step as a single action
    while (scenario_xml)
    {
        scenario::Scenario *scenario = new scenario::Scenario;
        scenario->id = scenario_xml->Value();
        TiXmlElement *step_xml = scenario_xml->FirstChildElement();

        while (step_xml)
        {
            std::string step_id = step_xml->Value();
            scenario::Step *step = new scenario::Step{step_id};
            TiXmlElement *action_xml = step_xml->FirstChildElement();

            while(action_xml)
            {
                std::string action_name = action_xml->Value();
                if (NULL != action_xml->Attribute("entityID"))
                {
                    if(NULL != action_xml->Attribute("taskID") && action_name!="async_action")
                    {
                        agents::Entity* pE=this->entity.getEntityPtr(action_xml->Attribute("entityID"));
                        actions::CompoundTask* pT=this->tasks.getTask(action_xml->Attribute("taskID"));
                        
                        step->add(pE, pT);
                    }
                    else 
                    {
                        // create a ct on the fly with random name for single actions
                        std::string name = step_id + std::to_string(make_unique); make_unique++;
                        actions::CompoundTask *ct = new actions::CompoundTask{name};
                        if (action_name == "goto_action")
                        {
                            std::string pose_id;
                            Pose target;

                            if (NULL != action_xml->Attribute("poseID"))
                            {
                                if(NULL != action_xml->Attribute("relativeX") || NULL != action_xml->Attribute("relativeY") || NULL != action_xml->Attribute("relativeTheta"))
                                { //Relative pose
                                    pose_id = action_xml->Attribute("poseID");
                                    Pose target = this->map.poses.get(pose_id);

                                    if(NULL != action_xml->Attribute("relativeX")) target.setX(target.getX() + std::stof(action_xml->Attribute("relativeX")));
                                    if(NULL != action_xml->Attribute("relativeY")) target.setY(target.getY() + std::stof(action_xml->Attribute("relativeY")));
                                    if(NULL != action_xml->Attribute("relativeTheta")) target.setTheta(target.getTheta() + std::stof(action_xml->Attribute("relativeTheta")));

                                    actions::GoToAction *new_action = new actions::GoToAction{target};
                                    ct->add(new_action);
                                }
                                else
                                {   //Predefined pose       
                                    pose_id = action_xml->Attribute("poseID");
                                    Pose target = this->map.poses.get(pose_id);
                                    actions::GoToAction *new_action = new actions::GoToAction{target};
                                    ct->add(new_action);
                                }
                            }
                            else if(NULL != action_xml->Attribute("socialSpaceID"))
                            {
                                if(std::get<0>(map.social_spaces.get(action_xml->Attribute("socialSpaceID"))))
                                {   //the pose will be assigned by a socialspace later on, so its only purpose here is to carry the ID
                                    actions::GoToAction *new_action = new actions::GoToAction{std::get<1>(map.social_spaces.get(action_xml->Attribute("socialSpaceID")))};
                                    ct->add(new_action);
                                }
                                else ROS_ERROR("SocialSpace %s does not exist ! Cannot create gotoaction.", action_xml->Attribute("socialSpaceID"));   
                            }
                            else
                            {
                                Pose new_pose;
                                if(NULL != action_xml->Attribute("x")) new_pose.setX(std::stof(action_xml->Attribute("x")));
                                if(NULL != action_xml->Attribute("y")) new_pose.setY(std::stof(action_xml->Attribute("y")));
                                if(NULL != action_xml->Attribute("theta")) new_pose.setTheta(std::stof(action_xml->Attribute("theta")));
                                actions::GoToAction *new_action = new actions::GoToAction{new_pose};
                                ct->add(new_action);
                            }
                            this->tasks.add(ct);
                            agents::Entity* pE=this->entity.getEntityPtr(action_xml->Attribute("entityID"));
                            
                            step->add(pE, ct);
                        }
                        else if (action_name == "wait_action")
                        {
                            if (NULL != action_xml->Attribute("duration"))
                            {
                                actions::WaitAction *new_action = new actions::WaitAction{std::stof(action_xml->Attribute("duration"))};
                                ct->add(new_action);
                                this->tasks.add(ct);
                                agents::Entity* pE=this->entity.getEntityPtr(action_xml->Attribute("entityID"));
                                
                                // scenario::Step *step = new scenario::Step{pE, ct};
                                // scenario->steps.add(step);
                                step->add(pE, ct);
                            }
                            else if (NULL != action_xml->Attribute("event") && NULL != action_xml->Attribute("topic"))
                            {
                                actions::WaitAction *new_action = new actions::WaitAction{action_xml->Attribute("topic"), action_xml->Attribute("event")};
                                ct->add(new_action);
                                this->tasks.add(ct);
                                agents::Entity* pE=this->entity.getEntityPtr(action_xml->Attribute("entityID"));
                                
                                step->add(pE, ct);
                            }
                            else
                            {
                                ROS_ERROR("A wait action is ill declared with empty attributes. Skipping it");
                            }
                        }
                        else
                        {
                            //If here, step maybe without taskID and or not a single action as value (e.g.: <goto_action "".../>)
                            ROS_ERROR("Step : %s is ill defined, will be ignored. Could not identify the corresponding action.", step_id.c_str());
                        }
                    }
                }
                //here because async_action is without entityID
                else if(NULL != action_xml->Attribute("taskID"))// && NULL != action_xml->Attribute("trigger"))
                {
                    if(action_name == "async_action_res")
                    {
                        // create a ct on the fly with random name for single actions
                        std::string name = step_id + std::to_string(make_unique); make_unique++;
                        actions::CompoundTask *ct = new actions::CompoundTask{name};

                        actions::AsyncActionRes *async = new actions::AsyncActionRes{ 
                            this->tasks.getTask(action_xml->Attribute("taskID")),
                            &entity
                        };
                        ct->add(async);
                        step->add(nullptr, ct); //no need to assign it to an agent
                    }
                }
                else
                {
                    ROS_ERROR("Step %s in scenario %s is ill-defined, specify attributes : entityID", step_id.c_str(), scenario->id.c_str());
                }
                action_xml = action_xml->NextSiblingElement();
            }
            scenario->steps.add(step);
            step_xml = step_xml->NextSiblingElement();
        }
        this->scenarios.add(scenario);
        scenario_xml = scenario_xml->NextSiblingElement();
    }

    ROS_INFO("Success parsing %s.", file.c_str());
}

void Imhus::createAgent(std::string id, std::string type)
{
    if (type == "human")
    {
        agents::Human *human_ptr = new agents::Human{id};
        this->entity.add(human_ptr);
    }
    else if (type == "robot")
    {
        agents::Robot *robot_ptr = new agents::Robot{id};
        this->entity.add(robot_ptr);
    }
}

std::vector<std::string> Imhus::string_split(const std::string &str)
{
    std::vector<std::string> result;
    std::istringstream iss(str);
    for (std::string s; iss >> s;)
        result.push_back(s);
    return result;
}

void Imhus::update()
{
    ros::spinOnce();
    for(auto agent : this->entity.getMembers())
        agent->update();
}