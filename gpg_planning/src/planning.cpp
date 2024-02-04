#include "gpg_planning/Planning.hpp"

namespace footstep_planning {

Planner::Planner(ros::NodeHandle& nodeHandle, bool& success)
        : node_handle(nodeHandle),
          footPosVisualizer(nodeHandle, "/foot_positions", 0), // Publisher /foot_positions para gerar markers
          nextFootPosVisualizer(nodeHandle, "/next_foot_positions", 1) // Publisher /next_foot_positions para gerar markers
{
    // Parâmetros de subscriber e publisher
    if (!readParameters())
    {
        success = false;
        return;
    };

    // Subscriber /ground_truth
    pose_subscriber = node_handle.subscribe(pose_topic, 100, &Planner::poseCallback, this);

    // Subscriber /cmd_vel
    cmd_subscriber = node_handle.subscribe(cmd_topic, 100, &Planner::cmdCallback, this);

    // Subscriber grid_map, acessando layer de elevation e traversability
    map_subscriber = node_handle.subscribe(map_topic, 1, &Planner::mapCallback, this);

    // Subscriber foot_positions para calcular, usando Raibert Heuristic, e publicar os next foot markers do robô
    //footFL_subscriber = node_handle.subscribe(foot_topic + "_FL", 1, &Planner::footCallback, this);
    //footFR_subscriber = node_handle.subscribe(foot_topic + "_FR", 1, &Planner::footCallback, this);
    //footRL_subscriber = node_handle.subscribe(foot_topic + "_RL", 1, &Planner::footCallback, this);
    //footRR_subscriber = node_handle.subscribe(foot_topic + "_RR", 1, &Planner::footCallback, this);

    success = true;
}


Planner::~Planner()
{
}


bool Planner::readParameters()
{
    node_handle.param("pose_topic", pose_topic, std::string("/ground_truth"));
    node_handle.param("cmd_topic", cmd_topic, std::string("/gpg_velocity_controller/cmd_vel"));
    node_handle.param("map_topic", map_topic, std::string("/grid_map_filter/filtered_map"));
    node_handle.param("foot_topic", foot_topic, std::string("/foot_positions"));
    node_handle.param("next_foot_topic", next_foot_topic, std::string("/next_foot_positions"));
    node_handle.param("base_to_hip_x", base_to_hip_x, 0.1881);
    node_handle.param("base_to_hip_y", base_to_hip_y, 0.04675);
    node_handle.param("robot_height", robot_height, 0.4);
    node_handle.param("t_stance", t_stance, 2.0);
    node_handle.param("k_velocity", k_velocity, 0.03);
    node_handle.param("gravity", gravity, 9.81);
    return true;
}

/*
poseCallback
    Pegar posição xy da base em relação ao world
    Pegar velocidade da base em relação ao world
    Calcular posição xy dos pés em relação ao world

cmdCallback
    Pegar velocidade do comando em relação ao world
    
mapCallback
    Pegar elevação e traversability da posição xy dos pés

updateFootMarkers
    Publicar posição dos pés (x,y,elevação) para mostrar no RViz

footCallback
    Calcular Raibert
    Publicar próxima posição de cada pé para mostrar no RViz

*/


void Planner::poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Transforma velocidade do base frame para world frame, usando matriz de rotação

    // orientação
    tf::Quaternion baseOrientation;
    baseOrientation.setX(msg->pose.pose.orientation.x);
    baseOrientation.setY(msg->pose.pose.orientation.y);
    baseOrientation.setZ(msg->pose.pose.orientation.z);
    baseOrientation.setW(msg->pose.pose.orientation.w);

    // translação
    tf::Vector3 translationTwist(0.0, 0.0, 0.0);

    // rotação que descreve a orientação do frame base_footprint em relação ao frame world
    w_transformTwist_b.setRotation(baseOrientation);
    w_transformTwist_b.setOrigin(translationTwist);

    b_baseTwist_linear.setValue(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    b_baseTwist_angular.setValue(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

    w_baseTwist_linear = w_transformTwist_b * b_baseTwist_linear;
    w_baseTwist_angular = w_transformTwist_b * b_baseTwist_angular;  

    w_basePos.setValue(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);


    // Transforma posição x,y dos pés do base frame para world frame, usando matriz de transformação

    // translação
    tf::Vector3 translationWorldToBaseFootprint(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    // transformação que descreve o frame base_footprint em relação ao frame world
    w_transformFootPos_b.setRotation(baseOrientation);
    w_transformFootPos_b.setOrigin(translationWorldToBaseFootprint);

    tf::Vector3 b_posFL(base_to_hip_x, base_to_hip_y, 0.0);
    tf::Vector3 b_posFR(base_to_hip_x, -base_to_hip_y, 0.0);
    tf::Vector3 b_posRL(-base_to_hip_x, base_to_hip_y, 0.0);
    tf::Vector3 b_posRR(-base_to_hip_x, -base_to_hip_y, 0.0);
    
    w_posFL = w_transformFootPos_b * b_posFL;
    w_posFR = w_transformFootPos_b * b_posFR;
    w_posRL = w_transformFootPos_b * b_posRL;
    w_posRR = w_transformFootPos_b * b_posRR;

    //ROS_INFO("w_posFL = (%f, %f, %f)\n", w_posFL.x(), w_posFL.y(), w_posFL.z());
}


void Planner::cmdCallback(const geometry_msgs::Twist& msg)
{
    b_cmdVel_linear.setValue(msg.linear.x, msg.linear.y, msg.linear.z);
    b_cmdVel_angular.setValue(msg.angular.x, msg.angular.y, msg.angular.z);

    w_cmdVel_linear = w_transformTwist_b * b_cmdVel_linear;
    w_cmdVel_angular = w_transformTwist_b * b_cmdVel_angular; 

    ROS_INFO("cmd (base): (%f, %f, %f)\ncmd (world): (%f, %f, %f)\n", 
        b_cmdVel_linear.x(), b_cmdVel_linear.y(), b_cmdVel_linear.z(), 
        w_cmdVel_linear.x(), w_cmdVel_linear.y(), w_cmdVel_linear.z());
}


void Planner::mapCallback(const grid_map_msgs::GridMap& msg)
{
    grid_map::GridMap inputMap;
    grid_map::GridMapRosConverter::fromMessage(msg, inputMap, {"elevation","traversability"});

    elevation_FL = inputMap.atPosition("elevation", grid_map::Position(w_posFL.x(), w_posFL.y()), grid_map::InterpolationMethods::INTER_LINEAR);
    elevation_FR = inputMap.atPosition("elevation", grid_map::Position(w_posFR.x(), w_posFR.y()), grid_map::InterpolationMethods::INTER_LINEAR);
    elevation_RL = inputMap.atPosition("elevation", grid_map::Position(w_posRL.x(), w_posRL.y()), grid_map::InterpolationMethods::INTER_LINEAR);
    elevation_RR = inputMap.atPosition("elevation", grid_map::Position(w_posRR.x(), w_posRR.y()), grid_map::InterpolationMethods::INTER_LINEAR);

    traversability_FL = inputMap.atPosition("traversability", grid_map::Position(w_posFL.x(), w_posFL.y()), grid_map::InterpolationMethods::INTER_LINEAR);
    traversability_FR = inputMap.atPosition("traversability", grid_map::Position(w_posFR.x(), w_posFR.y()), grid_map::InterpolationMethods::INTER_LINEAR);
    traversability_RL = inputMap.atPosition("traversability", grid_map::Position(w_posRL.x(), w_posRL.y()), grid_map::InterpolationMethods::INTER_LINEAR);
    traversability_RR = inputMap.atPosition("traversability", grid_map::Position(w_posRR.x(), w_posRR.y()), grid_map::InterpolationMethods::INTER_LINEAR);

    ROS_INFO("Elevation: %f\n", w_posFL.z());

    ROS_INFO("Traversability: %f\n", traversability_FL);

    // Publisher da posição dos foot markers atuais do robô 
    updateFootMarkers();

    // Testar no SuperPC: rostopic hz /grid_map_filter/filtered_map , /elevation_mapping/elevation_map
}


bool Planner::updateFootMarkers()
{
    // Coloca a posição de cada pé em um vector3d para ser publicado pela instância de FootPositionVisualizer
    Eigen::Vector3d FL(w_posFL.x(), w_posFL.y(), elevation_FL);
    Eigen::Vector3d FR(w_posFR.x(), w_posFR.y(), elevation_FR);
    Eigen::Vector3d RL(w_posRL.x(), w_posRL.y(), elevation_RL);
    Eigen::Vector3d RR(w_posRR.x(), w_posRR.y(), elevation_RR);

    footPositions.push_back(FL);
    footPositions.push_back(FR);
    footPositions.push_back(RL);
    footPositions.push_back(RR);

    footPosVisualizer.updateMarkers(footPositions);
    footPosVisualizer.publish();

    ROS_INFO("Step markers atualizados!");

    return true;
}


void Planner::footCallback(const visualization_msgs::Marker& msg)
{
    // Calcular próximos passos por meio de Raibert Heuristic
    tf::Vector3 p_hip, p_symmetry, p_centrifugal, r_cmd;

    p_hip.setValue(msg.pose.position.x, msg.pose.position.y, 0.0);

    // FL = 0 | FR = 1 | RL = 2 | RR = 3
    switch(msg.id) 
    {
        case 0:
            p_hip.setZ(w_posFL.z());
            break;
        case 1:
            p_hip.setZ(w_posFR.z());
            break;
        case 2:
            p_hip.setZ(w_posRL.z());
            break;
        case 3:
            p_hip.setZ(w_posRR.z());
            break;
        default:
            ROS_INFO("ID Marker errado! Raibert Heuristic vai retornar a posicao (0,0,0)");
    }

    p_symmetry = (0.5*t_stance*w_baseTwist_linear + k_velocity*(w_baseTwist_linear - w_cmdVel_linear));

    p_centrifugal = 0.5*sqrt(robot_height/gravity)*(w_baseTwist_linear.cross(w_baseTwist_angular));

    r_cmd = p_hip + p_symmetry + p_centrifugal;
    
    // FL = 0 | FR = 1 | RL = 2 | RR = 3
    switch(msg.id) 
    {
        case 0:
            r_cmd_FL.setValue(r_cmd.x(), r_cmd.y(), r_cmd.z());
            break;
        case 1:
            r_cmd_FR.setValue(r_cmd.x(), r_cmd.y(), r_cmd.z());
            break;
        case 2:
            r_cmd_RL.setValue(r_cmd.x(), r_cmd.y(), r_cmd.z());
            break;
        case 3:
            r_cmd_RR.setValue(r_cmd.x(), r_cmd.y(), r_cmd.z());
            break;
        default:
            r_cmd_FL.setValue(0,0,0);
            r_cmd_FR.setValue(0,0,0);
            r_cmd_RL.setValue(0,0,0);
            r_cmd_RR.setValue(0,0,0);
    }

    if (msg.id == 3)
    {
        updateNextFootMarkers();
    }
}

bool Planner::updateNextFootMarkers()
{
    // Coloca a posição de cada pé em um vector3d para ser publicado pela instância de FootPositionVisualizer
    /*Eigen::Vector3d nextFL(r_cmd_FL.x(), r_cmd_FL.y(), elevation_nextFL);
    Eigen::Vector3d nextFR(r_cmd_FR.x(), r_cmd_FR.y(), elevation_nextFR);
    Eigen::Vector3d nextRL(r_cmd_RL.x(), r_cmd_RL.y(), elevation_nextRL);
    Eigen::Vector3d nextRR(r_cmd_RR.x(), r_cmd_RR.y(), elevation_nextRR);*/
    Eigen::Vector3d nextFL(r_cmd_FL.x(), r_cmd_FL.y(), r_cmd_FL.z());
    Eigen::Vector3d nextFR(r_cmd_FR.x(), r_cmd_FR.y(), r_cmd_FR.z());
    Eigen::Vector3d nextRL(r_cmd_RL.x(), r_cmd_RL.y(), r_cmd_RL.z());
    Eigen::Vector3d nextRR(r_cmd_RR.x(), r_cmd_RR.y(), r_cmd_RR.z());

    nextFootPositions.push_back(nextFL);
    nextFootPositions.push_back(nextFR);
    nextFootPositions.push_back(nextRL);
    nextFootPositions.push_back(nextRR);

    nextFootPosVisualizer.updateMarkers(nextFootPositions);
    nextFootPosVisualizer.publish();

    ROS_INFO("Next steps markers atualizados!");

    return true;
}

}

/*
    // Base to hip position
    Eigen::Vector3d FL(stateWorld.pos_x+base_to_hip_x, stateWorld.pos_y+base_to_hip_y, 0.0);
    Eigen::Vector3d FR(stateWorld.pos_x+base_to_hip_x, stateWorld.pos_y-base_to_hip_y, 0.0);
    Eigen::Vector3d RL(stateWorld.pos_x-base_to_hip_x, stateWorld.pos_y+base_to_hip_y, 0.0);
    Eigen::Vector3d RR(stateWorld.pos_x-base_to_hip_x, stateWorld.pos_y-base_to_hip_y, 0.0);
*/
