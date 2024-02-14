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

    success = true;
}


Planner::~Planner()
{
}


bool Planner::readParameters()
{
    node_handle.param("/planner/pose_topic", pose_topic, std::string("/ground_truth"));
    node_handle.param("/planner/cmd_topic", cmd_topic, std::string("/gpg_velocity_controller/cmd_vel"));
    node_handle.param("/planner/map_topic", map_topic, std::string("/grid_map_filter/filtered_map"));
    node_handle.param("/planner/base_to_hip_x", base_to_hip_x, 0.1881);
    node_handle.param("/planner/base_to_hip_y", base_to_hip_y, 0.12675);
    node_handle.param("/planner/robot_height", robot_height, 0.4);
    node_handle.param("/planner/t_stance", t_stance, 1.0);
    node_handle.param("/planner/k_velocity", k_velocity, 0.03);
    node_handle.param("/planner/gravity", gravity, 9.81);
    node_handle.param("/planner/threshold_traversability", threshold_traversability, 0.5);
    return true;
}


void Planner::poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Velocidade linear e angular da base do robô
    b_baseTwist_linear.setValue(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    b_baseTwist_angular.setValue(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

    // Transforma posição x,y das patas do base frame para world frame, usando matriz de transformação

    // translação
    tf::Vector3 translationWorldToBaseFootprint(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    // orientação
    tf::Quaternion baseOrientation;
    baseOrientation.setX(msg->pose.pose.orientation.x);
    baseOrientation.setY(msg->pose.pose.orientation.y);
    baseOrientation.setZ(msg->pose.pose.orientation.z);
    baseOrientation.setW(msg->pose.pose.orientation.w);

    // transformação que descreve o frame base_footprint em relação ao frame world
    w_transformFootPos_b.setRotation(baseOrientation);
    w_transformFootPos_b.setOrigin(translationWorldToBaseFootprint);

    b_pos.at(FL) = {base_to_hip_x, base_to_hip_y, 0.0};
    b_pos.at(FR) = {base_to_hip_x, -base_to_hip_y, 0.0};
    b_pos.at(RL) = {-base_to_hip_x, base_to_hip_y, 0.0};
    b_pos.at(RR) = {-base_to_hip_x, -base_to_hip_y, 0.0};
    
    for (int i=0; i < b_pos.size(); i++)
        w_pos.at(i) = w_transformFootPos_b * b_pos.at(i);
    
    // Atualiza posições dos foot markers atuais do robô 
    updateFootMarkers();

    // Atualiza próximas posições dos foot markers do robô
    updateNextFootMarkers();
}


void Planner::cmdCallback(const geometry_msgs::Twist& msg)
{
    b_cmdVel_linear.setValue(msg.linear.x, msg.linear.y, msg.linear.z);
    b_cmdVel_angular.setValue(msg.angular.x, msg.angular.y, msg.angular.z);
}


void Planner::mapCallback(const grid_map_msgs::GridMap& msg)
{
    grid_map::GridMap inputMap;
    grid_map::GridMapRosConverter::fromMessage(msg, inputMap, {"elevation","traversability"});

    // Obtém elevação e atravessabilidade das posições atuais e futuras das patas
    for (int i=0; i < elevation.size(); i++)
    {
        elevation.at(i) = inputMap.atPosition("elevation", grid_map::Position(w_pos.at(i).x(), w_pos.at(i).y()), grid_map::InterpolationMethods::INTER_LINEAR);
        if (std::isnan(elevation.at(i))) // insere elevação nula para as posições que não tem dado de elevação (NaN)
            elevation.at(i) = 0;
    }

    for (int i=0; i < traversability.size(); i++)
    {
        traversability.at(i) = inputMap.atPosition("traversability", grid_map::Position(w_pos.at(i).x(), w_pos.at(i).y()), grid_map::InterpolationMethods::INTER_LINEAR);
        if (std::isnan(traversability.at(i)))
            traversability.at(i) = threshold_traversability;
    }

    // Aplica threshold de atravessabilidade
    binary_t = {traversability.at(FL) >= threshold_traversability,
                traversability.at(FR) >= threshold_traversability,
                traversability.at(RL) >= threshold_traversability,
                traversability.at(RR) >= threshold_traversability};

    nextBinary_t = {traversability.at(nextFL) >= threshold_traversability,
                traversability.at(nextFR) >= threshold_traversability,
                traversability.at(nextRL) >= threshold_traversability,
                traversability.at(nextRR) >= threshold_traversability};
}


bool Planner::updateFootMarkers()
{
    // Coloca a posição de cada pé em um vector3d para ser publicado pela instância de FootPositionVisualizer
    Eigen::Vector3d posFL(w_pos.at(FL).x(), w_pos.at(FL).y(), elevation.at(FL));
    Eigen::Vector3d posFR(w_pos.at(FR).x(), w_pos.at(FR).y(), elevation.at(FR));
    Eigen::Vector3d posRL(w_pos.at(RL).x(), w_pos.at(RL).y(), elevation.at(RL));
    Eigen::Vector3d posRR(w_pos.at(RR).x(), w_pos.at(RR).y(), elevation.at(RR));

    footPositions = {posFL, posFR, posRL, posRR};

    footPosVisualizer.updateMarkers(footPositions);
    footPosVisualizer.changeColor(binary_t, 0);
    footPosVisualizer.publish();

    return true;
}


bool Planner::updateNextFootMarkers()
{
    // Calcula próximos passos por meio de Raibert Heuristic
    tf::Vector3 p_symmetry, p_centrifugal;

    std::array<tf::Vector3, 4> p_hip = {w_pos.at(FL), w_pos.at(FR), w_pos.at(RL), w_pos.at(RR)};

    p_symmetry = 0.5*t_stance*b_baseTwist_linear + k_velocity*(b_baseTwist_linear - b_cmdVel_linear);

    p_centrifugal = 0.5*sqrt(robot_height/gravity)*(b_baseTwist_linear.cross(b_cmdVel_angular));

    for (int i=0, j=4; i < p_hip.size(); i++, j++)
            w_pos.at(j) = p_hip.at(i) + p_symmetry + p_centrifugal;
    
    // Coloca a posição de cada pé em um vector3d para ser publicado pela instância de FootPositionVisualizer
    Eigen::Vector3d posNextFL(w_pos.at(nextFL).x(), w_pos.at(nextFL).y(), elevation.at(nextFL));
    Eigen::Vector3d posNextFR(w_pos.at(nextFR).x(), w_pos.at(nextFR).y(), elevation.at(nextFR));
    Eigen::Vector3d posNextRL(w_pos.at(nextRL).x(), w_pos.at(nextRL).y(), elevation.at(nextRL));
    Eigen::Vector3d posNextRR(w_pos.at(nextRR).x(), w_pos.at(nextRR).y(), elevation.at(nextRR));

    nextFootPositions = {posNextFL, posNextFR, posNextRL, posNextRR};

    nextFootPosVisualizer.updateMarkers(nextFootPositions);
    nextFootPosVisualizer.changeColor(nextBinary_t, 1);
    nextFootPosVisualizer.publish();

    return true;
}

}
