#include <bt_executer/skills/store_actual_position_skill.hpp>
#include <std_msgs/msg/string.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <control_msgs/msg/interface_value.hpp>
#include <chrono>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose.hpp>

using DynamicJointState = control_msgs::msg::DynamicJointState;
using InterfaceValue = control_msgs::msg::InterfaceValue; //usato per ricostruire il messaggio da stampare in output

StoreActualPositionNode::StoreActualPositionNode(const std::string& name, 
						 const BT::NodeConfig& conf,
                        			 const BT::RosNodeParams& params)
    : RosTopicSubNode<DynamicJointState>(name, conf, params) 
{
   auto param_ns = getInput<std::string>("topic_name");
   ns_ = "/bt_executer/" + param_ns.value();
   
   //pubblico su un topic chiamato "/previous_position" per poi poter calcolare il delta movimento)
   joint_position_publisher_ = node_.lock()->create_publisher<InterfaceValue>("/previous_position", 10);

   cartesian_position_publisher_ = node_.lock()->create_publisher<geometry_msgs::msg::Pose>("/cartesian_position", 10);

   tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_.lock()->get_clock());
   tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


BT::NodeStatus StoreActualPositionNode::onTick(const std::shared_ptr<DynamicJointState>& last_msg)
{
    auto node_shared = node_.lock();
    if (!node_shared) {
        RCLCPP_ERROR(rclcpp::get_logger("StoreActualPositionNode"), "Nodo non più disponibile.");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_shared->get_logger(), "Messaggio ricevuto, elaborazione in corso...");

    //RCLCPP_INFO(node_.lock()->get_logger(), "QUI");

    //RCLCPP_INFO(node_.lock()->get_logger(), "Verifica last_msg: %p", last_msg.get());

    if(last_msg)
    {
        //RCLCPP_INFO(node_.lock()->get_logger(), "QUI QUI");

        // Vettore di coppie per memorizzare i nomi e le posizioni dei giunti KUKA
        std::vector<std::pair<std::string, double>> kuka_joint_positions;
        
      // Itera sui giunti e salva le posizioni relative a quelli KUKA
      for (size_t i = 0; i < last_msg->joint_names.size(); ++i) 
        {
            if (last_msg->joint_names[i].find("kuka") != std::string::npos || last_msg->joint_names[i].find("structure_joint_1") != std::string::npos) //std::string::npos indica che la sottostringa 'kuka' o 'structure_joint_1' non è stata trovata.
            {
                double position = last_msg->interface_values[i].values[0]; // 0 è la posizione
                kuka_joint_positions.emplace_back(last_msg->joint_names[i], position);            
            }
               
        }

        // Se abbiamo trovato posizioni KUKA, ritorna SUCCESS, altrimenti FAILURE
        if (!kuka_joint_positions.empty()) 
        {
            // Crea il messaggio InterfaceValue
            InterfaceValue kuka_joint_msg;
            kuka_joint_msg.interface_names.resize(7);
            kuka_joint_msg.values.resize(7);

            //ricompongo i messaggi con i giunti ordinati
            kuka_joint_msg.interface_names[0] = kuka_joint_positions[0].first;
            kuka_joint_msg.values[0] = kuka_joint_positions[0].second;

            kuka_joint_msg.interface_names[1] = kuka_joint_positions[2].first;
            kuka_joint_msg.values[1] = kuka_joint_positions[2].second;

            kuka_joint_msg.interface_names[2] = kuka_joint_positions[5].first;
            kuka_joint_msg.values[2] = kuka_joint_positions[5].second;

            kuka_joint_msg.interface_names[3] = kuka_joint_positions[6].first;
            kuka_joint_msg.values[3] = kuka_joint_positions[6].second;

            kuka_joint_msg.interface_names[4] = kuka_joint_positions[1].first;
            kuka_joint_msg.values[4] = kuka_joint_positions[1].second;

            kuka_joint_msg.interface_names[5] = kuka_joint_positions[4].first;
            kuka_joint_msg.values[5] = kuka_joint_positions[4].second;

            kuka_joint_msg.interface_names[6] = kuka_joint_positions[3].first;
            kuka_joint_msg.values[6] = kuka_joint_positions[3].second;
            
            //pubblico il messaggio
            joint_position_publisher_->publish(kuka_joint_msg);


            // costruisco un'unica stringa per il messaggio
            std::string full_msg = "Stato attuale dei giunti KUKA:\n";
            for (size_t i = 0; i < kuka_joint_msg.interface_names.size(); ++i)
            {
                full_msg += "Nome giunto: " + kuka_joint_msg.interface_names[i] +
                            ", Posizione: " + std::to_string(kuka_joint_msg.values[i]) + "\n";
            }

            // Stampa tutto il messaggio in una sola volta, più leggibile su terminale
            //RCLCPP_INFO(node_.lock()->get_logger(), "%s", full_msg.c_str());

            // Stampa il messaggio in output con più chiamate a RCLCPP_INFO
            /*RCLCPP_INFO(node_.lock()->get_logger(), "[%s] Stato attuale dei giunti KUKA:", name().c_str());
            for (size_t i = 0; i < kuka_joint_msg.interface_names.size(); ++i)
            {
                RCLCPP_INFO(node_.lock()->get_logger(), "Nome giunto: %s, Posizione: %f",
                            kuka_joint_msg.interface_names[i].c_str(),
                            kuka_joint_msg.values[i]);
            }*/

            // Acquisizione della trasformazione tra 'world' e 'kuka_sensor'
            geometry_msgs::msg::TransformStamped transformStamped;
            try
            {
                transformStamped = tf_buffer_->lookupTransform("world", "kuka_sensor", tf2::TimePointZero, tf2::durationFromSec(1.0));
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(node_.lock()->get_logger(), "Could not transform 'world' to 'kuka_sensor': %s", ex.what());
                return BT::NodeStatus::FAILURE;
            }

            // Estrarre la posizione e l'orientamento dal messaggio TransformStamped
            geometry_msgs::msg::Pose pose_msg;
            pose_msg.position.x = transformStamped.transform.translation.x;
            pose_msg.position.y = transformStamped.transform.translation.y;
            pose_msg.position.z = transformStamped.transform.translation.z;
            pose_msg.orientation = transformStamped.transform.rotation;

            // Pubblicare i dati cartesiani su /cartesian_position
            cartesian_position_publisher_->publish(pose_msg);

            // Logging della posizione cartesiana
            RCLCPP_INFO(node_.lock()->get_logger(), "Pubblicata posizione cartesiana: [x: %f, y: %f, z: %f]",
                        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);

            return BT::NodeStatus::SUCCESS;
        }
    }
    //RCLCPP_INFO(node_.lock()->get_logger(), "QUI QUI QUI");

    return BT::NodeStatus::FAILURE;
}
   
// Plugin registration.
// The class StoreActualPositionNode will self register with name  "StoreActualPositionSkill".
CreateRosNodePlugin(StoreActualPositionNode, "StoreActualPositionSkill");
