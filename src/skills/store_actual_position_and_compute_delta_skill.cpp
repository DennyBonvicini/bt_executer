#include <bt_executer/skills/store_actual_position_and_compute_delta_skill.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <control_msgs/msg/interface_value.hpp>
#include <chrono>
//#include <unordered_map>

using DynamicJointState = control_msgs::msg::DynamicJointState;
using InterfaceValue = control_msgs::msg::InterfaceValue; //usato per ricostruire il messaggio da stampare in output

StoreActualPositionNodeAndComputeDelta::StoreActualPositionNodeAndComputeDelta(const std::string& name, 
						 const BT::NodeConfig& conf,
                        			 const BT::RosNodeParams& params)
    : RosTopicSubNode<DynamicJointState>(name, conf, params),
      previous_positions_(7, 0.0)
{
   auto param_ns = getInput<std::string>("topic_name");
   ns_ = "/bt_executer/" + param_ns.value();
   
   // Inizializzo il publisher: pubblica su un topic chiamato "/delta_movements" per poi poter calcolare il centro della vite)
   delta_joint_position_publisher_ = node_.lock()->create_publisher<std_msgs::msg::Float64MultiArray>("/delta_movements_joint", 10);

   delta_cartesian_position_publisher_ = node_.lock()->create_publisher<std_msgs::msg::Float64>("/delta_movements_cartesian", 10);
   
   // Sottoscrizione al topic /previous_position
   joint_position_sub_ = node_.lock()->create_subscription<InterfaceValue>(
       "/previous_position", 10,
       std::bind(&StoreActualPositionNodeAndComputeDelta::previousPositionCallback, this, std::placeholders::_1));

   // Sottoscrizione al topic /cartesian_position
   cartesian_position_sub_ = node_.lock()->create_subscription<geometry_msgs::msg::Pose>(
       "/cartesian_position", 10,
       std::bind(&StoreActualPositionNodeAndComputeDelta::cartesianPositionCallback, this, std::placeholders::_1));

   tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_.lock()->get_clock());
   tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// Callback per il subscriber a /previous_position
void StoreActualPositionNodeAndComputeDelta::previousPositionCallback(const InterfaceValue::SharedPtr msg)
{
    // Aggiorna posizioni precedenti dei giunti KUKA
    for (size_t i = 0; i < msg->interface_names.size(); ++i)
    {
        previous_positions_[i] = msg->values[i];
        
        //debug
        RCLCPP_INFO(node_.lock()->get_logger(), "Posizione_prec giunto %zu : %f", i, msg->values[i]);
    }

    RCLCPP_INFO(node_.lock()->get_logger(), "Posizioni precedenti aggiornate");  
}

// Callback per il subscriber a /cartesian_position
void StoreActualPositionNodeAndComputeDelta::cartesianPositionCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
        previous_x = msg->position.x;
        previous_y = msg->position.y;
        previous_z = msg->position.z;

        //debug
        RCLCPP_INFO(node_.lock()->get_logger(), "Posizione_prec x: %f, y: %f ,z: %f", previous_x, previous_y, previous_z);

    RCLCPP_INFO(node_.lock()->get_logger(), "Posizioni precedenti aggiornate");
}


BT::NodeStatus StoreActualPositionNodeAndComputeDelta::onTick(const std::shared_ptr<DynamicJointState>& last_msg)
{    
    if(last_msg)
    {
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
            
           
            // costruisco un'unica stringa per il messaggio
            std::string full_msg = "Stato attuale dei giunti KUKA:\n";
            for (size_t i = 0; i < kuka_joint_msg.interface_names.size(); ++i)
            {
                full_msg += "Nome giunto: " + kuka_joint_msg.interface_names[i] +
                            ", Posizione: " + std::to_string(kuka_joint_msg.values[i]) + "\n";
            }
            

            // Stampa tutto il messaggio in una sola volta, più leggibile su terminale
            RCLCPP_INFO(node_.lock()->get_logger(), "%s", full_msg.c_str());
            
            
            // CALCOLO DELTA PER JOINT POSITION

            std::vector<double> delta(7, 0.0);
            
            // Calcolo dei delta con i valori di previous_position
            for (size_t i = 0; i < kuka_joint_msg.interface_names.size(); ++i)
            {
                delta[i] = kuka_joint_msg.values[i] - previous_positions_[i];
            }

            std_msgs::msg::Float64MultiArray delta_joint_msg;
            delta_joint_msg.data = delta;
            //pubblico il delta movimento
            delta_joint_position_publisher_->publish(delta_joint_msg);

            //costruisco stringa per debuggare delta
            std::string delta_str = "Delta Movements joints: ";
            for (const auto& value : delta_joint_msg.data) {
                delta_str += std::to_string(value) + " ";
            }

            RCLCPP_INFO(node_.lock()->get_logger(), "%s", delta_str.c_str());

            // Stampa il messaggio in output con più chiamate a RCLCPP_INFO
            /*RCLCPP_INFO(node_.lock()->get_logger(), "[%s] Stato attuale dei giunti KUKA:", name().c_str());
            for (size_t i = 0; i < kuka_joint_msg.interface_names.size(); ++i)
            {
                RCLCPP_INFO(node_.lock()->get_logger(), "Nome giunto: %s, Posizione: %f",
                            kuka_joint_msg.interface_names[i].c_str(),
                            kuka_joint_msg.values[i]);
            }*/

            // CALCOLO DELTA PER CARTESIAN POSITION

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

            // Logging della posizione cartesiana
            RCLCPP_INFO(node_.lock()->get_logger(), "Pubblicata posizione cartesiana: [x: %f, y: %f, z: %f]",
                        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);

            double delta_x = std::abs(pose_msg.position.x - previous_x);
            double delta_y = std::abs(pose_msg.position.y - previous_y);
            double delta_z = std::abs(pose_msg.position.z - previous_z);

            RCLCPP_INFO(node_.lock()->get_logger(), "delta cartesiani: [delta_x: %f, delta_y: %f, delta_z: %f]",
                        delta_x, delta_y, delta_z);

            // Trova il valore massimo
            double max_delta = std::max({(delta_x), (delta_y), (delta_z)});

            std_msgs::msg::Float64 delta_cartesian_msg;
            delta_cartesian_msg.data = max_delta;
            delta_cartesian_position_publisher_->publish(delta_cartesian_msg);

            RCLCPP_INFO(node_.lock()->get_logger(),"DELTA MOVIMENTO CARTESIANO: %f", delta_cartesian_msg.data);


            return BT::NodeStatus::SUCCESS;
        }
    }
    
    return BT::NodeStatus::FAILURE;
}
   
// Plugin registration.
// The class StoreActualPositionNodeAndComputeDelta will self register with name  "StoreActualPositionNodeAndComputeDeltaSkill".
CreateRosNodePlugin(StoreActualPositionNodeAndComputeDelta, "StoreActualPositionAndComputeDeltaSkill");
