#include <memory>
#include <vector>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

void planAndExecute(moveit::planning_interface::MoveGroupInterface& move_group_interface,
                    const std::string PlannerID,
                    const geometry_msgs::msg::PoseStamped& target_pose,                   
                    const double velocity_scaling_factor,
                    const double acceleration_scaling_factor,
                    const rclcpp::Logger& logger) {

  move_group_interface.setPlannerId(PlannerID);
  move_group_interface.setPoseTarget(target_pose);  //evtl. mit weiteren Funktionswerten (target_pose,"hand");
  move_group_interface.setMaxVelocityScalingFactor(velocity_scaling_factor);
  move_group_interface.setMaxAccelerationScalingFactor(acceleration_scaling_factor);
  
  if(PlannerID == "PTP"){
    move_group_interface.setNumPlanningAttempts(10);
  }
  else if(PlannerID == "LIN"){
    move_group_interface.setNumPlanningAttempts(10);
  }
  else{

  }

  if(PlannerID == "LIN"){
    move_group_interface.setGoalTolerance(0.001); //1mm
  }
  else{
    move_group_interface.setGoalTolerance(0.01); //0.0175 = 1 Grad
  }  

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
    move_group_interface.move();
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
}


// X Y Z -------------- TRANSFORMATION WORLD-BASE ----------------
// 0 0 0 -> world-tisch
//-0.394 -0.05 1.033 -> tisch-unterlage
// 0 0 0.045  -> unterlage-base
// GESAMT: -0.394 -0.05 1.078

// X Y Z -------------- TRANSFORMATION LINK6-TOOL ----------------
// xyz="0.01 0.04 -0.053" rpy="0 0 -1.57" -> link6-flange
// xyz="0.04 0.054 0.037" rpy="0 0 1.57"  -> flange-tool(mittig)
// DREHUNG 90Grad um Z -> 2.Transformation: z=z x=y y=-x
// xyz="0.01 0.04 -0.053" -> link6-flange
// xyz="0.054 -0.04 0.037" -> flange-tool(mittig)
// GESAMT: 0.064 0 -0.016
// Invertiert Gesamt: -0.064 0 0.016


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  	  // Create the MoveIt MoveGroup Interface
	using moveit::planning_interface::MoveGroupInterface;
	//auto move_group_interface = MoveGroupInterface(node, "panda_arm");kr10_arm
  auto move_group_interface = MoveGroupInterface(node, "kr10_arm");

	move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");

  // Wir können den Namen des Referenz-Frames für diesen Roboter bekommen:
  std::string planning_frame = move_group_interface.getPlanningFrame();
  RCLCPP_INFO(node->get_logger(), "============ Planning frame: %s", planning_frame.c_str());

  // Wir können auch den Namen des Endeffektors für diese Gruppe ausgeben:
  std::string eef_link = move_group_interface.getEndEffectorLink();
  RCLCPP_INFO(node->get_logger(), "============ End effector link: %s", eef_link.c_str());

    //Koordinaten sind angegeben im Weltkoordinatensystem -> Inverse Transformation von Welt zu Roboter Basis
    float Base_pos_X_mm = -394;
    float Base_pos_Y_mm =  -50;
    float Base_pos_Z_mm = 1078;

    // Abstand des Basiskoordinatensystems des Roboters zum Werkobjekt Nullpunkt (siehe CAM System Ursprung)
    float Wobj_X_base_mm = 500;
    float Wobj_Y_base_mm = 250;
    float Wobj_Z_base_mm = 300;

    // Inverser Kinematic Solver kann nur für Flansch Achse 6 Rechnen -> Inverse Transformation von link6 zu tool0 (Spindel)
    float link6_tool0_X_mm = -0.064;
    float link6_tool0_Y_mm =  0.0;
    float link6_tool0_Z_mm =  0.016;
    //float link6_tool0_X_grad = 0.0;
    //float link6_tool0_Y_grad = 0.0;
    //float link6_tool0_Z_grad = 0.0;

    // Startpunkt anfahren: 
    auto const rob_pose_start = [&] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      //msg.header.frame_id = "world";base
      msg.pose.orientation.x = 0.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 1.0;
      msg.pose.position.x = (Base_pos_X_mm + Wobj_X_base_mm + link6_tool0_X_mm + 75)/1000.0;
      msg.pose.position.y = (Base_pos_Y_mm + Wobj_Y_base_mm + link6_tool0_Y_mm + 75)/1000.0;
      msg.pose.position.z = (Base_pos_Z_mm + Wobj_Z_base_mm + link6_tool0_Z_mm + 150)/1000.0;
      return msg;
    }();
    planAndExecute(move_group_interface,"PTP",rob_pose_start,0.1,0.1,logger);

    // Textdatei einlesen und Bewegungen planen:
    // sPlanen2.src";
    // sTasche1.src";
    std::string filename = "/home/claudius/kr10pkg/src/ros_milling_cpp/src/nc_program/test/sPlanen2.src";
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cout << "Failed to open file " << filename << std::endl;
        return -1;
    }

    // Parameter Startwerte Geschwindigkeit skaliert und Beschleunigung skaliert
    float VEL_float = 0.1;
    float CP_acc = 0.1;
    float X_float = 0.0;
    float Y_float = 0.0;
    float Z_float = 0.0;


    // Fräsprozess planen:
    std::string line;
    int amount_of_lines = 0;
    int line_num = 0;
    while (std::getline(file, line))
    {
      if (line.substr(0, 3) == "LIN") // Prüfen, ob die Zeile mit "LIN" beginnt
      {
        ++amount_of_lines;
      }

      if (line.substr(0, 8) == "$VEL.CP=") // Prüfen, ob die Zeile mit "LIN" beginnt
      {
        ++amount_of_lines;
      }
    }
    file.close();


    std::ifstream file2(filename);

    if (!file2.is_open())
    {
        std::cout << "Failed to open file " << filename << std::endl;
        return -1;
    }

    while (std::getline(file2, line))
    {
      if (line.substr(0, 3) == "LIN") // Prüfen, ob die Zeile mit "LIN" beginnt
      {
        // Extrahieren der X-, Y- und Z-Werte aus der Zeile
        size_t x_pos = line.find("X");
        size_t y_pos = line.find("Y");
        size_t z_pos = line.find("Z");
        X_float = std::stof(line.substr(x_pos + 2, y_pos - x_pos - 2));
        Y_float = std::stof(line.substr(y_pos + 2, z_pos - y_pos - 2));
        Z_float = std::stof(line.substr(z_pos + 2));
        ++line_num;
        RCLCPP_INFO(node->get_logger(), "X: %f Y: %f Z: %f", X_float,Y_float,Z_float);
        RCLCPP_INFO(node->get_logger(), "Processing line %d of %d", line_num, amount_of_lines);

        float x_pos_f = (Base_pos_X_mm + Wobj_X_base_mm + link6_tool0_X_mm + X_float)/1000.0;
        float y_pos_f = (Base_pos_Y_mm + Wobj_Y_base_mm + link6_tool0_Y_mm + Y_float)/1000.0;
        float z_pos_f = (Base_pos_Z_mm + Wobj_Z_base_mm + link6_tool0_Z_mm + Z_float)/1000.0;

        auto const rob_pose_temp = [&] {
          geometry_msgs::msg::PoseStamped msg;
          msg.header.frame_id = "world";
          //msg.header.frame_id = "world";base
          msg.pose.orientation.x = 0.0;
          msg.pose.orientation.y = 0.0;
          msg.pose.orientation.z = 0.0;
          msg.pose.orientation.w = 1.0;
          msg.pose.position.x = x_pos_f;
          msg.pose.position.y = y_pos_f;
          msg.pose.position.z = z_pos_f;
          return msg;
        }();
        planAndExecute(move_group_interface,"LIN",rob_pose_temp,VEL_float,CP_acc,logger);
      }

      if (line.substr(0, 8) == "$VEL.CP=") // Prüfen, ob die Zeile mit "LIN" beginnt
      {
        VEL_float = std::stof(line.substr(8));
        ++line_num;
        RCLCPP_INFO(node->get_logger(), "VEL_CP: %s", std::to_string(VEL_float).c_str());
      }
    }

    file2.close();

    // Roboter zurück zu Start
    planAndExecute(move_group_interface,"PTP",rob_pose_start,0.1,0.1,logger);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}