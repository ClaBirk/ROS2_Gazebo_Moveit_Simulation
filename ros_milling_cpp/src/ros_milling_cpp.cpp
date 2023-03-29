#include <memory>
#include <vector>
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
  //move_group_interface.setEndEffectorLink("link6");
  //move_group_interface.setPoseReferenceFrame(reference_frame)

  
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
  std::vector<geometry_msgs::msg::PoseStamped> target_poses;

  // Wir können den Namen des Referenz-Frames für diesen Roboter bekommen:
  std::string planning_frame = move_group_interface.getPlanningFrame();
  RCLCPP_INFO(node->get_logger(), "============ Planning frame: %s", planning_frame.c_str());

  // Wir können auch den Namen des Endeffektors für diese Gruppe ausgeben:
  std::string eef_link = move_group_interface.getEndEffectorLink();
  RCLCPP_INFO(node->get_logger(), "============ End effector link: %s", eef_link.c_str());


    auto const rob_pose_1 = [] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      //msg.header.frame_id = "world";base
      msg.pose.orientation.x = 0.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 1.0;
      msg.pose.position.x = 0.28; //0.28
      msg.pose.position.y = 0.2;  //0.2
      msg.pose.position.z = 1.5;  //1.5
      return msg;
    }();
    planAndExecute(move_group_interface,"LIN",rob_pose_1,0.1,0.1,logger);


    auto const rob_pose_2 = [] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      //msg.header.frame_id = "world";base_link
      msg.pose.orientation.x = 0.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 1.0;
      msg.pose.position.x = 0.48;
      msg.pose.position.y = 0.2;
      msg.pose.position.z = 1.5;
      return msg;
    }();
    planAndExecute(move_group_interface,"LIN",rob_pose_2,0.01,0.01,logger);


    auto const rob_pose_3 = [] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      //msg.header.frame_id = "world";base_link
      msg.pose.orientation.x = 0.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 1.0;
      msg.pose.position.x = 0.38;
      msg.pose.position.y = 0.28;
      msg.pose.position.z = 1.5;
      return msg;
    }();
    planAndExecute(move_group_interface,"LIN",rob_pose_3,0.9,0.25,logger);


    auto const rob_pose_4 = [] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      //msg.header.frame_id = "world";base_link
      msg.pose.orientation.x = 0.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 1.0;
      msg.pose.position.x = 0.3;
      msg.pose.position.y = 0.4;
      msg.pose.position.z = 1.5;
      return msg;
    }();
    planAndExecute(move_group_interface,"LIN",rob_pose_4,0.05,0.05,logger);



  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

// ERGÄNZEN

/*
#include <iostream>
#include <fstream>
#include <string>

int main()
{
    // Öffne die Textdatei
    std::ifstream infile("example.txt");
    // Erstelle eine temporäre Datei zum Schreiben
    std::ofstream outfile("temp.txt");

    std::string line;
    // Durchlaufe jede Zeile der Textdatei
    while (std::getline(infile, line))
    {
        // Überprüfe, ob die Zeile mit ";" oder "&" beginnt
        if (line[0] == ';' || line[0] == '&')
        {
            // Falls ja, überspringe die Zeile
            continue;
        }
        // Schreibe die Zeile in die temporäre Datei
        outfile << line << std::endl;
    }

    // Schließe die Textdateien
    infile.close();
    outfile.close();

    // Lösche die ursprüngliche Datei
    std::remove("example.txt");
    // Benenne die temporäre Datei in den ursprünglichen Dateinamen um
    std::rename("temp.txt", "example.txt");

    return 0;
}
*/
