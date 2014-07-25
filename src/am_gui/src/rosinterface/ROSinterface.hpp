#ifndef ROSINTERFACE_H
#define ROSINTERFACE_H

#include <QObject>

#include <QFile>
#include <QTextStream>
#include <QString>
#include <vector>
#include <urdf/model.h>
#include <ros/package.h> 

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

#include <tf_rot.hpp>
#include <joint_info.hpp>

// EUROC 
#include <euroc_c2_msgs/ListScenes.h>
#include <euroc_c2_msgs/StartSimulator.h>
#include <euroc_c2_msgs/StopSimulator.h>
#include <euroc_c2_msgs/RequestNextObject.h>
#include <euroc_c2_msgs/MoveAlongJointPath.h>
#include <euroc_c2_msgs/GetTimingAlongJointPath.h>
#include <euroc_c2_msgs/SearchIkSolution.h>
#include <euroc_c2_msgs/SetStopConditions.h>

#include <euroc_c2_msgs/EnableServoMode.h>
#include <euroc_c2_msgs/SetServoTarget.h>


#include <euroc_c2_msgs/Telemetry.h>
#include <euroc_c2_msgs/GetForwardKinematics.h>


#include <boost/thread.hpp>

class ROSinterface : public QObject
{
	Q_OBJECT

private:

	explicit ROSinterface(QObject *parent=0);
	~ROSinterface();



	static ROSinterface* m_ROSinterface;
	int sgn(double);

	ros::NodeHandle nh;

	ros::ServiceClient start_simulator_client_;
	ros::ServiceClient stop_simulator_client_;
	ros::ServiceClient list_scenes_client_;
	ros::ServiceClient next_object_client_;
	ros::ServiceClient set_stop_conditions_client_;

	ros::ServiceClient move_along_joint_path_client_;
	ros::ServiceClient timing_along_joint_path_client_;
	ros::ServiceClient search_ik_solution_client_;

	ros::ServiceClient enable_servo_mode_client_;
	ros::ServiceClient set_servo_target_client_;

	ros::Subscriber telemetry_subscriber_;


	//Wait for the simulator services and wait for them to be available:
	std::string task_selector_;
	std::string start_simulator_;
	std::string stop_simulator_;
	std::string list_scenes_;
	std::string next_object_;
	std::string set_stop_conditions_;

	std::string enable_servo_mode_;
	std::string set_servo_target_;

	std::string euroc_c2_interface_;
	std::string telemetry_;
	std::string move_along_joint_path_;
	std::string timing_along_joint_path_;
	std::string search_ik_solution_;

	std::vector<std::string> joint_names_;
	double  measured_positions_[12];
	double  measured_forces_[12];
	double  measured_external_forces_[12];

	bool getIKSolution7DOF();
	boost::thread moveToTarget;

	geometry_msgs::Pose pose_;
	euroc_c2_msgs::SearchIkSolution search_ik_solution_srv_;
	euroc_c2_msgs::MoveAlongJointPath move_along_joint_path_srv_;
	euroc_c2_msgs::GetTimingAlongJointPath timing_along_joint_path_srv_;
	euroc_c2_msgs::ListScenes list_scenes_srv_;
	euroc_c2_msgs::RequestNextObject next_object_srv_;
	euroc_c2_msgs::SetStopConditions set_stop_conditions_srv_;
	euroc_c2_msgs::EnableServoMode enable_servo_mode_srv_;
	euroc_c2_msgs::SetServoTarget set_servo_target_srv_;

	urdf::Model model_robot_;
	urdf::Model model_gripper_;

	std::vector<jointinfo> system_limits_;

	void moveToTargetCB();

	void getUrdfConf();
	bool getSceneList();

public:

	static ROSinterface* getInstance();
	std::vector<euroc_c2_msgs::Scene> scenes_;
	euroc_c2_msgs::Telemetry _telemetry;
public slots:

void callStartSimulator(std::string);
void callStopSimulator();
void callSetCustomGoalConfiguration(double*);
void callMoveToTargetPose(double*);
void callNextObject();
void callSetStopConditions(std::vector<std::string>,std::vector<std::string>,std::vector<double>);

void sendCurrentCfgOnce();

void on_telemetry(const euroc_c2_msgs::Telemetry &telemetry);
private slots:

void getTimingAlongJointPath();

signals:
void emitMeasuredValues(std::vector<std::string>, double*, double*, double*);
void emitCurrentCfgOnce(std::vector<std::string>,double*);


};

#endif // ROSINTERFACE_H
