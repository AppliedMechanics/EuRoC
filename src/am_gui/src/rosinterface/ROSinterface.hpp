#ifndef ROSINTERFACE_H
#define ROSINTERFACE_H

#include <QObject>
//#include <QUdpSocket>
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
#include <euroc_c2_msgs/StartSimulator.h>
#include <euroc_c2_msgs/StopSimulator.h>
#include <euroc_c2_msgs/MoveAlongJointPath.h>
#include <euroc_c2_msgs/SearchIkSolution.h>

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

	ros::ServiceClient start_simulator_client;
	ros::ServiceClient stop_simulator_client;

	ros::ServiceClient move_along_joint_path_client;
	ros::ServiceClient search_ik_solution_client;

	ros::Subscriber telemetry_subscriber;

	euroc_c2_msgs::Telemetry _telemetry;


	//Wait for the simulator services and wait for them to be available:
	std::string task_selector;
	std::string start_simulator;
	std::string stop_simulator;

	std::string euroc_c2_interface;
	std::string telemetry;
	std::string move_along_joint_path;
	std::string search_ik_solution;

	std::vector<std::string> joint_names_;
	double  measured_positions_[12];
	double  measured_forces_[12];
	double  measured_external_forces_[12];

	bool getIKSolution7DOF();
	boost::thread moveToTarget;

	geometry_msgs::Pose pose_;
	euroc_c2_msgs::SearchIkSolution search_ik_solution_srv_;
	euroc_c2_msgs::MoveAlongJointPath move_along_joint_path_srv_;

	urdf::Model model_robot_;
	urdf::Model model_gripper_;

	std::vector<jointinfo> system_limits_;

	void moveToTargetCB();

	void getUrdfConf();

public:

	static ROSinterface* getInstance();

public slots:

void callStartSimulator(int);
void callStopSimulator();
void callSetCustomGoalConfiguration(double*);
void callMoveToTargetPose(double*);

void sendCurrentCfgOnce();

void on_telemetry(const euroc_c2_msgs::Telemetry &telemetry);
private slots:


signals:
void emitMeasuredValues(std::vector<std::string>, double*, double*, double*);
void emitCurrentCfgOnce(std::vector<std::string>,double*);

};

#endif // ROSINTERFACE_H
