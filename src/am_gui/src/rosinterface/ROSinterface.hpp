#ifndef ROSINTERFACE_H
#define ROSINTERFACE_H

#include <QObject>
//#include <QUdpSocket>
#include <QFile>
#include <QTextStream>
#include <QString>
#include <vector>
#include <ros/package.h> 

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

#include <tf_rot.hpp>

// EUROC 
#include <euroc_c2_msgs/StartSimulator.h>
#include <euroc_c2_msgs/StopSimulator.h>
#include <euroc_c2_msgs/MoveAlongJointPath.h>
#include <euroc_c2_msgs/SearchIkSolution.h>

#include <euroc_c2_msgs/Telemetry.h>
#include <euroc_c2_msgs/GetForwardKinematics.h>

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

	QString jointNames[12];
	double  measuredPositions[12];
	double  measuredForces[12];
	double  measuredExternalForces[12];
public:

	static ROSinterface* getInstance();

public slots:

void callStartSimulator(int);
void callStopSimulator();
void callMoveToTargetPose(double*);

void on_telemetry(const euroc_c2_msgs::Telemetry &telemetry);
private slots:


signals:
	void emitMeasuredValues(QString*, double*, double*, double*);


};

#endif // ROSINTERFACE_H
