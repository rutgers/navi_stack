#include <jaus/mobility/sensors/localposesensor.h>
#include <jaus/mobility/sensors/velocitystatesensor.h>
#include <jaus/mobility/drivers/localwaypointlistdriver.h>
#include <jaus/core/transport/judp.h>
#include <jaus/core/component.h>

#include <tf/transform_datatypes.h>

/* TODO: replace with tf or eigen */
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btMatrix3x3.h>

#include <cxutils/time.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

static JAUS::LocalPoseSensor *local_pose_sensor;
static JAUS::VelocityStateSensor *velocity_state_sensor;

void position_cb(nav_msgs::Odometry::Ptr odom)
{
    JAUS::LocalPose local_pose;
    local_pose.SetX(odom->pose.pose.position.x);
    local_pose.SetY(odom->pose.pose.position.y);
    local_pose.SetZ(odom->pose.pose.position.z);

    CxUtils::Time cx_time(odom->header.stamp.toSec());
    local_pose.SetTimeStamp(cx_time);

    btQuaternion q;
    double r, p, y;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
    btMatrix3x3(q).getEulerRPY(r, p, y);

    local_pose.SetRoll(r);
    local_pose.SetPitch(p);
    local_pose.SetYaw(y);

    local_pose_sensor->SetLocalPose(local_pose);

    JAUS::VelocityState velocity_state;

    velocity_state.SetVelocityX(odom->twist.twist.linear.x);
    velocity_state.SetVelocityY(odom->twist.twist.linear.y);
    velocity_state.SetVelocityZ(odom->twist.twist.linear.z);

    velocity_state.SetRollRate (odom->twist.twist.angular.x);
    velocity_state.SetPitchRate(odom->twist.twist.angular.y);
    velocity_state.SetYawRate  (odom->twist.twist.angular.z);

    velocity_state.SetTimeStamp(cx_time);

    velocity_state_sensor->SetVelocityState(velocity_state);
}

class ShutdownCB : public JAUS::Callback {
    public:
        ControlCallback(JAUS_Controller* c): parent(c) {}
        ~ControlCallback() {}
        virtual void ProcessMessage(const JAUS::Message* message);
    private:
        JAUS_Controller* parent;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "jaus");
    ros::NodeHandle nh;

    std::string odom_path;
    nh.getParam("~odom", odom_path, "/odom_fuse");

    JAUS::Component component;
    // Disable timeout. Normally, service would shutdown in 2 seconds.
    component.AccessControlService()->SetTimeoutPeriod(0);

    // WARNING: must be allocated via 'new'. JAUS::Component::Shutdown calls delete on this.
    local_pose_sensor = new JAUS::LocalPoseSensor();
    component.AddService(local_pose_sensor);

    // WARNING: must be allocated via 'new'. JAUS::Component::Shutdown calls delete on this.
    velocity_state_sensor = new JAUS::VelocityStateSensor();
    component.AddService(velocity_state_sensor);

    // WARNING: must be allocated via 'new'. JAUS::Component::Shutdown calls delete on this.
    component.AddService(new JAUS::ListManager());
    // WARNING: must be allocated via 'new'. JAUS::Component::Shutdown calls delete on this.
    JAUS::LocalWaypointListDriver* localWaypointListDriver = new JAUS::LocalWaypointListDriver();
    component.AddService(localWaypointListDriver);

    component.DiscoveryService()->SetSubsystemIdentification(JAUS::Subsystem::Vehicle, "navi");

    //Initialize JAUS, all components should be added at this time
    if(component.Initialize(JAUS::Address(ROBOT_SUBSYSTEM_ID, ROBOT_NODE_ID, ROBOT_COMPONENT_ID)) == false)
    {
        ROS_WARN("Failed to initialize JAUS");
        return 0;
    }

    // NOTE: muse run after local_pose_sensor and velocity_state_sensor created.
    ros::Subscriber position = nh.subscribe(odom_path, 1, position_cb);

    component.ManagementService()->SetStatus(JAUS::Management::Status::Standby);

    {
        JAUS::JUDP *transport = static_cast<JAUS::JUDP *>(component.TransportService());
        transport->AddConnection(COP_IP_ADDR, JAUS::Address(COP_SUBSYSTEM_ID, COP_NODE_ID, COP_COMPONENT_ID));
        transport->RegisterCallback(JAUS::SHUTDOWN, jaus_shutdown);
    }

    JAUS::Time::Stamp printTimeMs = 0;

    JAUS::Management *management = component.ManagementService();

    while(ros::ok())
    {
        JAUS::Management::Status status = management->GetStatus();

        if (status == JAUS::Management::Status::Shutdown)
        {
            ROS_INFO("Shutdown receviced");
            break;
        }

        if (status == JAUS::Management::Status::Standby)
        {
            /* FIXME: what do we do durring standby? Do we need to look at the waypoint list at all? */
            ROS_INFO("JAUS Standby");
        }

        //Update Status
        if(JAUS::Time::GetUtcTimeMs() - printTimeMs > 5000)                                         
        {                                                                                          
            // Print status of services.                                                           
            std::cout << "\n=======================Basic Service Status============================\n";             
            component.AccessControlService()->PrintStatus(); std::cout << std::endl;               
            component.ManagementService()->PrintStatus(); std::cout << std::endl;                  
            //globalPoseSensor->PrintStatus(); std::cout << std::endl;                               
            local_pose_sensor->PrintStatus(); std::cout << std::endl;                                
            velocity_state_sensor->PrintStatus(); std::cout << std::endl;                            
            localWaypointListDriver->PrintStatus();                                                
            printTimeMs = JAUS::Time::GetUtcTimeMs();                                              
        }                                                                                          

        CxUtils::SleepMs(250);
        ros::spinOnce();
    }
    //END MAIN JAUS LOOP---------------------------------------------------------------------------//
    // Shutdown any components associated with our subsystem.                                  
    component.Shutdown();                                                   

    return 0;
}
