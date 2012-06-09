//JAUS Headers-------------------------------------------------//
#include <jaus/mobility/sensors/globalposesensor.h>            
#include <jaus/mobility/sensors/localposesensor.h>             
#include <jaus/mobility/sensors/velocitystatesensor.h>         
#include <jaus/mobility/drivers/localwaypointlistdriver.h>     
#include <jaus/core/transport/judp.h>                          
#include <jaus/core/transport/jtcpclient.h>                    
#include <jaus/core/component.h>                               
#include <cxutils/keyboard.h>                                  
#include <cxutils/math/cxmath.h>                               
#include <iostream>                                            
#include <cstring>                                             
#include <cstdio>                                              
#include <cmath>                                               
/////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jaus");
    ros::NodeHandle nh;

    //SETUP JAUS-----------------------------------------------------------------------------------//

    //Create JAUS component
    JAUS::Component component;

    //Disable timeout
    component.AccessControlService()->SetTimeoutPeriod(0);

    //Add Global Position Sensor, Local Position Sensor, Velocity Sensor to JAUS component
    JAUS::GlobalPoseSensor* globalPoseSensor = new JAUS::GlobalPoseSensor();
    component.AddService(globalPoseSensor);

    JAUS::LocalPoseSensor* localPoseSensor = new JAUS::LocalPoseSensor();
    component.AddService(localPoseSensor);

    JAUS::VelocityStateSensor* velocityStateSensor = new JAUS::VelocityStateSensor();
    component.AddService(velocityStateSensor);

    //List for waypoints
    component.AddService(new JAUS::ListManager());
    JAUS::LocalWaypointListDriver* localWaypointListDriver = new JAUS::LocalWaypointListDriver();
    component.AddService(localWaypointListDriver);

    component.DiscoveryService()->SetSubsystemIdentification(JAUS::Subsystem::Vehicle, "navi");

    //Initialize JAUS, all components should be added at this time
    if(component.Initialize(JAUS::Address(ROBOT_SUBSYSTEM_ID, ROBOT_NODE_ID, ROBOT_COMPONENT_ID)) == false)
    {
        ROS_WARN("Failed to initialize JAUS");
        return 0;
    }

    //Standby
    component.ManagementService()->SetStatus(JAUS::Management::Status::Standby);

    JAUS::JUDP *transportService = static_cast<JAUS:JUDP *>(component.TransportService());
    transportService->AddConnection(COP_IP_ADDR, JAUS::Address(COP_SUBSYSTEM_ID, COP_NODE_ID, COP_COMPONENT_ID));

    JAUS::LocalPose local_pos;
    JAUS::VelocityState velocityState;

    localPoseSensor->SetLocalPose(local_post);
    velocityStateSensor->SetVelocityState(velocityState);

    JAUS::Time::Stamp printTimeMs = 0;

    //END SETUP JAUS--------------------------------------------------------------------------------//
    //Main JAUS loop-------------------------------------------------------------------------------//

    JAUS::Management management = component.ManagementService();

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

        // Get local waypoint list
        JAUS::Element::Map elementList = localWaypointListDriver->GetElementList();
        // Convert to SetLocalWaypointCommands
        JAUS::Element::Map::iterator listElement;
        std::vector<JAUS::SetLocalWaypoint> commandList;
        for(listElement = elementList.begin(); listElement != elementList.end(); listElement++)
        {
            if(listElement->second.mpElement->GetMessageCode() == JAUS::SET_LOCAL_WAYPOINT)
            {
                commandList.push_back(*((JAUS::SetLocalWaypoint *)(listElement->second.mpElement)) );
            }
        }

        //commandList is a vector of Waypoints

        //Execute 
        if(localWaypointListDriver->IsExecuting())
        {
            // TODO: Go through this, should be replace by executive

        }

        //XXX:Everything below is correct except TODO                                                                                           

        // Save newly calculated position and orientation.     
        //TODO: Update collection of data with calls
        //globalPose.SetYaw(/**/);                                                                 
        globalPose.SetLatitude(/**/);                                      
        globalPose.SetLongitude(/**/);                                 
        globalPose.SetAltitude(/**/);                          
        globalPose.SetTimeStamp(JAUS::Time(true));                                      

        velocityState.SetVelocityX(/**/);                             
        velocityState.SetYawRate(/**/);                           
        velocityState.SetTimeStamp(JAUS::Time(true));                      

        // Save the data to the service                                 
        globalPoseSensor->SetGlobalPose(globalPose);        
        localPoseSensor->SetLocalPose(globalPose);                         
        velocityStateSensor->SetVelocityState(velocityState);         





        //Update Status
        if(JAUS::Time::GetUtcTimeMs() - printTimeMs > 5000)                                         
        {                                                                                          
            // Print status of services.                                                           
            std::cout << "\n=======================Basic Service Status============================\n";             
            component.AccessControlService()->PrintStatus(); std::cout << std::endl;               
            component.ManagementService()->PrintStatus(); std::cout << std::endl;                  
            //globalPoseSensor->PrintStatus(); std::cout << std::endl;                               
            localPoseSensor->PrintStatus(); std::cout << std::endl;                                
            velocityStateSensor->PrintStatus(); std::cout << std::endl;                            
            localWaypointListDriver->PrintStatus();                                                
            printTimeMs = JAUS::Time::GetUtcTimeMs();                                              
        }                                                                                          

        // Exit if escape key pressed.                                                             
        if(CxUtils::GetChar() == 27)                                                               
        {                                                                                          
            break;                                                                                 
        }                                                                                          


        CxUtils::SleepMs(250);
        ros::spinOnce();
    }                                                                                              
    //END MAIN JAUS LOOP---------------------------------------------------------------------------//
                                                                                                   
    // Shutdown any components associated with our subsystem.                                  
    component.Shutdown();                                                   

    return 0;
}
