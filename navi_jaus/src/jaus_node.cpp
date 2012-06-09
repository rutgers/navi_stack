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
    
    //TODO: update with appropriate calls
    ros::init(argc, argv, "jaus");                     ///////////////
    ros::NodeHandle nh;

    //SETUP JAUS-----------------------------------------------------------------------------------//

    //Create JAUS component 
    JAUS::Component component;                   
                                    
    //Disable timeout                                    
    component.AccessControlService()->SetTimeoutPeriod(0);      

    //Add Global Position Sensor, Local Position Sensor, Velocity Sensor to JAUS component 
    JAUS::GlobalPoseSensor* globalPoseSensor = new JAUS::GlobalPoseSensor();
    globalPoseSensor->SetSensorUpdateRate(25); //update @ 25Hz   
    component.AddService(globalPoseSensor);      
                                                      
    JAUS::LocalPoseSensor* localPoseSensor = new JAUS::LocalPoseSensor(); 
    localPoseSensor->SetSensorUpdateRate(25);  //update @ 25Hz  
    component.AddService(localPoseSensor);  
                                                
    JAUS::VelocityStateSensor* velocityStateSensor = new JAUS::VelocityStateSensor();
    velocityStateSensor->SetSensorUpdateRate(25);  //update @ 25Hz   
    component.AddService(velocityStateSensor); 
                                        
    //List for waypoints                               
    component.AddService(new JAUS::ListManager());           
    JAUS::LocalWaypointListDriver* localWaypointListDriver = new JAUS::LocalWaypointListDriver();  
    component.AddService(localWaypointListDriver);     
                                       
    //make discoverable
    //TASK 1 -------------------------XXX
    component.DiscoveryService()->SetSubsystemIdentification(JAUS::Subsystem::Vehicle, "navi");

    //Initialize JAUS, all components should be added at this time
    if(component.Initialize(JAUS::Address(ROBOT_SUBSYSTEM_ID, ROBOT_NODE_ID, ROBOT_COMPONENT_ID)) == false)
    {
        ROS_WARN("Failed to initialize JAUS");
        return 0;
    }

    //Standby
    component.ManagementService()->SetStatus(JAUS::Management::Status::Standby);    
    
    //add JDUP as service ............. JTCPClient ......JTCP
    JAUS::JUDP *transportService = (JAUS::JUDP*)component.TransportService();

    transportService->AddConnection(COP_IP_ADDR, JAUS::Address(COP_SUBSYSTEM_ID, COP_NODE_ID, COP_COMPONENT_ID));
      
            
                                                                                     
    // Set an initial global pose.                                              
    // TODO: pull values from GPS    
    // TODO: pull values from kalman filter, change to global and pass to JAUS
    JAUS::GlobalPose globalPose;                                          
    globalPose.SetLatitude(/*Latitudecall*/);                            
    globalPose.SetLongitude(/*longcall*/);                               
    //globalPose.SetAltitude(0);                                       
    globalPose.SetPositionRMS(0.0);                                         
    //globalPose.SetRoll(0.0);                                             
    //globalPose.SetPitch(0.0);                                           
    //globalPose.SetYaw(/**/);     
    globalPose.SetAttitudeRMS(0.0);                                  
    globalPose.SetTimeStamp(JAUS::Time::GetUtcTime());          
                                                                                                   
    // Set an initial velocity state.                                                              
    JAUS::VelocityState velocityState;                                                             
    velocityState.SetVelocityX(0.0);                                                               
    velocityState.SetYawRate(0.0);                                                                 
    velocityState.SetVelocityRMS(0.0);                                                             
    velocityState.SetTimeStamp(JAUS::Time::GetUtcTime());               
        
    //Save the data to the service. 
    //send globalPose to localPose, conversion automatic
    globalPoseSensor->SetGlobalPose(globalPose);                                  
    localPoseSensor->SetLocalPose(globalPose);                                                     
    velocityStateSensor->SetVelocityState(velocityState);                                          
                                                                                                  
    JAUS::Time::Stamp printTimeMs = 0;  


    //END SETUP JAUS--------------------------------------------------------------------------------//

                                                                                                                                
    /////////////////////////////////////////////////////////////////////////////////////////////////

    
    
    
    
    
    //Main JAUS loop-------------------------------------------------------------------------------//
                                                                                                   //      
                                                                                                   //
    while(ros::ok())
    {
        
       //handle shutdown, standby, waypoint collection
        //shutdown
        if(component.ManagementService()->GetStatus() == JAUS::Management::Status::Shutdown)
        {
		    std::cout << "\nTerminating the Program. Shutdown command receviced!\n";

		    break;
		  }
		  //If standby
		  if(component.ManagementService()->GetStatus() == JAUS::Management::Status::Standby)
        {
		    std::cout << "\nStandby!\n";
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
				 commandList.push_back(*( (JAUS::SetLocalWaypoint *)(listElement->second.mpElement)) );
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
