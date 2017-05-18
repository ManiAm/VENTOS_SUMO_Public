
// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <microsim/MSVehicle.h>
#include <microsim/MSLane.h>
#include "MSCFModel_CACC.h"
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <utils/common/RandHelper.h>

// mani
#include <math.h>
#include <microsim/MSNet.h> // for getting the current simulation time

// ===========================================================================
// method definitions
// ===========================================================================

MSCFModel_CACC::MSCFModel_CACC(const MSVehicleType* vtype,
        int communicationType,
        SUMOReal MaxAccel,
        SUMOReal MaxDecel,
        SUMOReal T_d,
        SUMOReal tau,
        SUMOReal ComfAccel,
        SUMOReal ComfDecel,
        SUMOReal K_sc,
        SUMOReal K_v,
        SUMOReal K_d,
        SUMOReal K_a,
        SUMOReal V_int,
        SUMOReal K_v_f,
        SUMOReal K_g_f,
        bool degradeToACC) : MSCFModel_ACC(vtype, MaxAccel, MaxDecel, T_d, tau, ComfAccel, ComfDecel, K_sc, K_v, K_d, V_int)
{
    this->myCommunicationType = communicationType;
    this->myK_a = K_a;
    this->myK_v_f = K_v_f;
    this->myK_g_f = K_g_f;
    this->degradeToACC = degradeToACC;
}


MSCFModel_CACC::~MSCFModel_CACC() {}


SUMOReal
MSCFModel_CACC::followSpeed(const MSVehicle* const veh, SUMOReal speed, SUMOReal gap, SUMOReal predSpeed, SUMOReal predMaxDecel) const 
{
    // note 1: when this method is called, it means we definitely have a preceding car.         
    // note 2: we should not use variable 'a' directly from SUMO; we need to get it from OMNET++
    
    // modify vehicle parameters through vehAccess
    SUMOVehicle* sumoVehicle = MSNet::getInstance()->getVehicleControl().getVehicle(veh->getID());
    MSVehicle* vehAccess = dynamic_cast<MSVehicle*>(sumoVehicle);
    
    // ############################################################
    // Controller 1: CACC with one-vehicle look-ahead communication
    // ############################################################    
    if(myCommunicationType == 1)
    {
        // check if we have received all the required data from precedingVeh
        int result = allDataReceived(vehAccess, vehAccess->precedingVeh, speed, gap, predSpeed, predMaxDecel);        
        // switching to ACC
        if(result == -1) 
        {
            std::string vTypeID = veh->getVehicleType().getID();
            std::size_t found = vTypeID.find("@");
            if (found != std::string::npos)
                vTypeID.erase(found);

            // todo: a temporary solution to change parameters
            // should change back the parameters once we get out of ACC
            MSVehicleType* sumoVehicleType = MSNet::getInstance()->getVehicleControl().getVType(vTypeID);
            sumoVehicleType->getCarFollowModel().setHeadwayTime(1.2);
            sumoVehicleType->getCarFollowModel().setDelay(0.5);
            sumoVehicleType->getCarFollowModel().setK_v(1.0);
            sumoVehicleType->getCarFollowModel().setK_g(5.0);
            
            return MSCFModel_ACC::followSpeed(veh, speed, gap, predSpeed, predMaxDecel);
        }
    
        if(veh->debug)
        {        
            char buffer1 [900];
            sprintf (buffer1, "--------\n%s\n--------\nController: %s", 
                              veh->getID().c_str(),
                              "CACC with one-vehicle look-ahead communication");
            WRITE_MESSAGE(buffer1); 
        }
    
        // the gap in argument excludes minGap
        // we will add the minGap
        gap = gap + veh->getVehicleType().getMinGap();
        
        // apply measurement error
        if(veh->errorGap != 0 || veh->errorRelSpeed != 0)
            applyMeasurementError(vehAccess, &gap, speed, &predSpeed);        

        // printing input values (for debugging purposes)
        if(veh->debug)
        {
            char buffer [900];
            sprintf (buffer, "My parameters: speed=%.3f, accel=%.3f, gap=%.3f", 
                             speed, 
                             veh->getAcceleration(),
                             gap);
            WRITE_MESSAGE(buffer);
        
            char buffer2 [900];
            sprintf (buffer2, "Parameters of preceding vehicle %s: speed=%.3f, accel(wireless)=%.3f, maxDecel=%.3f", 
                              veh->precedingVeh.vehicleNameOmnet.c_str(),
                              predSpeed,
                              veh->precedingVeh.accelOmnet,
                              predMaxDecel);
            WRITE_MESSAGE(buffer2);
        }
        
        result = emergencyBrakeNeeded(vehAccess, speed, gap, predSpeed, predMaxDecel, 0);
        // we need emergency break!
        if(result != -1)
            return result;        
        
        // calculate following speed
        return controllerOneLogic(vehAccess, vehAccess->precedingVeh.accelOmnet, speed, gap, predSpeed);
    }
    // ########################################################
    // Controller 2: CACC with acceleration from platoon leader
    // ######################################################## 
    else if(myCommunicationType == 2)
    {
        // check if we have received all the required data from platoon leader
        int result = allDataReceived(vehAccess, vehAccess->platoonLeaderVeh, speed, gap, predSpeed, predMaxDecel);        
        // switching to ACC
        if(result == -1)
            return MSCFModel_ACC::followSpeed(veh, speed, gap, predSpeed, predMaxDecel);
    
        if(veh->debug)
        {        
            char buffer1 [900];
            sprintf (buffer1, "--------\n%s\n--------\nController: %s", 
                              veh->getID().c_str(),
                              "CACC with acceleration from platoon leader");
            WRITE_MESSAGE(buffer1); 
        }
    
        // the gap in argument excludes minGap
        // we will add the minGap
        gap = gap + veh->getVehicleType().getMinGap();
        
        // apply measurement error
        if(veh->errorGap != 0 || veh->errorRelSpeed != 0)
            applyMeasurementError(vehAccess, &gap, speed, &predSpeed);        

        // printing input values (for debugging purposes)
        if(veh->debug)
        {
            char buffer [900];
            sprintf (buffer, "My parameters: speed=%.3f, accel=%.3f, gap=%.3f", 
                             speed, 
                             veh->getAcceleration(),
                             gap);
            WRITE_MESSAGE(buffer);
        
            char buffer2 [900];
            sprintf (buffer2, "Parameters of preceding vehicle %s: speed=%.3f, maxDecel=%.3f", 
                              veh->precedingVeh.vehiclePtrSumo->getID().c_str(),
                              predSpeed,
                              predMaxDecel);
            WRITE_MESSAGE(buffer2);
            
            char buffer3 [900];
            sprintf (buffer3, "Parameters of platoon leader %s: accel(wireless)=%.3f", 
                              veh->platoonLeaderVeh.vehicleNameOmnet.c_str(),
                              veh->platoonLeaderVeh.accelOmnet);
            WRITE_MESSAGE(buffer3);
        }
        
        result = emergencyBrakeNeeded(vehAccess, speed, gap, predSpeed, predMaxDecel, 0);
        // we need emergency break!
        if(result != -1)
            return result;        
        
        // calculate following speed
        return controllerOneLogic(vehAccess, vehAccess->platoonLeaderVeh.accelOmnet, speed, gap, predSpeed);
    }
    // ##############################################
    // Controller 3: CACC with bi-directional control
    // ############################################## 
    else if(myCommunicationType == 3)
    {
        // we need these two parameters for my following vehicle
        SUMOReal followSpeed = -1;
        SUMOReal followGap = -1;
        
        // get the following vehicle
        const MSVehicle* following = veh->followVeh.vehiclePtrSumo;
        // make sure, I have a follower
        if(following != NULL)
        {
            followSpeed = following->getSpeed();
            followGap = veh->getBackPositionOnLane(following->getLane()) - following->getPositionOnLane() - following->getVehicleType().getMinGap();     
        }
        
        // check if we have received all the required data from platoon leader
        int result = allDataReceived(vehAccess, vehAccess->platoonLeaderVeh, speed, gap, predSpeed, predMaxDecel);        
        // switching to ACC
        if(result == -1)
            return MSCFModel_ACC::followSpeed(veh, speed, gap, predSpeed, predMaxDecel);
    
        if(veh->debug)
        {        
            char buffer1 [900];
            sprintf (buffer1, "--------\n%s\n--------\nController: %s", 
                              veh->getID().c_str(),
                              "CACC with bi-directional control");
            WRITE_MESSAGE(buffer1); 
        }
    
        // the gap in argument excludes minGap
        // we will add the minGap
        gap = gap + veh->getVehicleType().getMinGap(); 
        
        // apply measurement error
        if(veh->errorGap != 0 || veh->errorRelSpeed != 0)
            applyMeasurementError(vehAccess, &gap, speed, &predSpeed); 

        // printing input values (for debugging purposes)
        if(veh->debug)
        {
            // my own parameters
            char buffer [900];
            sprintf (buffer, "My parameters: speed=%.3f, accel=%.3f, gap=%.3f", 
                             speed,
                             veh->getAcceleration(),
                             gap);
            WRITE_MESSAGE(buffer);
        
            // my preceding
            char buffer2 [900];
            sprintf (buffer2, "Parameters of preceding vehicle %s: speed=%.3f, maxDecel=%.3f", 
                              veh->precedingVeh.vehiclePtrSumo->getID().c_str(),
                              predSpeed,
                              predMaxDecel);
            WRITE_MESSAGE(buffer2);
            
            // my following
            if(veh->followVeh.vehiclePtrSumo != NULL)
            {
                char buffer3 [900];
                sprintf (buffer3, "Parameters of following vehicle %s: speed=%.3f, gap=%.3f", 
                                  veh->followVeh.vehiclePtrSumo->getID().c_str(),
                                  veh->followVeh.vehiclePtrSumo->getSpeed(),
                                  followGap);
                WRITE_MESSAGE(buffer3);
            }
            else
            {
                char buffer3 [900];
                sprintf (buffer3, "I have no following vehicles!");
                WRITE_MESSAGE(buffer3);
            }

            // platoon leader parameters
            char buffer4 [900];
            sprintf (buffer4, "Parameters of platoon leader %s: accel(wireless)=%.3f", 
                              veh->platoonLeaderVeh.vehicleNameOmnet.c_str(),
                              veh->platoonLeaderVeh.accelOmnet);
            WRITE_MESSAGE(buffer4);
        }
        
        result = emergencyBrakeNeeded(vehAccess, speed, gap, predSpeed, predMaxDecel, 0);
        // we need emergency break!
        if(result != -1)
            return result;        
        
        // calculate following speed
        return controllerTwoLogic(vehAccess, vehAccess->platoonLeaderVeh.accelOmnet, speed, gap, predSpeed, followSpeed, followGap);
    }
    else
    {
        throw ProcessError("Invalid communicationType for CACC vehicle! \n");
    }
}


int
MSCFModel_CACC::allDataReceived(MSVehicle* vehAccess, myNeighboringVehicle_t senderVeh, SUMOReal speed, SUMOReal gap, SUMOReal predSpeed, SUMOReal predMaxDecel) const
{
    // get current simulation time step
    SUMOTime simTime = MSNet::getInstance()->getCurrentTimeStep(); 
    
    // no data from senderVeh
    // Reason: maybe senderVeh is not CACC capable
    if(senderVeh.timeStampOmnet == -1)
    {
        if(vehAccess->debug)
        {
            char buffer [900];
            sprintf (buffer, "SimTime=%.2f: %s switched to ACC\n"
                             "Reason: followSpeed has not received any data from leading vehicle so far!\n"
                             "Degrade to ACC mode\n",
                             simTime/1000.,
                             vehAccess->getID().c_str());
            WRITE_WARNING(buffer);
        }
        
        vehAccess->myCFMode = Mode_NoData;
        return -1;        
    }
    // we receive acceleration from senderVeh (timeStamp is not -1), but the value is invalid!  
    // Reason: sth is wrong in OMNET++ or SUMO code!
    else if( isspace(senderVeh.accelOmnet) || senderVeh.accelOmnet == INFINITY )
    {        
        throw ProcessError("corrupted data from OMNET++");        
    }
    // the last received acceleration is old, and it is not safe to use it 
    // Reason: communication loss occurred!
    else if( fabs(simTime - senderVeh.timeStampOmnet)/1000. > 0.1 )
    {
        if(vehAccess->debug)
        {
            char buffer [900];
            sprintf (buffer, "%s has outdated a\n"
                             "Reason: missed X=%f beacons from leading vehicle.\n"
                             "latest a_p=%0.3f, received at=%.6f",
                             vehAccess->getID().c_str(),
                             (simTime - senderVeh.timeStampOmnet)/1000.,
                             senderVeh.accelOmnet,
                             (senderVeh.timeStampOmnet)/1000.);
            WRITE_WARNING(buffer);
        }
           
        vehAccess->myCFMode = Mode_DataLoss;
        
        // if degradeToACC is on, switch to ACC
        if(degradeToACC)
        {
            if(vehAccess->debug)
            {
                char buffer [900];
                sprintf (buffer, "Degrade to ACC mode\n");
                WRITE_WARNING(buffer);
            }
            
            return -1;
        }
        // otherwise continue, and keep using the old data!
        else
        {
            if(vehAccess->debug)
            {
                char buffer [900];
                sprintf (buffer, "Degrade to ACC is off. Continue using CACC controller.\n");
                WRITE_WARNING(buffer);
            }
            
            return 1;
        }
    }
    
    return 1;
}


void
MSCFModel_CACC::applyMeasurementError(MSVehicle* vehAccess, SUMOReal *gap, SUMOReal speed, SUMOReal *predSpeed) const
{
    if(vehAccess->errorGap != 0)
    {
        SUMOReal oldGap = *gap;
        
        // measurement error (gap)    
        double r1 = ( ( rand() / double(RAND_MAX) ) - 0.5 ) * 2;     // -1 <= r1 <= 1
        r1 = r1 * ( (vehAccess->errorGap)*100 );                     // -1 <= r1 <= 1    
        *gap = *gap * ( (100 + r1) / 100);
        
        if(vehAccess->debug)
        {        
            char buffer1 [900];
            sprintf (buffer1, "adding measurement error: errorGap=%.3f, oldGap=%.3f, newGap=%.3f",
                              vehAccess->errorGap,
                              oldGap,
                              *gap);
            WRITE_MESSAGE(buffer1); 
        }
    }
    
    if(vehAccess->errorRelSpeed != 0)
    {    
        SUMOReal oldPredSpeed = *predSpeed;
        
        // measurement error (relative speed)
        double r2 = ( ( rand() / double(RAND_MAX) ) - 0.5) * 2;     // -1 <= r2 <= 1
        r2 = r2 * ( (vehAccess->errorRelSpeed)*100 );               // -5 <= r2 <= 5    
        *predSpeed = ( (*predSpeed-speed) * ( (100 + r2) / 100 ) ) + speed; 
        
        if(vehAccess->debug)
        {        
            char buffer1 [900];
            sprintf (buffer1, "adding measurement error: errorRelSpeed=%.3f, oldPredSpeed=%.3f, predSpeed=%.3f",
                              vehAccess->errorRelSpeed, 
                              oldPredSpeed,
                              *predSpeed);
            WRITE_MESSAGE(buffer1); 
        }
    }    
}


int
MSCFModel_CACC::emergencyBrakeNeeded(MSVehicle* vehAccess, SUMOReal speed, SUMOReal gap, SUMOReal predSpeed, SUMOReal predMaxDecel, SUMOReal constant) const
{
    // check if we need to do an emergency break?
    SUMOReal g_safe = SPEED2DIST(speed) - pow(predSpeed,2)/(2*predMaxDecel) + pow(speed,2)/(2*myDecel) + ( vehAccess->getVehicleType().getMinGap() + constant ); 
      
    if(gap > g_safe)
        return -1;        
    else
    {
        SUMOReal followV = speed - ACCEL2SPEED(myDecel);
        
        // make sure followV is not negative
        followV = MAX2(0.,followV);
        
        // printing output values (for debugging purposes)
        if(vehAccess->debug)
        {
            char buffer [900];
            sprintf (buffer, "Output values: (emergency break) followV=%.3f", followV);
            WRITE_MESSAGE(buffer); 
        }
        
        // update the CFMode
        if(followV == 0)   
            vehAccess->myCFMode = Mode_Stopped;         
        else    
            vehAccess->myCFMode = Mode_EmergencyBrake;
              
        return followV;        
    }
}


SUMOReal
MSCFModel_CACC::controllerOneLogic(MSVehicle* vehAccess, SUMOReal receivedAccel, SUMOReal speed, SUMOReal gap, SUMOReal predSpeed) const 
{
    // V_int should not be bigger than 'maximum lane speed' or 'maximum vehicle speed'
    SUMOReal myV_int2 = MIN3( myV_int, vehAccess->getLane()->getVehicleMaxSpeed(vehAccess), vehAccess->getVehicleType().getMaxSpeed() );
    
    // desired accel for speed control
    SUMOReal a_des_v = myK_sc * (myV_int2 - speed);
    
    // desired accel for gap control
    SUMOReal a_des_g = myK_a * receivedAccel + 
                       myK_v * (predSpeed - speed) + 
                       myK_g * (gap - vehAccess->getVehicleType().getMinGap() - (speed*myHeadwayTime));
    
    SUMOReal a_control = MIN2(a_des_v, a_des_g);   
    
    // get current acceleration of vehicle
    SUMOReal a = vehAccess->getAcceleration(); 
        
    SUMOReal a_des = ( (a_control - a) / myDelay ) * TS + a;
    
    // bound a_des to [-ComfDecel, ComfAccel]
    SUMOReal a_veh = MAX2(-myComfDecel, MIN2(a_des,myComfAccel) );
    
    SUMOReal followV = speed + ACCEL2SPEED(a_veh);
    
    // make sure followV is not negative
    followV = MAX2(0.,followV);
    
    // printing output values (for debugging purposes)
    if(vehAccess->debug)
    {
        char buffer2 [900];
        sprintf (buffer2, "Output values: a_des_v=%.3f, a_des_g=%.3f, a_des=%.3f, a_veh=%.3f, followV=%.3f",
                          a_des_v,
                          a_des_g,
                          a_des,
                          a_veh,
                          followV);
        WRITE_MESSAGE(buffer2); 
    }
    
    if(followV == 0)   
        vehAccess->myCFMode = Mode_Stopped;         
    else if(a_des_g < a_des_v)   
        vehAccess->myCFMode = Mode_GapControl;               
    else    
        vehAccess->myCFMode = Mode_SpeedControl;   
    
    return followV;    
}


SUMOReal
MSCFModel_CACC::controllerTwoLogic(MSVehicle* vehAccess, SUMOReal receivedAccel, SUMOReal speed, SUMOReal gap, SUMOReal predSpeed, SUMOReal followSpeed, SUMOReal followGap) const
{
    // V_int should not be bigger than 'maximum lane speed' and 'maximum vehicle speed'
    SUMOReal myV_int2 = MIN3( myV_int, vehAccess->getLane()->getVehicleMaxSpeed(vehAccess), vehAccess->getVehicleType().getMaxSpeed() );
    
    // desired accel for speed control
    SUMOReal a_des_v = myK_sc * (myV_int2 - speed);
    
    // desired accel for gap control
    SUMOReal a_des_g;    
        
    // if we have a follower
    if(vehAccess->followVeh.vehiclePtrSumo != NULL)
    {
        a_des_g = myK_a * receivedAccel + 
                  myK_v * (predSpeed - speed) + 
                  myK_g * ( gap - vehAccess->getVehicleType().getMinGap() - (speed * myHeadwayTime) ) +
                  myK_v_f * (followSpeed - speed) -
                  myK_g_f * (followGap - vehAccess->getVehicleType().getMinGap() - (speed * myHeadwayTime) );        
    }
    else
    {
        a_des_g = myK_a * receivedAccel + 
                  myK_v * (predSpeed - speed) + 
                  myK_g * (gap - vehAccess->getVehicleType().getMinGap() - (speed * myHeadwayTime) );
    }
    
    SUMOReal a_control = MIN2(a_des_v, a_des_g);   
    
    // get current acceleration of vehicle
    SUMOReal a = vehAccess->getAcceleration(); 
    
    SUMOReal a_des = ( (a_control - a) / myDelay ) * TS + a;
    
    // bound a_des to [-ComfDecel, ComfAccel]
    SUMOReal a_veh = MAX2(-myComfDecel, MIN2(a_des,myComfAccel) );
    
    SUMOReal followV = speed + ACCEL2SPEED(a_veh);
    
    // make sure followV is not negative
    followV = MAX2(0.,followV);
    
    // printing output values (for debugging purposes)
    if(vehAccess->debug)
    {
        char buffer2 [900];
        sprintf (buffer2, "Output values: a_des_v=%.3f, a_des_g=%.3f, a_des=%.3f, a_veh=%.3f, followV=%.3f",  
                          a_des_v, 
                          a_des_g, 
                          a_des, 
                          a_veh, 
                          followV);
        WRITE_MESSAGE(buffer2); 
    }
    
    if(followV == 0)    
        vehAccess->myCFMode = Mode_Stopped;         
    else if(a_des_g < a_des_v)    
        vehAccess->myCFMode = Mode_GapControl;               
    else   
        vehAccess->myCFMode = Mode_SpeedControl;       
    
    return followV;    
}


MSCFModel*
MSCFModel_CACC::duplicate(const MSVehicleType* vtype) const 
{
    return new MSCFModel_CACC(vtype, myCommunicationType, myAccel, myDecel, myHeadwayTime, myDelay, myComfAccel, myComfDecel, myK_sc, myK_v, myK_g, myK_a, myV_int, myK_v_f, myK_g_f, false);
}
