
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
#include "MSCFModel_ACC.h"
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <utils/common/RandHelper.h>

// mani
#include <math.h>
#include <microsim/MSNet.h> // for getting the current simulation time


// ===========================================================================
// method definitions
// ===========================================================================
MSCFModel_ACC::MSCFModel_ACC(const MSVehicleType* vtype,
        SUMOReal MaxAccel,
        SUMOReal MaxDecel,
        SUMOReal tau,
        SUMOReal delay,
        SUMOReal ComfAccel,
        SUMOReal ComfDecel,
        SUMOReal K_sc,
        SUMOReal K_v,
        SUMOReal K_d,
        SUMOReal V_int) : MSCFModel(vtype, MaxAccel, MaxDecel, tau)
{
    this->myDelay = delay;
    this->myComfAccel = ComfAccel;
    this->myComfDecel = ComfDecel;
    this->myK_sc = K_sc;
    this->myK_v = K_v;
    this->myK_g = K_d;
    this->myV_int = V_int;
}


MSCFModel_ACC::~MSCFModel_ACC() {}


SUMOReal
MSCFModel_ACC::moveHelper(MSVehicle* const veh, SUMOReal vPos) const 
{
    const SUMOReal oldV = veh->getSpeed(); // save old v for optional acceleration computation
    const SUMOReal vSafe = MIN2(vPos, veh->processNextStop(vPos)); // process stops
    // we need the acceleration for emission computation;
    //  in this case, we neglect dawdling, nonetheless, using
    //  vSafe does not incorporate speed reduction due to interaction
    //  on lane changing
    const SUMOReal vMin = getSpeedAfterMaxDecel(oldV);
    // do not exceed max decel even if it is unsafe
    SUMOReal vMax = MAX2(vMin,
                         MIN3(veh->getLane()->getVehicleMaxSpeed(veh), maxNextSpeed(oldV, veh), vSafe));
#ifdef _DEBUG
    //if (vMin > vMax) {
    //    WRITE_WARNING("Vehicle's '" + veh->getID() + "' maximum speed is lower than the minimum speed (min: " + toString(vMin) + ", max: " + toString(vMax) + ").");
    //}
#endif
    return veh->getLaneChangeModel().patchSpeed(vMin, MAX2(vMin, vMax), vMax, *this);
}


SUMOReal
MSCFModel_ACC::followSpeed(const MSVehicle* const veh, SUMOReal speed, SUMOReal gap, SUMOReal predSpeed, SUMOReal predMaxDecel) const 
{
    // note: when this method is called, it means we definitely have a preceding car.             
    
    // get current simulation time step
    SUMOTime simTime = MSNet::getInstance()->getCurrentTimeStep();
    
    // modify vehicle parameters through vehAccess
    SUMOVehicle* sumoVehicle = MSNet::getInstance()->getVehicleControl().getVehicle(veh->getID());
    MSVehicle* vehAccess = dynamic_cast<MSVehicle*>(sumoVehicle);

    // the gap in argument excludes minGap
    // we will add the minGap
    gap = gap + veh->getVehicleType().getMinGap();
    
    // measurement error (gap)    
    double r1 = ( ( rand() / double(RAND_MAX) ) - 0.5 ) * 2;     // -1 <= r1 <= 1
    r1 = r1 * ( (veh->errorGap)*100 );                           // -1 <= r1 <= 1    
    gap = gap * ( (100 + r1) / 100);
    
    // measurement error (relative speed)
    double r2 = ( ( rand() / double(RAND_MAX) ) - 0.5) * 2;     // -1 <= r2 <= 1
    r2 = r2 * ( (veh->errorRelSpeed)*100 );                     // -5 <= r2 <= 5    
    predSpeed = ( (predSpeed-speed) * ( (100 + r2) / 100 ) ) + speed;    
    
    SUMOReal g_safe = SPEED2DIST(speed) - pow(predSpeed,2)/(2*predMaxDecel) + pow(speed,2)/(2*myDecel) + 2.; 
    
    // should use MaxDecel
    if(gap <= g_safe)
    {                
        SUMOReal followV = speed - ACCEL2SPEED(myDecel);
        
        // make sure followV is not negative
        followV = MAX2(0.,followV);
        
        // update the CFMode
        if(followV == 0)  
            vehAccess->myCFMode = Mode_Stopped;         
        else   
            vehAccess->myCFMode = Mode_EmergencyBrake;
        
        return followV;        
    }
    
    // V_int should not be bigger than 'maximum lane speed' and 'maximum vehicle speed'
    SUMOReal myV_int2 = MIN3( myV_int, veh->getLane()->getVehicleMaxSpeed(veh), veh->getVehicleType().getMaxSpeed() );
    
    // desired accel for speed control
    SUMOReal a_des_v = myK_sc * (myV_int2 - speed);
    
    // desired accel for gap control
    SUMOReal a_des_g = myK_v * (predSpeed - speed) + 
                       myK_g * (gap - veh->getVehicleType().getMinGap() - (speed*myHeadwayTime));
    
    SUMOReal a_control = MIN2(a_des_v, a_des_g);   
    
    // get current acceleration of vehicle
    SUMOReal a = veh->getAcceleration();
    
    SUMOReal a_des = ( (a_control - a) / myDelay ) * TS + a;
    
    // bound a_des to [-ComfDecel, ComfAccel]
    SUMOReal a_veh = MAX2(-myComfDecel, MIN2(a_des,myComfAccel) );
    
    SUMOReal followV = speed + ACCEL2SPEED(a_veh);
    
    // make sure followV is not negative
    followV = MAX2(0.,followV);
    
    if(followV == 0)    
        vehAccess->myCFMode = Mode_Stopped;         
    else if(a_des_g < a_des_v)   
        vehAccess->myCFMode = Mode_GapControl;      
    else   
        vehAccess->myCFMode = Mode_SpeedControl;       

    return followV;    
}

SUMOReal
MSCFModel_ACC::stopSpeed(const MSVehicle* const veh, const SUMOReal speed, SUMOReal gap) const 
{
    return MIN2(maximumSafeStopSpeed(gap, speed, false, TS), maxNextSpeed(speed, veh));
}


MSCFModel*
MSCFModel_ACC::duplicate(const MSVehicleType* vtype) const 
{
    return new MSCFModel_ACC(vtype, myAccel, myDecel, myHeadwayTime, myDelay, myComfAccel, myComfDecel, myK_sc, myK_v, myK_g, myV_int);
}
