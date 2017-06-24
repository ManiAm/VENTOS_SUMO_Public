
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <microsim/MSVehicle.h>
#include <microsim/MSLane.h>
#include "MSCFModel_KraussFixed.h"
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <utils/common/RandHelper.h>

// mani
#include <math.h>


// ===========================================================================
// method definitions
// ===========================================================================
MSCFModel_KraussFixed::MSCFModel_KraussFixed(const MSVehicleType* vtype,
        SUMOReal MaxAccel,
        SUMOReal MaxDecel,
        SUMOReal tau,
        SUMOReal sigma,
        SUMOReal delay) : MSCFModel(vtype, MaxAccel, MaxDecel, tau)
{
    this->myDawdle = sigma;
    this->myDelay = delay;
}


MSCFModel_KraussFixed::~MSCFModel_KraussFixed() {}


SUMOReal
MSCFModel_KraussFixed::moveHelper(MSVehicle* const veh, SUMOReal vPos) const 
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
    return veh->getLaneChangeModel().patchSpeed(vMin, MAX2(vMin, dawdle(vMax)), vMax, *this);
}


SUMOReal
MSCFModel_KraussFixed::followSpeed(const MSVehicle* const veh, SUMOReal speed, SUMOReal gap, SUMOReal predSpeed, SUMOReal predMaxDecel) const 
{
    // calculate safe speed
    SUMOReal v_safe = _vsafe(gap, predSpeed, predMaxDecel);
    
    // bound v_safe into [0,V_max]
    SUMOReal v_des = MAX2( 0., MIN2( v_safe, myType->getMaxSpeed() ) );
    
    // calculate desired accel (note that a_des can be negative!)
    SUMOReal a_des = ( v_des - veh->getSpeed() ) / myDelay;
    
    // bound a_des into [-MaxDecel,MaxAccel]
    SUMOReal a_veh = MAX2( -myDecel , MIN2(a_des, myAccel) );    
    
    SUMOReal followV = veh->getSpeed() + ACCEL2SPEED(a_veh);
    
    // make sure followV is not negative
    followV = MAX2(0.,followV);
    
    return followV;
}


SUMOReal
MSCFModel_KraussFixed::stopSpeed(const MSVehicle* const veh, const SUMOReal speed, SUMOReal gap) const 
{ 
    return MIN2(_vsafe(gap, 0, 0), maxNextSpeed(speed, veh));
}


SUMOReal
MSCFModel_KraussFixed::dawdle(SUMOReal speed) const 
{
    // generate random number out of [0,1]
    SUMOReal random = RandHelper::rand();
    // Dawdle.
    if (speed < myAccel) 
    {
        // we should not prevent vehicles from driving just due to dawdling
        //  if someone is starting, he should definitely start
        // (but what about slow-to-start?)!!!
        speed -= ACCEL2SPEED(myDawdle * speed * random);
    } else 
    {
        speed -= ACCEL2SPEED(myDawdle * myAccel * random);
    }
    
    return MAX2(SUMOReal(0), speed);
}


/** Returns the SK-vsafe. */
SUMOReal
MSCFModel_KraussFixed::_vsafe(SUMOReal gap, SUMOReal predSpeed, SUMOReal predMaxDecel) const 
{    
    const SUMOReal b = MIN2(myDecel, predMaxDecel);    
    SUMOReal Gmin = myType->getMinGap();
    SUMOReal v_safe = (-1. * b * myHeadwayTime) + sqrt( pow(b * myHeadwayTime,2) + (b/predMaxDecel)*pow(predSpeed,2) + 2*b*(gap-Gmin) );
    
    return v_safe;
    
}

MSCFModel*
MSCFModel_KraussFixed::duplicate(const MSVehicleType* vtype) const 
{
    return new MSCFModel_KraussFixed(vtype, myAccel, myDecel, myHeadwayTime, myDawdle, myDelay);
}


//void MSCFModel::saveState(std::ostream &os) {}

