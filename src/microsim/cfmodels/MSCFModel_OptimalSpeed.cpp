
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
#include "MSCFModel_OptimalSpeed.h"
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <utils/common/RandHelper.h>

// mani
#include <string.h>


// ===========================================================================
// method definitions
// ===========================================================================
MSCFModel_OptimalSpeed::MSCFModel_OptimalSpeed(const MSVehicleType* vtype,
        SUMOReal accel,
        SUMOReal decel,
        SUMOReal tau,
        SUMOReal dawdle,
        SUMOReal sens) : MSCFModel(vtype, accel, decel, tau)
{
    this->mySensitivity = sens;
    this->myDawdle = dawdle;
}


MSCFModel_OptimalSpeed::~MSCFModel_OptimalSpeed() {}


SUMOReal
MSCFModel_OptimalSpeed::moveHelper(MSVehicle* const veh, SUMOReal vPos) const {  
    
    const SUMOReal oldV = veh->getSpeed(); // save old v for optional acceleration computation
    const SUMOReal vSafe = MIN2(vPos, veh->processNextStop(vPos)); // process stops
    // we need the acceleration for emission computation;
    //  in this case, we neglect dawdling, nonetheless, using
    //  vSafe does not incorporate speed reduction due to interaction
    //  on lane changing
    const SUMOReal vMin = getSpeedAfterMaxDecel(oldV);
    // do not exceed max decel even if it is unsafe
    SUMOReal vMax = MAX2(vMin,MIN3(veh->getLane()->getVehicleMaxSpeed(veh), maxNextSpeed(oldV, veh), vSafe));

#ifdef _DEBUG
    //if (vMin > vMax) {
    //    WRITE_WARNING("Vehicle's '" + veh->getID() + "' maximum speed is lower than the minimum speed (min: " + toString(vMin) + ", max: " + toString(vMax) + ").");
    //}
#endif
    
    return veh->getLaneChangeModel().patchSpeed(vMin, MAX2(vMin, dawdle(vMax)), vMax, *this);
}


SUMOReal
MSCFModel_OptimalSpeed::followSpeed(const MSVehicle* const veh, SUMOReal speed, SUMOReal gap, SUMOReal /*predSpeed*/, SUMOReal /*predMaxDecel*/) const {
    
    // note that gap = g - minGap (g is the actual distance between two vehicles)
    
    SUMOReal v_des = MAX2(0., MIN2(gap/myHeadwayTime,myType->getMaxSpeed()) );
    SUMOReal acceleration = (v_des - speed) * mySensitivity;    
    SUMOReal follow = speed + ACCEL2SPEED(acceleration);  
    
    if(follow > 0)
        return follow;
    else     // prevent vehicle from going backward!
        return 0;
}


SUMOReal
MSCFModel_OptimalSpeed::stopSpeed(const MSVehicle* const veh, const SUMOReal speed, SUMOReal gap) const {
    return MIN2(20., maxNextSpeed(veh->getSpeed(), veh));
}


SUMOReal
MSCFModel_OptimalSpeed::dawdle(SUMOReal speed) const {
    return MAX2(SUMOReal(0), speed - ACCEL2SPEED(myDawdle * myAccel * RandHelper::rand()));
}


MSCFModel*
MSCFModel_OptimalSpeed::duplicate(const MSVehicleType* vtype) const {
    return new MSCFModel_OptimalSpeed(vtype, myAccel, myDecel, myHeadwayTime, myDawdle, mySensitivity);
}


//void MSCFModel::saveState(std::ostream &os) {}
