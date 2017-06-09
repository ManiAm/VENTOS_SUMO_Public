
#ifndef MSCFModel_CACC_h
#define	MSCFModel_CACC_h

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MSCFModel_ACC.h"
#include <utils/xml/SUMOXMLDefinitions.h>

// ===========================================================================
// class definitions
// ===========================================================================

/** @class MSCFModel_CACC
 * @brief The original Krauss (1998) car-following model and parameter
 * @see MSCFModel
 */
class MSCFModel_CACC : public MSCFModel_ACC 
{
public:
    /** @brief Constructor
     * @param[in] accel The maximum acceleration
     * @param[in] decel The maximum deceleration
     * @param[in] dawdle The driver imperfection
     * @param[in] tau The driver's reaction time
     */
    MSCFModel_CACC(const MSVehicleType*, int, SUMOReal, SUMOReal, SUMOReal, SUMOReal, SUMOReal, SUMOReal, SUMOReal, SUMOReal, SUMOReal, SUMOReal, SUMOReal, SUMOReal, SUMOReal, bool, SUMOReal);
    
    /// @brief Destructor
    ~MSCFModel_CACC();

    /// @name Implementations of the MSCFModel interface
    /// @{

    /** @brief Computes the vehicle's safe speed (no dawdling)
     * @param[in] veh The vehicle (EGO)
     * @param[in] speed The vehicle's speed
     * @param[in] gap2pred The (netto) distance to the LEADER
     * @param[in] predSpeed The speed of LEADER
     * @return EGO's safe speed
     * @see MSCFModel::ffeV
     */
    virtual SUMOReal followSpeed(const MSVehicle* const veh, SUMOReal speed, SUMOReal gap2pred, SUMOReal predSpeed, SUMOReal predMaxDecel) const;
    
    double controller_1(MSVehicle* vehAccess, SUMOReal, SUMOReal, SUMOReal, SUMOReal) const;
    double controller_2(MSVehicle* vehAccess, SUMOReal, SUMOReal, SUMOReal, SUMOReal) const;
    double controller_3(MSVehicle* vehAccess, SUMOReal, SUMOReal, SUMOReal, SUMOReal) const;

    int checkPlatoonConfigTimestamp(MSVehicle*, platoonConfig_t &) const;
    void applyMeasurementError(MSVehicle*, SUMOReal *, SUMOReal, SUMOReal *) const;
    int emergencyBrakeNeeded(MSVehicle*, SUMOReal, SUMOReal, SUMOReal, SUMOReal, SUMOReal) const; 
    double switchToACC(MSVehicle*, SUMOReal, SUMOReal, SUMOReal, SUMOReal) const;
    
    /** @brief Returns the model's name
     * @return The model's name
     * @see MSCFModel::getModelName
     */
    virtual int getModelID() const 
    {
        return SUMO_TAG_CF_CACC;
    }

    virtual int getCommunicationType() const
    {
        return myStrategy;
    }

    /// @}
    
    /** @brief Duplicates the car-following model
     * @param[in] vtype The vehicle type this model belongs to (1:1)
     * @return A duplicate of this car-following model
     */
    virtual MSCFModel* duplicate(const MSVehicleType* vtype) const;
    
    virtual int getCFModelNumber() const
    {
        return SUMO_CF_CACC;
    }

private:
    
    int myStrategy;
    bool degradeToACC;
    SUMOReal invalidTimer;

    SUMOReal myK_a;
    
    // used in bi-directional control
    SUMOReal myK_v_f;
    SUMOReal myK_g_f;
};
     
#endif	/* MSCFModel_CACC_H */
