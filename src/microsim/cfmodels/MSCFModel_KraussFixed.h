
#ifndef MSCFModel_KraussFixed_h
#define	MSCFModel_KraussFixed_h

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MSCFModel.h"
#include <utils/xml/SUMOXMLDefinitions.h>


// ===========================================================================
// class definitions
// ===========================================================================
/** @class MSCFModel_Krauss
 * @brief Krauss car-following model, with acceleration decrease and faster start
 * @see MSCFModel
 */
class MSCFModel_KraussFixed : public MSCFModel 
{
public:
    /** @brief Constructor
     * @param[in] accel The maximum acceleration
     * @param[in] decel The maximum deceleration
     * @param[in] dawdle The driver imperfection
     * @param[in] headwayTime The driver's reaction time
     */
    MSCFModel_KraussFixed(const MSVehicleType* vtype, SUMOReal MaxAccel, SUMOReal MaxDecel, SUMOReal sigma, SUMOReal T_d, SUMOReal tau);


    /// @brief Destructor
    ~MSCFModel_KraussFixed();
    
     SUMOReal moveHelper(MSVehicle* const veh, SUMOReal vPos) const;

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


    /** @brief Computes the vehicle's safe speed for approaching a non-moving obstacle (no dawdling)
     * @param[in] veh The vehicle (EGO)
     * @param[in] gap2pred The (netto) distance to the the obstacle
     * @return EGO's safe speed for approaching a non-moving obstacle
     * @see MSCFModel::ffeS
     * @todo generic Interface, models can call for the values they need
     */
    virtual SUMOReal stopSpeed(const MSVehicle* const veh, const SUMOReal speed, SUMOReal gap2pred) const;


    /** @brief Returns the model's name
     * @return The model's name
     * @see MSCFModel::getModelName
     */
    virtual int getModelID() const {
        return SUMO_TAG_CF_KRAUSSFIXED;
    }
    /// @}
    
    SUMOReal getImperfection() const {
        return myDawdle;
    }
    

    // mani
    virtual int getCFModelNumber() const
    {
        return SUMO_CF_KRAUSSFIXED;
    }


        /// @name Setter methods
    /// @{
    /** @brief Sets a new value for maximum deceleration [m/s^2]
     * @param[in] accel The new deceleration in m/s^2
     */
    void setMaxDecel(SUMOReal decel) {
        myDecel = decel;
    }


    /** @brief Sets a new value for driver imperfection
     * @param[in] accel The new driver imperfection
     */
    void setImperfection(SUMOReal imperfection) {
        myDawdle = imperfection;
    }


    /** @brief Sets a new value for driver reaction time [s]
     * @param[in] headwayTime The new driver reaction time (in s)
     */
    void setHeadwayTime(SUMOReal headwayTime) {
        myHeadwayTime = headwayTime;
    }
    /// @}

    /** @brief Duplicates the car-following model
     * @param[in] vtype The vehicle type this model belongs to (1:1)
     * @return A duplicate of this car-following model
     */
    virtual MSCFModel* duplicate(const MSVehicleType* vtype) const;


private:
    /** @brief Returns the "safe" velocity
     * @param[in] gap2pred The (netto) distance to the LEADER
     * @param[in] predSpeed The LEADER's speed
     * @return the safe velocity
     */
    SUMOReal _vsafe(SUMOReal gap, SUMOReal predSpeed, SUMOReal predMaxDecel) const;


    /** @brief Applies driver imperfection (dawdling / sigma)
     * @param[in] speed The speed with no dawdling
     * @return The speed after dawdling
     */
    SUMOReal dawdle(SUMOReal speed) const;
    
    
    SUMOReal myDawdle;  
    SUMOReal myDelay;
};

#endif	/* MSCFMODEL_KRAUSS_H */
