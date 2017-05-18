
#ifndef MSCFModel_ACC_h
#define	MSCFModel_ACC_h

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
/** @class MSCFModel_ACC
 * @brief The original Krauss (1998) car-following model and parameter
 * @see MSCFModel
 */
class MSCFModel_ACC : public MSCFModel 
{
public:
    /** @brief Constructor
     * @param[in] accel The maximum acceleration
     * @param[in] decel The maximum deceleration
     * @param[in] dawdle The driver imperfection
     * @param[in] tau The driver's reaction time
     */
    MSCFModel_ACC(const MSVehicleType* vtype, SUMOReal MaxAccel, SUMOReal MaxDecel, SUMOReal T_d, SUMOReal tau, SUMOReal ComfAccel, SUMOReal ComfDecel, SUMOReal K_sc, SUMOReal K_v, SUMOReal K_d, SUMOReal V_int);

    /// @brief Destructor
    ~MSCFModel_ACC();


    /// @name Implementations of the MSCFModel interface
    /// @{

    /** @brief Applies interaction with stops and lane changing model influences
     * @param[in] veh The ego vehicle
     * @param[in] vPos The possible velocity
     * @return The velocity after applying interactions with stops and lane change model influences
     */
    SUMOReal moveHelper(MSVehicle* const veh, SUMOReal vPos) const;


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
    virtual int getModelID() const 
    {
        return SUMO_TAG_CF_ACC;
    }

    /// @}


    /** @brief Duplicates the car-following model
     * @param[in] vtype The vehicle type this model belongs to (1:1)
     * @return A duplicate of this car-following model
     */
    virtual MSCFModel* duplicate(const MSVehicleType* vtype) const;
    
    
    virtual int getCFModelNumber() const
    {
        return SUMO_CF_ACC;
    }
    
    virtual int setDelay(SUMOReal d)
    {
        myDelay = d;
    }
    
    virtual void setVint(SUMOReal v)
    {
        myV_int = v;        
    }
    
    virtual void setComfAccel(SUMOReal ca)
    {
        myComfAccel = ca;        
    }
    
    virtual void setComfDecel(SUMOReal cd) 
    {
        myComfDecel = cd;       
    }
    
    virtual void setK_v(SUMOReal cg) 
    {
        myK_v = cg;       
    }
        
    virtual void setK_g(SUMOReal cg) 
    {
        myK_g = cg;        
    }
    
protected:    
    SUMOReal myDelay;
    
    SUMOReal myV_int; 
    SUMOReal myComfAccel;
    SUMOReal myComfDecel; 
        
    SUMOReal myK_sc;
    SUMOReal myK_v;
    SUMOReal myK_g;   
};

#endif	/* MSCFModel_ACC_H */

