
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Glider model for high altitude balloon drop
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_GLIDER_ENABLED

#include "SIM_Aircraft.h"
#include <AP_Param/AP_Param.h>

namespace SITL {

/*
  a very simple plane simulator
 */
class Glider : public Aircraft {
public:
    Glider(const char *frame_str);

    /* update model by one time step */
    virtual void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new Glider(frame_str);
    }

    bool on_ground() const override;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    float alpharad;
    float betarad;

    AP_Float balloon_burst_amsl;
    AP_Float balloon_rate;

    /*
      parameters that define the glider model
     */
    const struct Model {
        // total vehicle mass
        float mass = 4.595; // kg

        // reference area
        float Sref = 0.089; // m^2
        float refSpan = 1.04; // m
        float refChord = 0.175; // m

        float IXX = 0.01611; // kg-m^2
        float IYY = 0.389; // kg-m^2
        float IZZ = 0.4069; // kg-m^2

        // CN is the coefficients for forces on +Z axis
        float CN0 = 0.12828;
        float CN0s = -0.0020832;
        float CN1 = 6.8696;
        float CN1s = -0.0626288;

        // CA is the coefficients for forces on +X axis
        float CA0 = 0.03305;
        float CA0s = -0.000101;
        float CA1 = 0.19234;
        float CA1s = -0.0027254;

        // CY is the coefficients for forces on the +Y axis
        float CY0 = -1.51541;
        float CY0s = 0.0000766;

        // ------------------------------------------------------------------------------

        // Cl is the coefficients for moments on X axis
        float Clb = -0.01334;
        float Clbs = -0.0000196;

        // Cm is the coefficients for moments on Y axis
        float Cma = -0.82931;
        float Cmas = -0.036174;

        // Cm0 is the zero-alpha moment coefficient
        float Cm0 = 0.05315;
        float Cm0s = -0.002287; 

        // Cn is the coefficients for moments on Z axis
        float Cnb = 0.77834;
        float Cnbs = -0.0001104;

        // ------------------------------------------------------------------------------

        // controls neutral dynamic derivatives
        float Cmq = -33.27537;
        float Cmqs = 0.272671;

        float Clp = -0.62372;
        float Clps = 0.0104838;

        float Clr = 0.04076;
        float Clrs = -0.0000606;

        float Cnp = -0.03912;
        float Cnps = -0.0003038;

        float Cnr = -0.89731;
        float Cnrs = 0.0003424;

        // ------------------------------------------------------------------------------

        // elevator
        float elevatorDeflectionLimitDeg = -12.5;

        float CNe = -0.50982; 
        float CNes = -0.0001564;

        float CAe = 0.01035;
        float CAes = -0.0001742;

        float Cmde = 1.66008; 
        float Cmdes = -0.008633;

        // rudder
        float rudderDeflectionLimitDeg = 18.0;

        float Cldr = 0.01246;
        float Cldrs = 0.0001178;

        float Cndr = -0.28013;
        float Cndrs = 0.0000516;

        // aileron
        float aileronDeflectionLimitDeg = 15.5;

        float Clda = 0.03676;
        float Cldas = 0.0000024;

        float Cnda = 0.00514; //-0.00514;
        float Cndas = 0.0000816; //-0.0000816

        float swingWingDeflectionLimitDeg = 75.0; // starting at 10 deg, +75 is 85 degrees total

        // Forces in the +X direction are –CA * q * Sref
        // Forces in the +Y direction are  +CY * q * Sref
        // Forces in the +Z direction are  –CN * q *Sref
        // Moments about the X axis are +Cl * q * Sref * RefSpan
        // Moments about the Y axis are +Cm * q * Sref * RefChord
        // Moments about the Z axis are +Cn * q * Sref * RefSpan

        // low altitude
        float alphaRadMax = 0.209;
        float betaRadMax = 0.209;

        // balloon launch parameters
        float tetherLength = 50.0f;       // length of tether from balloon to aircraft (m)
        float tetherPogoFreq = 2.0f;      // measured vertical frequency of on tether (Hz)

    } model;

    Vector3f getForce(float inputAileron, float inputElevator, float inputRudder, float wing_pos_deg);
    Vector3f getTorque(float inputAileron, float inputElevator, float inputRudder, float wing_pos_deg, const Vector3f &force) const;
    bool update_balloon(float balloon, Vector3f &force, Vector3f &rot_accel);
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);

    Vector3f balloon_velocity;           // balloon velocity NED
    Vector3f balloon_position{0.0f, 0.0f, -45.0f}; // balloon position NED from origin

    enum class carriageState {
        NONE = 0, // no carriage option available
        WAITING_FOR_PICKUP = 1, // in launch cradle waiting to be picked up by launch vehicle
        WAITING_FOR_RELEASE = 2, // being carried by luanch vehicle waitng to be released
        PRE_RELEASE = 3, // had been released by launch vehicle
        RELEASED = 4 // had been released by launch vehicle
    } carriage_state;
    bool plane_air_release;    // true when plane has separated from the airborne launching platform

    uint32_t last_drag_ms;
    float sim_LD;
};

} // namespace SITL

#endif // AP_SIM_GLIDER_ENABLED
