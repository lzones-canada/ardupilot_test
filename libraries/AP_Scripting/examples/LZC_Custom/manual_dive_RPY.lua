-- Constants
local NAV_SCRIPT_TIME = 42702 -- Mission command for SCRIPTING_TIME
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local FLIGHT_MODE = {FLY_BY_WIRE_A=5,AUTO=10,RTL=11} -- Flight modes
local UPDATE_RATE_MS = 25 -- Update rate in milliseconds

-- Plane Constants
local DIVE_5K = 5000 -- 10k dive in ft
local AS_MAX = 190 -- Max airspeed for dive
local WA_DIVE = 30 -- Wing angle for dive
local DIVE_ANG_DEF = -75.0 -- Default dive angle
local PULLUP_PITCH = -10.0 -- Target pitch for pullup
local RLL2SRV_TCONST = Parameter("RLL2SRV_TCONST")
local PITCH_TCONST = Parameter("PTCH2SRV_TCONST")
local ROLL_LIMIT = Parameter("ROLL_LIMIT_DEG")

-- Parameters
local final_dive_altitude = 0 -- Final altitude for dive
local dive_angle_deg = 0 -- Target pitch for nose-down dive
local stage = 0  -- Tracks the current stage of the dive sequence
-- Target yaw (keep current heading)
local target_yaw = 0
local mission_idx = 0
local mission_cnt = 0


-- Function to set wing angle -- SITL
local SWEEP_WING_SRV_CHANNEL = 13 -- Servo channel for sweep wing
local function set_wing_angle(angle)
    -- convert degrees to PWM
    local sweep_pwm = (angle + 102) * 10
    SRV_RelayEvent:do_set_servo(SWEEP_WING_SRV_CHANNEL, sweep_pwm)
end

-- Function to set wing angle -- HITL
-- local function set_wing_angle(angle)
--     -- Round the value
--     local angle_round = math.floor(angle + 0.5)
--     volz:set_target_command(angle_round)
-- end

local function set_rate_targets(throttle, roll_rate, pitch_rate, yaw_rate)
    -- Don't want a rudder offset or yaw rate ??
    vehicle:set_rudder_offset(0, false)
    vehicle:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, yaw_rate)
 end

 local function wrap_360(angle)
    local res = math.fmod(angle, 360.0)
     if res < 0 then
         res = res + 360.0
     end
     return res
 end

 local function wrap_180(angle) 
    local res = wrap_360(angle)
    if res > 180 then
       res = res - 360
    end
    return res
end

-- roll angle error 180 wrap to cope with errors while in inverted segments
local function roll_angle_error_wrap(roll_angle_error)
    if math.abs(roll_angle_error) > 180 then
     if roll_angle_error > 0 then
        roll_angle_error = roll_angle_error - 360
     else 
        roll_angle_error= roll_angle_error +360
     end 
    end
    return roll_angle_error
 end

--roll controller to keep wings level in earth frame. if arg is 0 then level is at only 0 deg, otherwise its at 180/-180 roll also for loops
local function earth_frame_wings_level(arg)
    local roll_deg = math.deg(ahrs:get_roll())
    local roll_angle_error = 0.0
    if (roll_deg > 90 or roll_deg < -90) and arg ~= 0 then
     roll_angle_error = 180 - roll_deg
    else
     roll_angle_error = - roll_deg
    end
    return roll_angle_error_wrap(roll_angle_error)/(RLL2SRV_TCONST:get())
 end

 -- a controller to target a zero pitch angle and zero heading change, used in a roll
-- output is a body frame pitch rate, with convergence over time tconst in seconds
local function pitch_controller(target_pitch_deg, target_yaw_deg, tconst)
    local roll_deg = math.deg(ahrs:get_roll())
    local pitch_deg = math.deg(ahrs:get_pitch())
    local yaw_deg = math.deg(ahrs:get_yaw())
 
    -- get earth frame pitch and yaw rates
    local ef_pitch_rate = (target_pitch_deg - pitch_deg) / tconst
    local ef_yaw_rate = wrap_180(target_yaw_deg - yaw_deg) / tconst
 
    local bf_pitch_rate = math.sin(math.rad(roll_deg)) * ef_yaw_rate + math.cos(math.rad(roll_deg)) * ef_pitch_rate
    local bf_yaw_rate   = math.cos(math.rad(roll_deg)) * ef_yaw_rate - math.sin(math.rad(roll_deg)) * ef_pitch_rate

    -- gcs:send_text(MAV_SEVERITY.INFO, string.format("PitchRate: %.1f Pitch: %.1f", bf_pitch_rate, pitch_deg))

    return bf_pitch_rate, bf_yaw_rate
 end


-- Update function that checks the current mode and script commands
function update()

    -- Track the airspeed
    local airspeed = ahrs:airspeed_estimate()

    -- Get the current mission command and argument needed for Dive
    local id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
     -- Stage 0: Init Dive Setup
    if id and stage == 0 then
        -- Dive Angle in Degrees (arg1)
        if arg1 ~= 0 then
            dive_angle_deg = arg1 -- arg1 - Dive angle in Degrees.
        else 
            dive_angle_deg = DIVE_ANG_DEF -- Default dive angle
        end
        -- Dive Alt in ft (arg2)
        if arg2 ~= 0 then
            final_dive_altitude = (arg2 / 3.28084) -- arg2 - Altitude ft to meters
        else 
            final_dive_altitude = baro:get_altitude() - (DIVE_5K / 3.28084) -- 10,000 ft off current altitude
        end
        gcs:send_text(MAV_SEVERITY.INFO, string.format("DiveInit - AoA: %.1f deg Alt: %.1f ft.", dive_angle_deg, (final_dive_altitude * 3.28084)))

        -- Save number of total mission commands
        mission_cnt = mission:num_commands()

        -- Set the PUP_HEADING parameter to current heading
        --local curr_heading = wrap_360(math.deg(ahrs:get_yaw()))
        --curr_heading = math.floor(curr_heading + 0.5)
        --gcs:send_text(MAV_SEVERITY.INFO, string.format("Current heading: %.1f", curr_heading))

        -- Disable WAIT_ROLL on pullup..
        param:set("PUP_HEADING", -1)
        -- Set the pullup pitch parameter
        param:set("PUP_PITCH_START", -60)
        stage = stage + 1
    end

    -- Stage 1: Moves wings to dive position
    if (stage == 1) and (baro:get_altitude() > final_dive_altitude) then
        mission_idx = mission:get_current_nav_index() -- Save the current mission index
        param:set("SCR_USER1", 0) -- Turn off AUTO wings
        set_wing_angle(WA_DIVE) -- Set wings to DIVE

        -- Check the current sweep angle
        local current_wing_angle = volz:get_sweep_angle()

        -- Move to the next stage once wings have reached the desired angle
        if math.abs(current_wing_angle - WA_DIVE) <= 1.0 then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("Wings reached %.1f degrees", current_wing_angle))
            -- Target yaw (None)
            target_yaw = 0 -- math.deg(ahrs:get_yaw())
            stage = stage + 1
        end
    end

    -- Stage 2: Dive
    if stage == 2 then
        -- Keep the wings level in earth frame
        local roll_rate = earth_frame_wings_level(1)
        -- Calculate pitch rate to keep the nose down
        local pitch_rate, yaw_rate = pitch_controller(dive_angle_deg, target_yaw, PITCH_TCONST:get())

        -- Set the rate targets -- zero throttle / zero yaw
        set_rate_targets(0, roll_rate, pitch_rate, 0)

        -- Check if the dive altitude has been reached
        if baro:get_altitude() <= final_dive_altitude or airspeed >= AS_MAX then
            gcs:send_text(MAV_SEVERITY.INFO, "Dive Alt - Pullup.")
            -- Move to the next stage
            stage = stage + 1
        end
    end

    -- Stage 3: Dive Pullout
    if stage == 3 then
        -- Retrieve the current pitch and roll angles
        local pitch_deg = math.deg(ahrs:get_pitch())
        local roll_deg = math.deg(ahrs:get_roll())

        -- Keep the wings level in earth frame
        local roll_rate = earth_frame_wings_level(1)
        -- Calculate pitch rate to keep the nose down
        local pitch_rate, yaw_rate = pitch_controller(PULLUP_PITCH, target_yaw, PITCH_TCONST:get())
        -- Roll Limit (Iinverted orientation)
        local INVERT_ROLL_LIM = (180 - ROLL_LIMIT:get())

        -- Account for inverted flight
        if math.abs(roll_deg) >= INVERT_ROLL_LIM then
            pitch_rate = -pitch_rate
        end

        -- Set the rate targets -- zero throttle / zero yaw
        set_rate_targets(0, roll_rate, pitch_rate, 0)
        
        -- Check for roll limits upright orientation
        if math.abs(roll_deg) > ROLL_LIMIT:get() then
            -- If failed in upright, confirm in inverted orientation
            if math.abs(roll_deg) <= INVERT_ROLL_LIM then
                gcs:send_text(MAV_SEVERITY.INFO, string.format("Roll limit exceeded: %.1f deg", roll_deg))
                -- Jump to the next waypoint - (+1 NAV_ALTITUDE_WAIT)
                mission_idx = mission_idx + 1
                -- jump to next waypoint
                mission:set_current_cmd(mission_idx)
                -- Move to the next stage
                stage = stage + 1
            end
        end

        -- Check if we have pulled out ourselves..
        if(pitch_deg >= (PULLUP_PITCH * 2)) then
            gcs:send_text(MAV_SEVERITY.INFO, "Pullup Script finished.")
            -- Check if there are more waypoints to jump too (+1 NAV_ALTITUDE_WAIT, +2 to next waypoint)
            if((mission_idx + 2) < mission_cnt) then
                -- Jump to the next waypoint
                mission_idx = mission_idx + 2
                -- jump to next waypoint
                mission:set_current_cmd(mission_idx)
            else
                -- Trigger RTL - Done Missions
                vehicle:set_mode(FLIGHT_MODE.RTL)
            end
            -- Move to the next stage
            stage = stage + 1
        end
    end

    -- Stage 4: End Scripting
    if stage == 4 then
        -- Signal we are done with the mission script
        vehicle:nav_script_time_done(1)
        -- Turn ON AUTO wings
        param:set("SCR_USER1", 1.0)
        -- Move to the next stage - Done
        stage = stage + 1
        return
    end

    -- Continue the loop at the specified loop rate (LOOP_RATE is a predefined constant)
    -- Returns the update function and sets the frequency to run every loop cycle
    return update, UPDATE_RATE_MS
end

-- Initialize the update loop
return update()
