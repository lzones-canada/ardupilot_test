
--[[
   main update function, called at 1Hz
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local WING_MIN_DEGREES = 8
local WING_MAX_DEGREES = 88

local SWEEP_WING_SRV_CHANNEL = 14

-- Variable to keep track of time
local time_counter_s = 0

-- Random Angle for testing
local new_angle = 45

function deg_to_pwm(angle_deg)
    return (angle_deg + 102) * 10
end

function update()

    -- Get the current sweep angle
    local sweep_angle = volz:get_sweep_angle()
    gcs:send_text(MAV_SEVERITY.INFO, string.format("Current sweep_wing_angle: %.1f", sweep_angle))

    -- Increment the counter (1000 ms = 1 second per update call)
    time_counter_s = time_counter_s + 1

    -- Every 20 seconds (20 * 1000 ms = 10000 ms), set a new angle
    if time_counter_s >= 20 then

        -- SITL way to set the angle
        local sweep_pwm = deg_to_pwm(new_angle)
        SRV_RelayEvent:do_set_servo(SWEEP_WING_SRV_CHANNEL, sweep_pwm)
        gcs:send_text(MAV_SEVERITY.INFO, string.format("Setting new sweep_wing_pwm: %u", sweep_pwm))

        ---------------------------------------------------------------------------------------------------

        -- Regular way to set the angle
        -- volz:set_target_command(new_angle)
        -- gcs:send_text(MAV_SEVERITY.INFO, string.format("Setting new sweep_wing_angle: %.1f", new_angle))
        
        -- Reset the counter
        time_counter_s = 0
    end

    -- Return the update function, set to run every 1000 ms (1 second)
    return update, 1000
end

-- run immediately before starting to reschedule
return update()
