-- Constants
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local FLIGHT_MODE = {MANUAL=0,CIRCLE=1,STABILIZE=2,TRAINING=3,ACRO=4,FLY_BY_WIRE_A=5,FLY_BY_WIRE_B=6,CRUISE=7,AUTOTUNE=8,AUTO=10,RTL=11,LOITER=12,TAKEOFF=13,AVOID_ADSB=14,GUIDED=15,INITIALISING=16} -- Flight modes


local UPDATE_RATE_MS = 50 -- Update rate in milliseconds

-- Wing sweep parameters
local WING_DEG_PER_SEC = 2.5
WING_MOTION_CONTROL = false

WA_CURR_POS = 32 -- Current wing position
WA_CURR_TGT = 31 -- Target wing position

local WA_MIN = 10 -- Min wing sweep
local WA_MAX = 45 -- Max wing sweep
local WA_DIVE = 30

-- Airspeed definitions
local V_CB = 60 -- Max airspeed for wings forward (above this, wings start retracting)
local V_MAX = 160 -- Max airspeed for adjustable wing (above this, wings swept to wa_max)

local SWEEP_WING_SRV_CHANNEL = 13 -- Servo channel for wing sweep
-- Function to set wing angle -- SITL
local function set_wing_angle(angle)
    -- convert degrees to PWM
    local sweep_pwm = math.floor((angle + 102) * 10)
    SRV_RelayEvent:do_set_servo(SWEEP_WING_SRV_CHANNEL, sweep_pwm)
end

-- Function to set wing angle -- HITL
-- local function set_wing_angle(angle)
--     -- Round the value
--     local angle_round = math.floor(angle + 0.5)
--     volz:set_target_command(angle_round)
-- end

-- Linear interpolation function for 1D
local function interpolate_1d(x, x0, x1, y0, y1)
    if x1 == x0 then
        return y0 -- Avoid division by zero
    elseif x <= x0 then
        return y0
    elseif x >= x1 then
        return y1
    else
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    end
end

function update()

    local airspeed = ahrs:airspeed_estimate()
    local mode = vehicle:get_mode()
    local AUTO_WINGS_FLT_MODE = (mode ~= FLIGHT_MODE.FLY_BY_WIRE_A and mode ~= FLIGHT_MODE.FLY_BY_WIRE_B and mode ~= FLIGHT_MODE.CRUISE and mode ~= FLIGHT_MODE.GUIDED)

    if param:get("SCR_USER1") == 1 then
        WING_MOTION_CONTROL = true
    else
        WING_MOTION_CONTROL = false
    end

    if WING_MOTION_CONTROL == true then
        if airspeed <= V_CB then -- stall up through cruise condition, wings forward
            WA_CURR_TGT = WA_MIN
        elseif airspeed > V_CB and airspeed <= V_MAX then -- transition to high speed
            WA_CURR_TGT = interpolate_1d(airspeed, V_CB, V_MAX, WA_MIN, WA_MAX)
        elseif airspeed > V_MAX then -- high speed, wings swept
            WA_CURR_TGT = WA_MAX
        else
            WA_CURR_TGT = WA_MIN
        end
    end

    -- Slowing motion to below limit set by WING_DEG_PER_SEC
    if math.abs(WA_CURR_TGT - WA_CURR_POS) > (WING_DEG_PER_SEC/(UPDATE_RATE_MS/1000)) then
        WA_CURR_POS = WA_CURR_POS + WING_DEG_PER_SEC * (WA_CURR_TGT - WA_CURR_POS) / math.abs(WA_CURR_TGT - WA_CURR_POS)
    else
        WA_CURR_POS = WA_CURR_TGT
    end

    --gcs:send_text(MAV_SEVERITY.INFO, string.format("WA Target: %.1f degrees - WA Prev: %.1f degrees", WA_CURR_TGT, WA_CURR_POS))

    -- Auto wing control when ARMED, SCR_USER1 and not in FLY_BY_WIRE_A, FLY_BY_WIRE_B, CRUISE, GUIDED
    if arming:is_armed() and WING_MOTION_CONTROL and AUTO_WINGS_FLT_MODE then
        set_wing_angle(WA_CURR_POS)
    end

    -- Continue the loop at the specified loop rate
    return update, UPDATE_RATE_MS
end

return update()
