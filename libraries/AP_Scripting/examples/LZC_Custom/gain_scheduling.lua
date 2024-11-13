-- Constants
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local UPDATE_RATE_MS = 50 -- Update rate in milliseconds

-- Gains for wing sweep
local WING_PRESETS = {10, 25}

local PTCH_RATE_FF = {0.533, 0.949}
local PTCH_RATE_P = {0.424, 0.642}
local PTCH_RATE_I = {0.533, 0.949}
local PTCH_RATE_D = {0.0076, 0.003}

local RLL_RATE_FF = {0.621, 0.501}
local RLL_RATE_P = {0.214, 0.421}
local RLL_RATE_I = {0.214, 0.421}
local RLL_RATE_D = {0.0077, 0.003}

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

    -- Get the current sweep wing angle
    WA_CURR_POS = math.floor(volz:get_sweep_angle() + 0.5)
    -- gcs:send_text(MAV_SEVERITY.INFO, string.format("Wing angle: %.1f degrees", WA_CURR_POS))

    -- Adjust gains based on new wing angle
    param:set("PTCH_RATE_FF", interpolate_1d(WA_CURR_POS, WING_PRESETS[1], WING_PRESETS[2], PTCH_RATE_FF[1], PTCH_RATE_FF[2]))
    param:set("PTCH_RATE_P",  interpolate_1d(WA_CURR_POS, WING_PRESETS[1], WING_PRESETS[2], PTCH_RATE_P[1], PTCH_RATE_P[2]))
    param:set("PTCH_RATE_I",  interpolate_1d(WA_CURR_POS, WING_PRESETS[1], WING_PRESETS[2], PTCH_RATE_I[1], PTCH_RATE_I[2]))
    param:set("PTCH_RATE_D",  interpolate_1d(WA_CURR_POS, WING_PRESETS[1], WING_PRESETS[2], PTCH_RATE_D[1], PTCH_RATE_D[2]))

    param:set("RLL_RATE_FF",  interpolate_1d(WA_CURR_POS, WING_PRESETS[1], WING_PRESETS[2], RLL_RATE_FF[1], RLL_RATE_FF[2]))
    param:set("RLL_RATE_P",   interpolate_1d(WA_CURR_POS, WING_PRESETS[1], WING_PRESETS[2], RLL_RATE_P[1], RLL_RATE_P[2]))
    param:set("RLL_RATE_I",   interpolate_1d(WA_CURR_POS, WING_PRESETS[1], WING_PRESETS[2], RLL_RATE_I[1], RLL_RATE_I[2]))
    param:set("RLL_RATE_D",   interpolate_1d(WA_CURR_POS, WING_PRESETS[1], WING_PRESETS[2], RLL_RATE_D[1], RLL_RATE_D[2]))

    -- gcs:send_text(MAV_SEVERITY.INFO, string.format("\nPTCH:\t%.3f\t%.3f\t%.3f\t%.3f\nRLL:\t%.3f\t%.3f\t%.3f\t%.3f", param:get("PTCH_RATE_FF"), param:get("PTCH_RATE_P"), param:get("PTCH_RATE_I"), param:get("PTCH_RATE_D"), param:get("RLL_RATE_FF"), param:get("RLL_RATE_P"), param:get("RLL_RATE_I"), param:get("RLL_RATE_D")))

    -- Continue the loop at the specified loop rate
    return update, UPDATE_RATE_MS
end

return update()
