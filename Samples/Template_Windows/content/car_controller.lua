-- car_controller.lua
-- Minimal arcade driving script for Wicked Engine (Lua).
--
-- Expects an entity named "Car" in the scene (e.g. a model you loaded in the Editor).
-- Run it with:  dofile("path/to/car_controller.lua")  in the Backlog (HOME key),
-- or from startup.lua, or via a Script component.
-- Controls: W/S throttle+brake/reverse, A/D steer, R reset.
--
-- This is a *kinematic* car: the script moves the transform directly.
-- Remove any RigidBody from the car entity so physics doesn't fight the script.

local scene = GetScene()

-- Set to false to leave the camera alone (useful in the Editor, which also
-- drives the camera and can fight with this script).
local FOLLOW_CAMERA = true

-- The script drives the car along +Z. If your model's nose points elsewhere, correct it here (radians):
--   nose along +X  ->  -math.pi / 2
--   nose along -X  ->   math.pi / 2
--   nose along -Z  ->   math.pi
local MODEL_YAW_OFFSET = math.pi / 2

-- Which keys drive the car: "wasd", "arrows" or "both".
-- The Editor moves its own camera with WASD, so when running inside the Editor use
-- "arrows" to keep the camera from also reacting to your driving keys.
local DRIVE_KEYS = "both"

-- Draws a red line showing the direction the script is driving. It should come
-- out of the car's nose. Set to false once the model is aligned.
local DEBUG_HEADING = false

-- Optional: let each wheel move up and down independently so the wheels stay on the
-- ground while the body pitches and rolls (suspension travel). Fill in the names of
-- your wheel entities (as shown in the Editor) and where each one sits relative to
-- the car's origin, in metres and in the car's own directions:
--   x = to the right of centre (negative = left)   z = ahead of centre (negative = behind)
-- Leave the list empty to skip it; the body will still pitch and roll.
local WHEELS = {
    -- { name = "Wheel_FL", x = -0.8, z =  1.4 },
    -- { name = "Wheel_FR", x =  0.8, z =  1.4 },
    -- { name = "Wheel_RL", x = -0.8, z = -1.3 },
    -- { name = "Wheel_RR", x =  0.8, z = -1.3 },
}

---------------------------------------------------------------------------
-- Tuning
---------------------------------------------------------------------------
local TUNING = {
    max_speed   = 40.0,  -- m/s forward
    max_reverse = 10.0,  -- m/s backward
    accel       = 6.0,  -- m/s^2
    brake       = 20.0,  -- m/s^2
    rolling     = 5.0,   -- passive slowdown when coasting (m/s^2)
    steer_rate  = 2.0,   -- rad/s at full lock
    steer_speed = 8.0,   -- speed at which steering reaches full authority
    steer_top_scale  = 0.35, -- fraction of steer_rate left at max_speed (1.0 = no reduction)
    steer_falloff_curve = 1.0, -- 1 = linear loss of steering with speed; >1 stays sharp longer, then drops late
    steer_response   = 4.0,  -- how fast the steering wheel follows your key (units/s, full lock = 1); lower = smoother
    steer_return     = 6.0,  -- how fast it recentres when you release the key
    handbrake            = 14.0, -- m/s^2 of braking while Space is held (engine power is cut too)
    handbrake_steer_boost = 1.6, -- turn-rate multiplier while the handbrake is on, so the tail swings round
    handbrake_slide      = 0.8,  -- 0..1: how much of each turn the direction of travel does NOT follow with zero grip (0 = no sliding at all)
    handbrake_min_speed  = 3.0,  -- slower than this (m/s) the car can't slide
    slip_hold_recover    = 0.6,  -- how quickly a slide settles when there is NO grip (per second; lower = longer slides)
    slip_recover         = 4.0,  -- how quickly a slide settles with FULL grip (per second; higher = snappier)
    slip_max             = 1.0,  -- biggest slide angle in radians (1.0 is about 57 degrees)
    handbrake_traction   = 0.0,  -- grip left while the handbrake is held at speed (0 = none, 1 = full)
    traction_loss_rate   = 6.0,  -- how fast grip is lost when Space is held (per second)
    traction_recover_time = 1.5, -- seconds to get full grip back after the handbrake (higher = stays loose for longer)
    low_traction_accel   = 0.4,  -- fraction of engine acceleration available with zero grip (wheelspin)
    pitch_gain   = 0.005,   -- radians of body pitch per m/s^2 of acceleration (nose dives under braking, squats under power)
    roll_gain    = 0.0035,  -- radians of body roll per m/s^2 of sideways acceleration (body leans outward in corners)
    max_pitch    = 0.07,    -- pitch limit in radians (0.07 is about 4 degrees)
    max_roll     = 0.09,    -- roll limit in radians (0.09 is about 5 degrees)
    heave_long   = 0.0015,  -- metres the body sinks per m/s^2 of forward/back acceleration
    heave_lat    = 0.0010,  -- metres the body sinks per m/s^2 of sideways acceleration
    max_heave    = 0.06,    -- deepest the body can sink (metres)
    body_freq    = 2.2,     -- suspension spring frequency in Hz (higher = stiffer and quicker to settle)
    body_damping = 0.5,     -- damping ratio: 1.0 = no bounce, 0.3 = bouncy
    weight_steer_effect = 0.25, -- weight on the front axle changes steering: +25% turn rate when nose-dived, -25% when squatting (0 = off)
    wheel_travel_limit = 0.20,  -- most a wheel can move relative to the body, in metres (only used when WHEELS is filled in)
    cam_back        = 4.5,   -- distance behind the car when stopped
    cam_back_fast   = 1.1,  -- distance behind the car at top speed
    cam_up          = 2.5,   -- camera height when stopped
    cam_up_fast     = 2.5,   -- camera height at top speed
    cam_speed_curve = 1.0,   -- 1 = linear with speed; >1 stays close longer, then pulls back late
    cam_blend_smooth = 2.5,  -- how quickly the pull-back eases in/out (lower = lazier)
    cam_smooth      = 8.0,   -- higher = camera snaps to its target position faster
    cam_yaw_time    = 0.25,  -- seconds the camera takes to swing round to the car's heading (higher = lazier, smoother)
    orbit_sensitivity = 0.005, -- radians of camera rotation per pixel of mouse movement (right mouse button held)
    orbit_pitch_min   = -0.2,  -- how far the camera may drop below its normal angle (radians)
    orbit_pitch_max   = 1.2,   -- how far it may rise above it (radians); about 1.5 is straight overhead
    orbit_return_time = 0.5,   -- seconds to ease back behind the car after you let go (0 = stay where you left it)
    cam_reverse_flip    = true, -- swing the camera round to the front of the car while reversing
    cam_flip_speed      = 2.0,  -- reverse faster than this (m/s) and the camera swings round
    cam_flip_back_speed = 0.5,  -- slow to below this (m/s) while reversing and it swings back
    cam_flip_time       = 0.6,  -- seconds the swing takes (higher = slower, wider arc)
}

---------------------------------------------------------------------------
-- Setup
---------------------------------------------------------------------------
local car = scene.Entity_FindByName("Car")
if car == INVALID_ENTITY then
    backlog_post("car_controller: no entity named 'Car' found in the scene")
    return
end
local car_transform = scene.Component_GetTransform(car)

local start_pos = car_transform.GetPosition()
local state = { pos = start_pos, yaw = 0.0, speed = 0.0, steer = 0.0, slip = 0.0, traction = 1.0 }

-- Suspension state of the body: angles and sink are spring-damper driven (see update_body).
-- pitch: positive = nose down.  roll: positive = leaning left (right side up).
local body = {
    pitch = 0.0, pitch_vel = 0.0,
    roll  = 0.0, roll_vel  = 0.0,
    heave = 0.0, heave_vel = 0.0,   -- vertical offset in metres (negative = sunk)
    prev_vel = Vector(0, 0, 0),     -- travel velocity last frame, to measure acceleration
}

-- Wheels for suspension travel. Offsets applied by an earlier run of this script are
-- undone first, so re-running the script never leaves wheels displaced.
CAR_WHEEL_OFFSETS = CAR_WHEEL_OFFSETS or {}
local wheels = {}
for _, w in ipairs(WHEELS) do
    local e = scene.Entity_FindByName(w.name)
    if e == INVALID_ENTITY then
        backlog_post("car_controller: wheel entity '" .. w.name .. "' not found")
    else
        local tr = scene.Component_GetTransform(e)
        local previous = CAR_WHEEL_OFFSETS[e]
        if previous ~= nil and previous ~= 0 then
            tr.Translate(Vector(0, -previous, 0))
        end
        CAR_WHEEL_OFFSETS[e] = 0
        wheels[#wheels + 1] = { entity = e, transform = tr, x = w.x, z = w.z, applied = 0.0 }
    end
end

-- Camera rig: the camera is driven through a transform on a helper entity.
local camera = GetCamera()
local CAM_HELPER_NAME = "CarCameraHelper"

-- Reuse the helper if an earlier run already created it; otherwise create it
-- once and give it a name so it is easy to spot (and found again next run).
local function get_camera_helper()
    local e = scene.Entity_FindByName(CAM_HELPER_NAME)
    if e == INVALID_ENTITY then
        e = CreateEntity()
        scene.Component_CreateName(e).SetName(CAM_HELPER_NAME)
        scene.Component_CreateTransform(e)
    end
    return scene.Component_GetTransform(e)
end

local cam_transform = get_camera_helper()
local cam_pos = nil
local cam_blend = 0.0  -- smoothed 0..1 speed factor that drives camera distance/height
local cam_yaw = nil    -- the camera's own heading; eases toward the car's heading
local cam_yaw_vel = 0.0

-- Manual camera orbit (hold right mouse button and move the mouse)
local orbit_yaw, orbit_pitch = 0.0, 0.0
local orbit_yaw_vel, orbit_pitch_vel = 0.0, 0.0
local last_pointer_x, last_pointer_y = 0.0, 0.0
local was_orbiting = false

-- Reverse view: the camera swings round to the front of the car while reversing
local reverse_view = false
local flip, flip_vel = 0.0, 0.0   -- 0 = behind the car, pi = in front of it

---------------------------------------------------------------------------
-- Input (letters are passed as their character code)
---------------------------------------------------------------------------
local KEY = {
    reset     = string.byte('R'),
    handbrake = KEYBOARD_BUTTON_SPACE,
}

local WASD = {
    throttle = string.byte('W'),
    brake    = string.byte('S'),
    left     = string.byte('A'),
    right    = string.byte('D'),
}

local ARROWS = {
    throttle = KEYBOARD_BUTTON_UP,
    brake    = KEYBOARD_BUTTON_DOWN,
    left     = KEYBOARD_BUTTON_LEFT,
    right    = KEYBOARD_BUTTON_RIGHT,
}

local function held(action)
    if DRIVE_KEYS ~= "arrows" and input.Down(WASD[action]) then
        return true
    end
    if DRIVE_KEYS ~= "wasd" and ARROWS[action] ~= nil and input.Down(ARROWS[action]) then
        return true
    end
    return false
end

local function read_input()
    local throttle = held("throttle") and 1 or 0
    local brake    = held("brake") and 1 or 0
    local steer    = 0
    if held("right") then steer = steer + 1 end
    if held("left")  then steer = steer - 1 end
    local handbrake = KEY.handbrake ~= nil and input.Down(KEY.handbrake)
    return throttle, brake, steer, handbrake
end

---------------------------------------------------------------------------
-- Simulation
---------------------------------------------------------------------------
-- Move `current` toward `target` by at most `max_delta`.
local function approach(current, target, max_delta)
    if current < target then
        return math.min(current + max_delta, target)
    end
    return math.max(current - max_delta, target)
end

local function heading()
    -- Wicked Engine is left-handed, Y up: yaw 0 faces +Z, positive yaw turns toward +X
    return Vector(math.sin(state.yaw), 0, math.cos(state.yaw))
end

-- The direction the car is actually travelling. Normally the same as its heading;
-- during a handbrake slide the heading swings away from it by `slip` radians.
local function travel_heading()
    local a = state.yaw - state.slip
    return Vector(math.sin(a), 0, math.cos(a))
end

local function step_car(dt)
    local t = TUNING
    local throttle, brake, steer, handbrake = read_input()

    if handbrake then
        -- Handbrake: power is cut and the car slows down (it never pushes into reverse).
        local drag = t.handbrake * dt
        if math.abs(state.speed) <= drag then
            state.speed = 0
        else
            state.speed = state.speed - ((state.speed > 0) and drag or -drag)
        end
    elseif throttle > 0 then
        state.speed = state.speed + t.accel * math.lerp(t.low_traction_accel, 1.0, state.traction) * dt
    elseif brake > 0 then
        -- brake while rolling forward, reverse once (nearly) stopped
        local a = (state.speed > 0.5) and t.brake or (t.accel * 0.5)
        state.speed = state.speed - a * dt
    else
        -- coast: ease speed toward zero without overshooting
        local drag = t.rolling * dt
        if math.abs(state.speed) <= drag then
            state.speed = 0
        else
            state.speed = state.speed - ((state.speed > 0) and drag or -drag)
        end
    end
    state.speed = math.clamp(state.speed, -t.max_reverse, t.max_speed)

    -- Smooth the steering input so changing direction is a quick sweep, not a snap.
    -- (Recentring is allowed to be faster than turning in.)
    local response = (steer == 0) and t.steer_return or t.steer_response
    state.steer = approach(state.steer, steer, response * dt)

    -- Steering authority scales with speed at low speed (no spinning on the spot)
    -- and flips sign in reverse, like a real car.
    local authority = math.clamp(state.speed / t.steer_speed, -1, 1)

    -- ...and at high speed the turn rate is reduced, from 100% when crawling
    -- down to steer_top_scale at max_speed.
    local speed_frac = math.saturate(math.abs(state.speed) / t.max_speed) ^ t.steer_falloff_curve
    local grip = math.lerp(1.0, t.steer_top_scale, speed_frac)

    local boost = handbrake and t.handbrake_steer_boost or 1.0
    -- Weight transfer: braking loads the front tyres (sharper turn-in), power unloads
    -- them (understeer). Uses the suspension's actual pitch, so it arrives with the same lag.
    local front_load = (t.max_pitch > 0) and math.clamp(body.pitch / t.max_pitch, -1, 1) or 0
    local weight_steer = 1.0 + t.weight_steer_effect * front_load

    local dyaw  = state.steer * t.steer_rate * authority * grip * boost * weight_steer * dt
    state.yaw = state.yaw + dyaw

    -- Traction: holding the handbrake at speed locks the rear wheels, so grip drops
    -- quickly. After you let go it only comes back gradually, so the car stays loose
    -- for a while (slides on with steering, wheelspin on throttle) instead of
    -- snapping back to full grip.
    if handbrake and math.abs(state.speed) > t.handbrake_min_speed then
        state.traction = approach(state.traction, t.handbrake_traction, t.traction_loss_rate * dt)
    else
        state.traction = approach(state.traction, 1.0, dt / math.max(t.traction_recover_time, 0.0001))
    end

    -- Slide: with less than full grip, only part of each turn carries the direction
    -- of travel with it, so the tail swings out. The slide settles back to zero
    -- quickly with full grip and slowly without it.
    local loose = 1.0 - state.traction
    state.slip = state.slip + dyaw * t.handbrake_slide * loose
    local settle = math.lerp(t.slip_hold_recover, t.slip_recover, state.traction)
    state.slip = state.slip - state.slip * math.saturate(settle * dt)
    state.slip = math.clamp(state.slip, -t.slip_max, t.slip_max)

    state.pos = vector.Add(state.pos, vector.Multiply(travel_heading(), state.speed * dt))
end

-- One step of a damped spring pulling `x` toward `target` (semi-implicit Euler,
-- sub-stepped so a slow frame can't make it unstable). Returns new x, new velocity.
local function spring_step(x, v, target, dt)
    local w = 2 * math.pi * TUNING.body_freq
    local z = TUNING.body_damping
    local steps = math.max(1, math.ceil(dt * 120))
    local h = dt / steps
    for _ = 1, steps do
        v = v + (w * w * (target - x) - 2 * z * w * v) * h
        x = x + v * h
    end
    return x, v
end

-- Weight transfer. Acceleration is measured from how the travel velocity changes, split
-- into forward and sideways parts in the car's own frame, then pushed through springs:
-- braking dives the nose, power squats the tail, cornering leans the body outward and
-- hard loads sink the whole body a little.
local function update_body(dt)
    local t = TUNING
    local vel   = vector.Multiply(travel_heading(), state.speed)
    local accel = vector.Multiply(vector.Subtract(vel, body.prev_vel), 1 / math.max(dt, 0.0001))
    body.prev_vel = vel

    local fwd   = heading()
    local right = Vector(fwd.GetZ(), 0, -fwd.GetX())
    local a_long = vector.Dot(accel, fwd)
    local a_lat  = vector.Dot(accel, right)

    local target_pitch = math.clamp(-a_long * t.pitch_gain, -t.max_pitch, t.max_pitch)
    local target_roll  = math.clamp( a_lat  * t.roll_gain,  -t.max_roll,  t.max_roll)
    local target_heave = -math.min(math.abs(a_long) * t.heave_long + math.abs(a_lat) * t.heave_lat, t.max_heave)

    body.pitch, body.pitch_vel = spring_step(body.pitch, body.pitch_vel, target_pitch, dt)
    body.roll,  body.roll_vel  = spring_step(body.roll,  body.roll_vel,  target_roll,  dt)
    body.heave, body.heave_vel = spring_step(body.heave, body.heave_vel, target_heave, dt)
end

-- The model's own axes are turned by MODEL_YAW_OFFSET, and the engine applies pitch/roll
-- about the model's axes before yaw. For offsets that are multiples of 90 degrees the
-- heading-frame pitch/roll map onto the model's X/Z rotations like this:
local QUARTER_TURNS = math.floor(MODEL_YAW_OFFSET / (math.pi / 2) + 0.5)
local OFFSET_IS_QUARTER = math.abs(MODEL_YAW_OFFSET - QUARTER_TURNS * (math.pi / 2)) < 0.01
local warned_offset = false

local function tilt_euler(pitch, roll)
    if not OFFSET_IS_QUARTER then
        if not warned_offset then
            backlog_post("car_controller: MODEL_YAW_OFFSET is not a multiple of 90 degrees, so body pitch/roll is disabled")
            warned_offset = true
        end
        return 0.0, 0.0
    end
    local q = QUARTER_TURNS % 4
    if q == 0 then return pitch, roll end
    if q == 1 then return -roll, pitch end
    if q == 2 then return -pitch, -roll end
    return roll, -pitch
end

local function apply_car()
    local ex, ez = tilt_euler(body.pitch, body.roll)
    car_transform.ClearTransform()
    car_transform.Translate(vector.Add(state.pos, Vector(0, body.heave, 0)))
    -- MODEL_YAW_OFFSET only turns the visible model; driving direction is unaffected.
    car_transform.Rotate(Vector(ex, state.yaw + MODEL_YAW_OFFSET, ez)) -- rotation about X, Y, Z (radians)
    car_transform.UpdateTransform()
end

-- Suspension travel: tilting the body would lift or sink each wheel, so move every wheel
-- the opposite way along the body's up axis to keep it planted. A wheel at (x, z) is
-- displaced by heave + x*roll - z*pitch, so it gets the negative of that.
local function apply_wheels()
    local limit = TUNING.wheel_travel_limit
    for _, w in ipairs(wheels) do
        local lift = -(body.heave + w.x * body.roll - w.z * body.pitch)
        lift = math.clamp(lift, -limit, limit)
        local delta = lift - w.applied
        if math.abs(delta) > 0.00001 then
            w.transform.Translate(Vector(0, delta, 0))
            w.applied = lift
            CAR_WHEEL_OFFSETS[w.entity] = lift
        end
    end
end

-- Wrap an angle into [-pi, pi).
local function wrap_pi(a)
    return (a + math.pi) % (2 * math.pi) - math.pi
end

-- Critically damped spring toward `target` (same idea as Unity's SmoothDamp).
-- Unlike a plain lerp, speed builds up and settles gradually, so reversing
-- direction doesn't make the camera jerk. Returns new value, new velocity.
local function smooth_damp(current, target, velocity, smooth_time, dt)
    smooth_time = math.max(smooth_time, 0.0001)
    local omega  = 2.0 / smooth_time
    local x      = omega * dt
    local decay  = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x)
    local change = current - target
    local temp   = (velocity + omega * change) * dt
    velocity = (velocity - omega * temp) * decay
    return target + (change + temp) * decay, velocity
end

-- Same, for angles: always takes the short way round.
local function smooth_damp_angle(current, target, velocity, smooth_time, dt)
    return smooth_damp(current, current + wrap_pi(target - current), velocity, smooth_time, dt)
end

-- Hold the right mouse button and move the mouse to swing the camera around the
-- car. On release it eases back to the normal chase position. Mouse movement is
-- measured from the pointer position each frame.
local function update_orbit(dt)
    local t = TUNING
    local pointer = input.GetPointer()
    local px, py = pointer.GetX(), pointer.GetY()

    if input.Down(MOUSE_BUTTON_RIGHT) then
        if was_orbiting then  -- skip the first frame so the view doesn't jump
            orbit_yaw   = orbit_yaw + (px - last_pointer_x) * t.orbit_sensitivity
            orbit_pitch = math.clamp(orbit_pitch + (py - last_pointer_y) * t.orbit_sensitivity,
                                     t.orbit_pitch_min, t.orbit_pitch_max)
        end
        orbit_yaw_vel, orbit_pitch_vel = 0.0, 0.0
        was_orbiting = true
    else
        if was_orbiting then
            orbit_yaw = wrap_pi(orbit_yaw)  -- so the way back is the short way round
            was_orbiting = false
        end
        if t.orbit_return_time > 0 then
            orbit_yaw,   orbit_yaw_vel   = smooth_damp_angle(orbit_yaw,   0.0, orbit_yaw_vel,   t.orbit_return_time, dt)
            orbit_pitch, orbit_pitch_vel = smooth_damp_angle(orbit_pitch, 0.0, orbit_pitch_vel, t.orbit_return_time, dt)
        end
    end

    last_pointer_x, last_pointer_y = px, py
end

local function follow_camera(dt)
    -- Pull the camera back (and up a little) as forward speed rises.
    local target_blend = math.saturate(math.max(state.speed, 0) / TUNING.max_speed) ^ TUNING.cam_speed_curve
    cam_blend = math.lerp(cam_blend, target_blend, math.saturate(TUNING.cam_blend_smooth * dt))
    local dist   = math.lerp(TUNING.cam_back, TUNING.cam_back_fast, cam_blend)
    local height = math.lerp(TUNING.cam_up,   TUNING.cam_up_fast,   cam_blend)

    -- The camera sits behind its own, smoothed heading rather than the car's raw
    -- heading, so quick left/right changes swing the camera gently.
    if cam_yaw == nil then
        cam_yaw = state.yaw
    end
    cam_yaw, cam_yaw_vel = smooth_damp_angle(cam_yaw, state.yaw, cam_yaw_vel, TUNING.cam_yaw_time, dt)
    update_orbit(dt)

    -- Reverse view: once reversing properly, swing round to the front of the car so
    -- you can see where you're going; swing back when you slow down or go forward.
    -- Two speed thresholds stop it flickering as the car crosses zero speed.
    if TUNING.cam_reverse_flip then
        if state.speed < -TUNING.cam_flip_speed then
            reverse_view = true
        elseif state.speed > -TUNING.cam_flip_back_speed then
            reverse_view = false
        end
    else
        reverse_view = false
    end
    flip, flip_vel = smooth_damp(flip, reverse_view and math.pi or 0.0, flip_vel, TUNING.cam_flip_time, dt)

    -- Manual orbit: yaw turns the camera around the car; pitch trades distance for height.
    local view_yaw    = cam_yaw + orbit_yaw + flip
    local cam_heading = Vector(math.sin(view_yaw), 0, math.cos(view_yaw))
    local horizontal  = dist * math.cos(orbit_pitch)
    local vertical    = height + dist * math.sin(orbit_pitch)

    local back   = vector.Multiply(cam_heading, -horizontal)
    local wanted = vector.Add(state.pos, vector.Add(back, Vector(0, vertical, 0)))

    if cam_pos == nil then
        cam_pos = wanted
    else
        cam_pos = vector.Lerp(cam_pos, wanted, math.saturate(TUNING.cam_smooth * dt))
    end

    -- Aim the camera at the car by computing yaw/pitch from the direction vector.
    -- The helper transform is cleared first and rebuilt from scratch every frame,
    -- so nothing can accumulate from one frame to the next.
    local target    = vector.Add(state.pos, Vector(0, 1.2, 0))
    local to_target = vector.Subtract(target, cam_pos)
    local dx, dy, dz = to_target.GetX(), to_target.GetY(), to_target.GetZ()
    local yaw   = math.atan(dx, dz)                           -- 0 faces +Z
    local pitch = math.atan(-dy, math.sqrt(dx * dx + dz * dz)) -- positive = looking down

    cam_transform.ClearTransform()
    cam_transform.Translate(cam_pos)
    cam_transform.Rotate(Vector(pitch, yaw, 0))  -- rotation about X, Y, Z (radians)
    cam_transform.UpdateTransform()
    camera.TransformCamera(cam_transform)
    camera.UpdateCamera()
end

---------------------------------------------------------------------------
-- Main loop, as a process (coroutine). Re-running the script replaces the
-- old instance, so you can iterate quickly with dofile() in the Backlog.
---------------------------------------------------------------------------
if CAR_CONTROLLER_PROCESS then
    killProcess(CAR_CONTROLLER_PROCESS)
end

local ok, co = runProcess(function()
    while true do
        update()                                   -- wait for the next frame
        local dt = math.min(getDeltaTime(), 0.05)  -- clamp frame hitches

        if input.Press(KEY.reset) then
            state.pos = start_pos
            state.yaw = 0.0
            state.speed = 0.0
            state.steer = 0.0
            state.slip = 0.0
            state.traction = 1.0
            body.pitch, body.pitch_vel = 0.0, 0.0
            body.roll,  body.roll_vel  = 0.0, 0.0
            body.heave, body.heave_vel = 0.0, 0.0
            body.prev_vel = Vector(0, 0, 0)
        end

        step_car(dt)
        update_body(dt)
        apply_car()
        apply_wheels()
        if FOLLOW_CAMERA then
            follow_camera(dt)
        end

        if DEBUG_HEADING then
            local tip = vector.Add(state.pos, vector.Add(vector.Multiply(heading(), 6), Vector(0, 1, 0)))
            DrawLine(vector.Add(state.pos, Vector(0, 1, 0)), tip, Vector(1, 0, 0, 1), false)
        end

        -- Placeholder HUD: world-space debug text floating above the car
        DrawDebugText(
            string.format("%d km/h", math.floor(math.abs(state.speed) * 3.6)),
            vector.Add(state.pos, Vector(0, 2.5, 0)),
            Vector(1, 1, 1, 1),
            1.0,
            DEBUG_TEXT_CAMERA_FACING | DEBUG_TEXT_CAMERA_SCALING
        )
    end
end)

CAR_CONTROLLER_PROCESS = co
