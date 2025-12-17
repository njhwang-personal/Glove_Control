from controller import Robot
import socket
import select

TIME_STEP = 32

YAW_DECREASE = 15.0
YAW_INCREASE = 15.0
PITCH_INCREASE = 15.0
PITCH_DECREASE = 15.0
ROLL_DECREASE = 15.0
ROLL_INCREASE = 15.0

MOVEMENT_DETECT = 4

def angle_diff(a, b):

    d = a - b
    if d > 180:
        d -= 360
    elif d < -180:
        d += 360
    return d

def main():
    robot = Robot()
    step_count = 0

    # --- Motor setup ---
    m1 = robot.getDevice("m1_motor")
    m2 = robot.getDevice("m2_motor")
    m3 = robot.getDevice("m3_motor")
    m4 = robot.getDevice("m4_motor")

    for m in (m1, m2, m3, m4):
        m.setPosition(float('inf'))
        m.setVelocity(0.0)

    # --- UDP setup ---
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", UDP_PORT))
    sock.setblocking(False)

    print(f"[glove_ctrl] Started, listening on UDP port {UDP_PORT}")

    latest = {
        "seq": 0,
        "yaw": 0.0,
        "pitch": 0.0,
        "roll": 0.0,
        "flex0": 0,
        "flex1": 0,
        "flex2": 0,
        "flex3": 0,
        "fsr": 0,
    }

    # --- Calibration ---
    yaw0   = None
    pitch0 = None
    roll0  = None
   

    movement_counters = {
        "yaw_left": 0,
        "yaw_right": 0,
        "pitch_forward": 0,
        "pitch_backward": 0,
        "roll_left": 0,
        "roll_right": 0,
    }

    active_movement = {
        "yaw_left": False,
        "yaw_right": False,
        "pitch_forward": False,
        "pitch_backward": False,
        "roll_left": False,
        "roll_right": False,
    }

    hover = 55.4
    translation_gain = 2.0
    rotation_gain = 2.0
    altitude_gain = 3.0

    while robot.step(TIME_STEP) != -1:
        step_count += 1

        # --- UDP read ---
        r, _, _ = select.select([sock], [], [], 0)
        if sock in r:
            try:
                data, addr = sock.recvfrom(1024)
                text = data.decode('ascii', errors='ignore').strip()
                parts = text.split(',')
                if len(parts) == 9:
                    latest["seq"]   = int(parts[0])
                    latest["yaw"]   = float(parts[1])
                    latest["pitch"] = float(parts[2])
                    latest["roll"]  = float(parts[3])
                    latest["flex0"] = int(parts[4])
                    latest["flex1"] = int(parts[5])
                    latest["flex2"] = int(parts[6])
                    latest["flex3"] = int(parts[7])
                    latest["fsr"]   = int(parts[8])

                    if yaw0 is None:
                        yaw0   = latest["yaw"]
                        pitch0 = latest["pitch"]
                        roll0  = latest["roll"]
                       
                        print("[glove_ctrl] Calibration done:")
                        print(f"  yaw0   = {yaw0}")
                        print(f"  pitch0 = {pitch0}")
                        print(f"  roll0  = {roll0}")


            except Exception as e:
                print(f"[glove_ctrl] UDP error: {e}")

        if yaw0 is None:
            for m in (m1, m2, m3, m4):
                m.setVelocity(0.0)
            continue

        yaw_corr   = angle_diff(latest["yaw"], yaw0)
        pitch_corr = angle_diff(latest["pitch"], pitch0)
        roll_corr  = angle_diff(latest["roll"], roll0)

        #Take the average of all 4 flex sensors
        flex_avg = (latest["flex0"] + latest["flex1"] +
                   latest["flex2"] + latest["flex3"]) / 4.0


        speed_factor = max(0.0, min(2.0, 0.5 + abs( latest["flex1"]* 0.3)))


        temp_movements = {
            "yaw_left": False,
            "yaw_right": False,
            "pitch_forward": False,
            "pitch_backward": False,
            "roll_left": False,
            "roll_right": False,
        }

        #Yaw filter
        if yaw_corr < -YAW_DECREASE:
            temp_movements["yaw_left"] = True
        elif yaw_corr > YAW_INCREASE:
            temp_movements["yaw_right"] = True

        # Pitch filter
        if pitch_corr > PITCH_INCREASE:
            temp_movements["pitch_forward"] = True
        elif pitch_corr < -PITCH_DECREASE:
            temp_movements["pitch_backward"] = True

        # Roll filter
        if roll_corr < -ROLL_DECREASE:
            temp_movements["roll_left"] = True
        elif roll_corr > ROLL_INCREASE:
            temp_movements["roll_right"] = True

        # Noise filter
        for key in movement_counters:
            if temp_movements[key]:
                movement_counters[key] += 1
                if movement_counters[key] >= MOVEMENT_DETECT:
                    active_movement[key] = True
            else:
                movement_counters[key] = 0
                active_movement[key] = False


        cmd_pitch = 0.0
        cmd_roll = 0.0
        cmd_yaw = 0.0
        cmd_throttle = 0.0


        fsr_active = (latest["fsr"] == 1)

        if fsr_active:
            if active_movement["pitch_forward"]:
                cmd_throttle = -altitude_gain * speed_factor
                print(f"[glove_ctrl] Mode altitude: DESCENTE")
            elif active_movement["pitch_backward"]:
                cmd_throttle = altitude_gain * speed_factor
                print(f"[glove_ctrl] Mode altitude: MONTÉE")
        else:
            if active_movement["pitch_forward"]:
                cmd_pitch = translation_gain * speed_factor
            elif active_movement["pitch_backward"]:
                cmd_pitch = -translation_gain * speed_factor

        if active_movement["yaw_left"]:
            cmd_roll = -translation_gain * speed_factor
        elif active_movement["yaw_right"]:
            cmd_roll = translation_gain * speed_factor

        if active_movement["roll_left"]:
            cmd_yaw = -rotation_gain * speed_factor
        elif active_movement["roll_right"]:
            cmd_yaw = rotation_gain * speed_factor


        base_throttle = hover + cmd_throttle

        m1_val = base_throttle + cmd_pitch - cmd_roll - cmd_yaw
        m2_val = base_throttle + cmd_pitch + cmd_roll + cmd_yaw
        m3_val = base_throttle - cmd_pitch - cmd_roll + cmd_yaw
        m4_val = base_throttle - cmd_pitch + cmd_roll - cmd_yaw

        m1.setVelocity(-m1_val)
        m2.setVelocity( m2_val)
        m3.setVelocity(-m3_val)
        m4.setVelocity( m4_val)

        # Debug output
        if step_count % 60 == 0:
            active_list = [k for k, v in active_movement.items() if v]
            mode = "ALTITUDE" if fsr_active else "NORMAL"
            print(f"[glove_ctrl] step {step_count} | Mode: {mode}")
            print(f"  Angles: yaw={yaw_corr:.1f}° pitch={pitch_corr:.1f}° roll={roll_corr:.1f}°")
            print(f"  FSR={latest['fsr']}, Speed: {speed_factor:.2f}, Active: {active_list}")

if __name__ == "__main__":
    main()
