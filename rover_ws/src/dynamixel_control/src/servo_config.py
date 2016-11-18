import math

# home position in encoder ticks for the servo.

servo_param = {
    1: {                        # Default for new servo.  Please issue 'new_servo.write_id(new_id)' and setup your own home position!
        'home_encoder': 0x7FF,
        'max_ang': math.radians(360.0),
        'min_ang': math.radians(-360.0),
        'max_speed': math.radians(360*1.75)
        # 'home_encoder': 8190,
        # 'max_ang': math.radians(720.0),
        # 'min_ang': math.radians(-720.0),
        # 'max_encoder': 16380
       }, 
    2: {                        # Flop 90 each way
        'home_encoder': 0x7FF,
        'max_ang': math.radians(360.0),
        'min_ang': math.radians(-360.0),
        'max_speed': math.radians(360*1.75),
        'flipped': 1
       }, 
}

