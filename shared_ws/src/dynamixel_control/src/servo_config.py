import math

# home position in encoder ticks for the servo.

servo_param = {
    1: {                        # Default for new servo.  Please issue 'new_servo.write_id(new_id)' and setup your own home position!
        'home_encoder': 2216,
        'max_encoder': 0xFFF*4,
        'max_ang': math.radians(360.0*7),
        'min_ang': math.radians(-360.0*7),
        'max_speed': math.radians(360*1.75)
        # 'home_encoder': 8190,
        # 'max_ang': math.radians(720.0),
        # 'min_ang': math.radians(-720.0),
        # 'max_encoder': 16380
       }, 
    2: {                        # Flop 90 each way
        'home_encoder': 2293,
        'max_encoder': 0xFFF*4,
        'max_ang': math.radians(360.0*7),
        'min_ang': math.radians(-360.0*7),
        'max_speed': math.radians(360*1.75),
        'flipped': 1
       },
    3: {                        # Flop 90 each way
        'home_encoder': 512,
        'rad_per_enc': math.radians(300)/1024.0,
        'max_ang': math.radians(90),
        'min_ang': math.radians(-90),
        'max_speed': math.radians(360*1.75),
        'flipped': 1
       }, 
    4: {                        # Flop 90 each way
        'home_encoder': 312,
        'rad_per_enc': math.radians(300)/1024.0,
        'max_ang': math.radians(207),
        'min_ang': math.radians(-90),
        'max_speed': math.radians(360*1.75),
        'flipped': 0
       }, 
}

