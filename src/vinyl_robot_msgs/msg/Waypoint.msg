float64 x_mm
float64 z_mm
float64 a_deg
bool    skip_x
bool    skip_z
bool    skip_a
float64 velocity_scale   # 0.0 = use trajectory default_velocity_scale
uint8   ramp_mode        # 0=position, 1=vel+, 2=vel- (mirrors firmware RPDO byte 7 bits[1:0])
uint8   dmax_factor      # 0=use configured DMAX; 1-63 = dmax_factor/64 x DMAX (blend softness)
