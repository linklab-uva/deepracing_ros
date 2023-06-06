uint8 traction_control          # 0 (off) - 2 (high)drs#
uint8 anti_lock_brakes           # 0 (off) - 1 (on)
uint8 fuel_mix                  # Fuel mix - 0 = lean, 1 = standard, 2 = rich, 3 = max
uint8 front_brake_bias           # Front brake bias (percentage)
uint8 pit_limiter_status         # Pit limiter status - 0 = off, 1 = on
float32 fuel_in_tank               # Current fuel mass
float32 fuel_capacity             # Fuel capacity
float32 fuel_remaining_laps        # Fuel remaining in terms of laps (value on MFD)
uint16 max_rpm                   # Cars max RPM, point of rev limiter
uint16 idle_rpm                  # Cars idle RPM
uint8 max_gears                 # Maximum number of gears
uint8 drs_allowed               # 0 = not allowed, 1 = allowed, -1 = unknown
uint16 drs_activation_distance    # 0 = DRS not available, non-zero - DRS will be available
                                        # in [X] metres

uint8[4] tyres_wear             # Tyre wear percentage
uint8 actual_tyre_compound	    # F1 Modern - 16 = C5, 17 = C4, 18 = C3, 19 = C2, 20 = C1
                                    # 7 = inter, 8 = wet
                                    # F1 Classic - 9 = dry, 10 = wet
                                    # F2 – 11 = super soft, 12 = soft, 13 = medium, 14 = hard
                                    # 15 = wet
uint8 visual_tyre_compound        # F1 visual (can be different from actual compound)
                                        # 16 = soft, 17 = medium, 18 = hard, 7 = inter, 8 = wet
                                        # F1 Classic – same as above
                                        # F2 – same as above
uint8 tyres_age_laps             # Age in laps of the current set of tyres
uint8[4] tyres_damage           # Tyre damage (percentage)
uint8 front_left_wing_damage      # Front left wing damage (percentage)
uint8 front_right_wing_damage     # Front right wing damage (percentage)
uint8 rear_wing_damage           # Rear wing damage (percentage)

# Added Beta 3:
uint8 drs_fault                 # Indicator for DRS fault, 0 = OK, 1 = fault

uint8 engine_damage             # Engine damage (percentage)
uint8 gear_box_damage            # Gear box damage (percentage)
int8 vehicle_fia_flags          # -1 = invalid/unknown, 0 = none, 1 = green
                                        # 2 = blue, 3 = yellow, 4 = red
float32 ers_store_energy           # ERS energy store in Joules
uint8 ers_deploy_mode            # ERS deployment mode, 0 = none, 1 = medium
                                        # 2 = overtake, 3 = hotlap
float32 ers_harvested_this_lap_mguk  # ERS energy harvested this lap by MGU-K
float32 ers_harvested_this_lap_mguh  # ERS energy harvested this lap by MGU-H
float32 ers_deployed_this_lap       # ERS energy deployed this lap