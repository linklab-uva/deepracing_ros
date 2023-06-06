uint8 session_type                      # 0 = unknown, 1 = P1, 2 = P2, 3 = P3, 4 = Short P, 5 = Q1
                                            # 6 = Q2, 7 = Q3, 8 = Short Q, 9 = OSQ, 10 = R, 11 = R2
                                            # 12 = Time Trial
uint8 time_offset                       # Time in minutes the forecast is for
uint8 weather                           # Weather - 0 = clear, 1 = light cloud, 2 = overcast
                                            # 3 = light rain, 4 = heavy rain, 5 = storm
int8 track_temperature                  # Track temp. in degrees celsius
int8 air_temperature                    # Air temp. in degrees celsius
