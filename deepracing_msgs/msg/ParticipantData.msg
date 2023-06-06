uint8 ai_controlled           # Whether the vehicle is AI (1) or Human (0) controlled
uint8 driver_id               # Driver id - see appendix
uint8 team_id                 # Team id - see appendix
uint8 race_number             # Race number of the car
uint8 nationality            # Nationality of the driver
string name                   # Name of participant in UTF-8 format – null terminated
                                   # Will be truncated with … (U+2026) if too long
uint8 your_telemetry          # The player's UDP setting, 0 = restricted, 1 = public
