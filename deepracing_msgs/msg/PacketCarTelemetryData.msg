PacketHeader header
CarTelemetryData[22] car_telemetry_data
uint32 button_status
uint8 mfd_panel_index       # Index of MFD panel open - 255 = MFD closed
                                        # Single player, race – 0 = Car setup, 1 = Pits
                                        # 2 = Damage, 3 =  Engine, 4 = Temperatures
                                        # May vary depending on game mode
uint8 mfd_panel_index_secondary_player   
int8 suggested_gear       # Suggested gear for the player (1-8)
                            # 0 if no gear suggested