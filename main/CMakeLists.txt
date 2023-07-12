idf_component_register(SRCS "main.c"
							"speech_commands_action.c"
							"../ui/custom.c"
							"../ui/events_init.c"
							"../ui/gui_guider.c"
							"../ui/lv_font_montserratMedium_12.c"
							"../ui/lv_font_montserratMedium_16.c"
							"../ui/setup_scr_screen.c"
                        INCLUDE_DIRS "."
                        "../components/lvgl"
                        "../components/lvgl_esp32_drivers"
						"../ui"
						"./include"
                        REQUIRES lvgl
								esp-sr
								led_strip
								hardware_driver
								)
