cmake_minimum_required(VERSION 3.5)

set(srcs
		"group_bulk_read.c"
		"group_bulk_write.c"
		"group_sync_read.c"
		"group_sync_write.c"
		"packet_handler.c"
		"port_handler.c"
		"port_handler_esp_idf.c"
		"protocol1_packet_handler.c"
		"protocol2_packet_handler.c"
		)

# everything needs the ESP32 flag, not sure why this won't work
# going to hack by adding the ESP32 define in the h file
	#`target_compile_options(${COMPONENT_LIB} PRIVATE "-DESP32")

	idf_component_register(SRCS "${srcs}"
			INCLUDE_DIRS "./include"  )
