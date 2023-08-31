
set(srcs    "main.c"
            "aux_ctu_hw.c"
            "led_strip.c"
            "espnow.c"
            )
set(include_dirs include)

idf_component_register(SRCS           "${srcs}"
                       INCLUDE_DIRS   "${include_dirs}")