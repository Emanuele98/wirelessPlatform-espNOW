set(srcs    "main.c"
            "peer.c"
            "wifi.c"
            "espnow.c"
            )
set(include_dirs include)

idf_component_register(SRCS           "${srcs}"
                       INCLUDE_DIRS   "${include_dirs}")