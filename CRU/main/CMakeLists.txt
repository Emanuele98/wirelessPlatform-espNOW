set(srcs    "main.c"
            "cru_hw.c"
            "lis3dh.c"
            )
set(include_dirs include)

idf_component_register(SRCS           "${srcs}"
                       INCLUDE_DIRS   "${include_dirs}")
