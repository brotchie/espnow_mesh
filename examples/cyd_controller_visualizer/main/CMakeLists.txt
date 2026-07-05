idf_component_register(
    SRCS "main.c" "mesh_visualizer.c"
    INCLUDE_DIRS "."
    REQUIRES espnow_mesh cyd_4in_st7796_lvgl
)
