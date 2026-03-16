idf_component_register(
    SRCS "main.c" "mcp2518fd.c"
    INCLUDE_DIRS "."
    REQUIRES driver log esp_timer freertos mlx90642-lib
)