set(COMPONENT_SRCS
    "main.c"
    "mcp2518fd.c"
)

set(COMPONENT_INCLUDES ".")

if(IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/../3rd/mlx90642-library/inc")
    list(APPEND COMPONENT_SRCS "../3rd/mlx90642-library/src/MLX90642.c")
    list(APPEND COMPONENT_INCLUDES "../3rd/mlx90642-library/inc")
else()
    message(WARNING "mlx90642-library not found under ../3rd; building without MLX90642 support")
endif()

idf_component_register(SRCS ${COMPONENT_SRCS}
                       INCLUDE_DIRS ${COMPONENT_INCLUDES})
