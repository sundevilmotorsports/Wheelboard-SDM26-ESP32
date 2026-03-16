idf_component_register()
execute_process(
    COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/fix.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)