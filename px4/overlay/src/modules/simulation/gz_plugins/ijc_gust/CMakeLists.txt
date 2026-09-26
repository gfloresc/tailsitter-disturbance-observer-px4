project(IjcGustSystem)

add_library(${PROJECT_NAME} SHARED
    IjcGustSystem.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC ${GZ_PLUGIN_TARGET}
    PUBLIC ${GZ_SIM_TARGET}
)
