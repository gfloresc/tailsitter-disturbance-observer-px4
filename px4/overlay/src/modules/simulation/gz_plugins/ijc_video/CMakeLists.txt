project(IjcVideoSystem)

add_library(${PROJECT_NAME} SHARED
    IjcVideoSystem.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC ${GZ_PLUGIN_TARGET}
    PUBLIC ${GZ_SIM_TARGET}
)
