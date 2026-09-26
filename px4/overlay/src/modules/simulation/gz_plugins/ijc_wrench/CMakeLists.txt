project(IjcWrenchSystem)

add_library(${PROJECT_NAME} SHARED
    IjcWrenchSystem.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC ${GZ_PLUGIN_TARGET}
    PUBLIC ${GZ_SIM_TARGET}
    PUBLIC ${GZ_TRANSPORT_TARGET}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(IjcAeroSystem SHARED
    IjcAeroSystem.cpp
)

target_link_libraries(IjcAeroSystem
    PUBLIC ${GZ_PLUGIN_TARGET}
    PUBLIC ${GZ_SIM_TARGET}
    PUBLIC ${GZ_TRANSPORT_TARGET}
)
