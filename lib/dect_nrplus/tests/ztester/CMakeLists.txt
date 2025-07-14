# cmake_minimum_required(VERSION 3.20.0)
# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
# project(mac_flow)

# # Enable Ztest
# set(ENABLE_ZTEST TRUE)

# # Add test source files
# target_sources(app PRIVATE
#     src/test_mac_flow.c
#     # ../../tests/mocks/mock_nrf_modem_dect_phy.c
#     # ../../src/dect_mac_api.c
#     # Add other source files as needed
# )

# # Include directories
# target_include_directories(app PRIVATE
#     ../tests/mocks
#     ../include
#     ../include/mac
#     ${ZEPHYR_BASE}/include
# )

# # Link Ztest library
# target_link_libraries(app PRIVATE ztest)


cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mac_flow)  # or project(dect_nrplus_driver)


# target_sources(app PRIVATE ${ZEPHYR_BASE}/tests/drivers/sensor/temp_sensor/src/main.c)

# Add source files
target_sources(app PRIVATE
    src/test_mac_flow.c  # For mac_flow tests
    # C:/ncs/v3-zettalink/nrf/lib/dect_nrplus/tests/mocks/mock_nrf_modem_dect_phy.c
    # ../dect_mac_api.c    
    # C:/ncs/v3-zettalink/nrf/lib/dect_nrplus/src/dect_mac_api.c
    # Add other source files as needed
)

# Include directories
target_include_directories(app PRIVATE
    # ${ZEPHYR_BASE}/include
    # ${ZEPHYR_BASE}/include/zephyr
    # ${ZEPHYR_BASE}/subsys/testsuite/include  # For ztest.h

    ../mocks
    # C:/ncs/v3-zettalink/nrf/lib/dect_nrplus/include
    # C:/ncs/v3-zettalink/nrf/lib/dect_nrplus/include/mac


    # C:/ncs/v3-zettalink/nrf/lib/dect_nrplus/tests/mocks
    # C:/ncs/v3-zettalink/nrf/lib/dect_nrplus/include
    # C:/ncs/v3-zettalink/nrf/lib/dect_nrplus/include/mac
)

# Link Ztest for test projects
# target_link_libraries(app PRIVATE ztest)