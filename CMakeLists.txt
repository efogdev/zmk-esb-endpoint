add_subdirectory(src)
zephyr_include_directories(include)
zephyr_compile_options(-Wno-address-of-packed-member)

if(CONFIG_ZMK_ESB_ENDPOINT)
    zephyr_link_libraries(-Wl,--wrap=bt_le_adv_start)
    zephyr_link_libraries(-Wl,--wrap=bt_le_adv_update_data)
    zephyr_link_libraries(-Wl,--wrap=bt_le_adv_start_legacy)

    set(NRFXLIB_DIR ${CMAKE_CURRENT_LIST_DIR}/vendor/nrfxlib)
    zephyr_include_directories(${NRFXLIB_DIR}/mpsl/include)
    zephyr_include_directories(${NRFXLIB_DIR}/mpsl/include/protocol)
    zephyr_include_directories(${NRFXLIB_DIR}/mpsl/fem/include)
    zephyr_include_directories(${NRFXLIB_DIR}/mpsl/fem/include/protocol)

    if(CONFIG_FPU AND CONFIG_FPU_SHARING)
        set(MPSL_FLOAT_DIR "hard-float")
    elseif(CONFIG_FPU)
        set(MPSL_FLOAT_DIR "softfp-float")
    else()
        set(MPSL_FLOAT_DIR "soft-float")
    endif()

    set(MPSL_LIB     ${NRFXLIB_DIR}/mpsl/lib/nrf52/${MPSL_FLOAT_DIR}/libmpsl.a)
    set(MPSL_FEM_LIB ${NRFXLIB_DIR}/mpsl/fem/common/lib/nrf52/${MPSL_FLOAT_DIR}/libmpsl_fem_common.a)
    zephyr_link_libraries(${MPSL_FEM_LIB})
    zephyr_link_libraries(${MPSL_LIB})

    set(NRF_ESB_DIR ${CMAKE_CURRENT_LIST_DIR}/vendor/nrf-esb)

    zephyr_include_directories(${NRF_ESB_DIR}/include)
    zephyr_include_directories(${NRF_ESB_DIR}/subsys)

    if(CONFIG_HAS_HW_NRF_PPI)
        target_sources(app PRIVATE
            ${NRF_ESB_DIR}/subsys/esb.c
            ${NRF_ESB_DIR}/subsys/esb_ppi.c
        )
    else()
        target_sources(app PRIVATE
            ${NRF_ESB_DIR}/subsys/esb.c
            ${NRF_ESB_DIR}/subsys/esb_dppi.c
        )
    endif()
endif()
