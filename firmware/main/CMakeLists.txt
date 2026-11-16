idf_component_register(
    SRCS
        "main.c"
        "can_bus.c"
        "rocon_params.c"
        "led.c"
        "zigbee.c"
        "timesync.c"
        "ble_update.c"
        "ble_log.c"
        "maint.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        bt
        driver
        esp_driver_gpio
        esp_timer
        nvs_flash
        app_update
        espressif__esp-zigbee-lib
)

# Gli header di compatibilità 1.x di esp-zigbee-lib 2.x (include/compat/*.h) emettono
# '#warning "This file is deprecated..."' a ogni inclusione. -Wno-cpp nasconde solo i #warning
# e solo per i file di questo componente; gli altri warning (con -Werror) restano attivi.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-cpp)
