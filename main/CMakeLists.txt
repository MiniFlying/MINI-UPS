idf_component_register(
    SRCS "tusb_hid_example_main.c"
    INCLUDE_DIRS "."
    REQUIRES freertos tinyusb
    PRIV_REQUIRES nvs_flash
)