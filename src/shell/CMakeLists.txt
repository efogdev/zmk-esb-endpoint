target_sources_ifdef(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY app PRIVATE shell_relay.c)

# shell_relay.c includes esb_transport.h from src/esb/
target_include_directories_ifdef(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../esb)
