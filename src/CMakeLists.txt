target_sources_ifdef(CONFIG_ZMK_ESB_ENDPOINT app PRIVATE
    esb_endpoint.c
    timeslot_handler.c
    esb_transport.c
    pairing.c
    hid_relay.c
)

target_sources_ifdef(CONFIG_ZMK_ESB_ENDPOINT app PRIVATE
    input_processor_esb.c
)
