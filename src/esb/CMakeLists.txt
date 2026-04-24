target_sources_ifdef(CONFIG_ZMK_ESB_ENDPOINT app PRIVATE
    esb_endpoint.c
    esb_transport.c
    pairing.c
    hid_relay.c
    channel_hop.c
)

target_sources_ifdef(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP app PRIVATE channel_hop_ep.c)

target_sources_ifdef(CONFIG_ZMK_ESB_ENDPOINT app PRIVATE input_processor_esb.c)
target_sources_ifdef(CONFIG_ZMK_ESB_ENDPOINT_BENCH app PRIVATE bench.c)
