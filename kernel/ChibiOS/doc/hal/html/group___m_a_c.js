var group___m_a_c =
[
    [ "MACConfig", "struct_m_a_c_config.html", [
      [ "mac_address", "struct_m_a_c_config.html#a9383f4a6d8552430e20bcc743b1cdee7", null ]
    ] ],
    [ "MACDriver", "struct_m_a_c_driver.html", [
      [ "state", "struct_m_a_c_driver.html#ac60feaf846f5e51054b1b9d0569c4d31", null ],
      [ "config", "struct_m_a_c_driver.html#ae8939c9f6be60a4ce209fe8b2526282d", null ],
      [ "tdqueue", "struct_m_a_c_driver.html#aea7416c287be4a4aac7f8a895021e49a", null ],
      [ "rdqueue", "struct_m_a_c_driver.html#a56d96a6d16db9367d49ab8d6b3dde539", null ],
      [ "rdevent", "struct_m_a_c_driver.html#a7191b8b9ce9305dbbd508ec318a8d6d7", null ]
    ] ],
    [ "MACTransmitDescriptor", "struct_m_a_c_transmit_descriptor.html", [
      [ "offset", "struct_m_a_c_transmit_descriptor.html#a6e7a3f1d34c3d4d7e30de34af42bae6d", null ],
      [ "size", "struct_m_a_c_transmit_descriptor.html#ab5e0d09a410e7d4e2dbf0908c32955b6", null ]
    ] ],
    [ "MACReceiveDescriptor", "struct_m_a_c_receive_descriptor.html", [
      [ "offset", "struct_m_a_c_receive_descriptor.html#ab229471bd1b06ebec6c866dd0eff1b5e", null ],
      [ "size", "struct_m_a_c_receive_descriptor.html#ad36c0d8b599dce5743aa47ea1cbdbe9f", null ]
    ] ],
    [ "MAC_USE_ZERO_COPY", "group___m_a_c.html#gad763d9426413cf2fe3922ebb8578bc59", null ],
    [ "MAC_USE_EVENTS", "group___m_a_c.html#ga887da1c1383a9e7b45c4464877a7e05e", null ],
    [ "macGetReceiveEventSource", "group___m_a_c.html#ga927277e434de4aa3e2831aeeccb5e5a7", null ],
    [ "macWriteTransmitDescriptor", "group___m_a_c.html#ga306157ceb1009737b7e9d7c74d00ae8a", null ],
    [ "macReadReceiveDescriptor", "group___m_a_c.html#ga51d154fd3f0a904886f22c63e1ff87ba", null ],
    [ "macGetNextTransmitBuffer", "group___m_a_c.html#gab5327899902d4a7b20e685375dba83e0", null ],
    [ "macGetNextReceiveBuffer", "group___m_a_c.html#ga72f7a5b2fcd77d3aab0bee963c09ea0d", null ],
    [ "MAC_SUPPORTS_ZERO_COPY", "group___m_a_c.html#gae6398bfe23e4fc8986e1baf0d4d98445", null ],
    [ "PLATFORM_MAC_USE_MAC1", "group___m_a_c.html#ga786e47ccd845e6fa09f6aab36e2b4865", null ],
    [ "MACDriver", "group___m_a_c.html#ga3aac9f212d59486b44672224838c0682", null ],
    [ "macstate_t", "group___m_a_c.html#ga25874e6c823560823e9525983226fe93", [
      [ "MAC_UNINIT", "group___m_a_c.html#gga25874e6c823560823e9525983226fe93a8e12024b63992abd5d7f7139eeedf9f9", null ],
      [ "MAC_STOP", "group___m_a_c.html#gga25874e6c823560823e9525983226fe93ace4baf3e1eb6d2896c332064404f7656", null ],
      [ "MAC_ACTIVE", "group___m_a_c.html#gga25874e6c823560823e9525983226fe93a6bb160c95d9833165d720b1617e39203", null ]
    ] ],
    [ "macInit", "group___m_a_c.html#ga2f76703cfc76da57feedb57e9d37c8a4", null ],
    [ "macObjectInit", "group___m_a_c.html#ga34d60b00ad9794d52da6833e18c9ec70", null ],
    [ "macStart", "group___m_a_c.html#ga60efeea4e7d078afc84a122de9c73f53", null ],
    [ "macStop", "group___m_a_c.html#ga244744ac12c755238cfc2a02cbf6564a", null ],
    [ "macWaitTransmitDescriptor", "group___m_a_c.html#ga64a9b25102b9b51ea62bef5c0984e2d0", null ],
    [ "macReleaseTransmitDescriptor", "group___m_a_c.html#ga0f692163094614561eace2a8c30938f9", null ],
    [ "macWaitReceiveDescriptor", "group___m_a_c.html#ga610978e0feba9ab67b32ca3633d4ba60", null ],
    [ "macReleaseReceiveDescriptor", "group___m_a_c.html#ga4c6470b965aef1ee65bc6dd24c71bc5c", null ],
    [ "macPollLinkStatus", "group___m_a_c.html#gadf64472e0a71dead707f66cd9edabe2a", null ],
    [ "mac_lld_init", "group___m_a_c.html#ga37e276cbed823fb80fdce3df3f9f7ddf", null ],
    [ "mac_lld_start", "group___m_a_c.html#gac692ef12e2b14afd329e350501b529cb", null ],
    [ "mac_lld_stop", "group___m_a_c.html#ga911fa66cf2aa37f1c981fbf74d140565", null ],
    [ "mac_lld_get_transmit_descriptor", "group___m_a_c.html#ga34d066d0a473e0720569cbcb1b5bb90f", null ],
    [ "mac_lld_release_transmit_descriptor", "group___m_a_c.html#ga29cd5df4ce571b7088c7107cf7901330", null ],
    [ "mac_lld_get_receive_descriptor", "group___m_a_c.html#gac17ca421177418f544234918cf83e409", null ],
    [ "mac_lld_release_receive_descriptor", "group___m_a_c.html#gaa04797c813452cbc715d4233e4c44f49", null ],
    [ "mac_lld_poll_link_status", "group___m_a_c.html#gadace88fa6d5051c6c56e068b029dfee8", null ],
    [ "mac_lld_write_transmit_descriptor", "group___m_a_c.html#gafb8c9ab832baf5a87dfeeed680192c73", null ],
    [ "mac_lld_read_receive_descriptor", "group___m_a_c.html#ga2104ce3254db587b7a85dcd128d3cab7", null ],
    [ "mac_lld_get_next_transmit_buffer", "group___m_a_c.html#ga6a13b0c37ee27d46e76e5d230ba97de3", null ],
    [ "mac_lld_get_next_receive_buffer", "group___m_a_c.html#ga21b2f7b535cc088881c4ee6aa773adf1", null ],
    [ "ETHD1", "group___m_a_c.html#gab6c7e189b6b5a76c35734f77af4bc44b", null ]
];