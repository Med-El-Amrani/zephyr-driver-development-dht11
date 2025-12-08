
zephyr_library()
zephyr_library_sources_ifdef(CONFIG_CUSTOM_DHT11 dht11.c)