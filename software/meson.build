internal_deps = []

subdir('display')
display_dep = declare_dependency(
    sources: [display_src],
    include_directories: display_inc,
)
internal_deps = internal_deps + [display_dep]

subdir('gpio')
gpio_dep = declare_dependency(sources: gpio_src, include_directories: gpio_inc)
internal_deps = internal_deps + [gpio_dep]

subdir('log')
log_dep = declare_dependency(sources: log_src, include_directories: log_inc)
internal_deps = internal_deps + [log_dep]

subdir('network')
network_dep = declare_dependency(
    sources: network_src,
    include_directories: network_inc,
)
internal_deps = internal_deps + [network_dep]

subdir('spi')
spi_dep = declare_dependency(sources: spi_src, include_directories: spi_inc)
internal_deps = internal_deps + [spi_dep]

subdir('startup')
startup_dep = declare_dependency(
    sources: startup_src,
    include_directories: startup_inc,
)
internal_deps = internal_deps + [startup_dep]

subdir('lvgl_ui')
lvgl_ui_dep = declare_dependency(sources: lvgl_ui_src, include_directories: lvgl_ui_inc)
internal_deps = internal_deps + [lvgl_ui_dep]

subdir('dht22')
dht22_dep = declare_dependency(sources: dht22_src, include_directories: dht22_inc)
internal_deps = internal_deps + [dht22_dep]
