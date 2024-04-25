internal_deps = []

subdir('display')
display_dep = declare_dependency(include_directories: display_inc)
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
