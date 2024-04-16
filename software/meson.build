internal_deps = []

subdir('gpio')
gpio_dep = declare_dependency(sources: gpio_src, include_directories: gpio_inc)
internal_deps = internal_deps + [gpio_dep]

subdir('startup')
startup_dep = declare_dependency(sources: startup_src, include_directories: startup_inc)
internal_deps = internal_deps + [startup_dep]
