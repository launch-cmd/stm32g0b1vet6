project('m0plus', 'c',
	meson_version: '>=1.3.1',
	default_options: 
	[
		'c_std=c11',
		'debug=true',
		'optimization=0'
	]
)


common_cflags = 	[
						'-gdwarf-5', 							# Enable Dwarf5 debugging standard
						'-ffunction-sections',					# Generate a seperate ELF section for each function in the source file
						'-fdata-sections',						# Generate a seperate ELF section for each variable in the source file
						'-fanalyzer',							# Use various heuristics to attempt to explore the control flow and data flow in the program
						'-fno-common',							# Compiler places uninitialized global variables in the BSS section of the object file
#						'-c',									# Only compile source files into object files, do not link
						'-ffreestanding',						# Standard lib may not exist and startup may not be in 'main' function
						'-fno-builtin',							# Do not replace library functions with buitin compiled code
						'-march=armv6-m',						# The name of the ARM architecture
						'-mcpu=cortex-m0plus.small-multiply',	# The name of the ARM processor
#						'-mtune=',								# The name of the ARM processor for which GCC should tune the performace of the code
#						'-mfpu=',								# The floating-point hardware (or hardware emulation)
						'-mfloat-abi=soft'						# Specifies which floating-point ABI to use	
					]


# LINKER SCRIPT
linker_template = '-T@0@/@1@'
linker_script_name = 'advanced.ld'
linker_script = linker_template.format(meson.project_source_root(), linker_script_name)

# MAP FILE
map_template = '-Wl,-Map=@0@/@1@.elf.map,--cref'
map_file = map_template.format(meson.current_build_dir(), 'm0plus')


common_ldflags = 	[
						linker_script,
						map_file,
						'-Wl,--gc-sections',					# Removes unused symbols like functions and variables
						'-nostdlib',							# Do not use the standard library
						'-mthumb'								# Select between generating code that executes in ARM and Thumb states				
					]
					
add_project_arguments(common_cflags, language: 'c')
add_project_link_arguments(common_ldflags, common_cflags, language: 'c')

# External dependencies
arm_cmsis_6_proj = subproject('arm_cmsis_6')
arm_cmsis_6_inc = arm_cmsis_6_proj.get_variable('arm_cmsis_6_inc')
arm_cmsis_6_dep = declare_dependency(sources: [], include_directories: [arm_cmsis_6_inc])

st_g0_cmsis_proj = subproject('st_g0_cmsis')
st_g0_cmsis_inc = st_g0_cmsis_proj.get_variable('st_g0_cmsis_inc')
st_g0_cmsis_dep = declare_dependency(sources: [], include_directories: [st_g0_cmsis_inc])

freertos_proj = subproject('freertos')
freertos_inc = freertos_proj.get_variable('freertos_inc')
freertos_src = freertos_proj.get_variable('freertos_src')
freertos_dep = declare_dependency(sources: [freertos_src], include_directories: [freertos_inc])

freertos_plus_tcp_proj = subproject('freertos_plus_tcp')
freertos_plus_tcp_inc = freertos_plus_tcp_proj.get_variable('freertos_plus_tcp_inc')
freertos_plus_tcp_src = freertos_plus_tcp_proj.get_variable('freertos__plus_tcp_src')
freertos_plus_tcp_dep = declare_dependency(sources: [freertos_plus_tcp_src], include_directories: [freertos_plus_tcp_inc])

lvgl_proj = subproject('lvgl')
lvgl_inc = lvgl_proj.get_variable('lvgl_inc')
lvgl_dep = declare_dependency(sources: [], include_directories: [lvgl_inc])

# Internal dependencies
subdir('software')

executable('m0plus.elf', 'software/main.c', 
	dependencies: [
		arm_cmsis_6_dep, 
		st_g0_cmsis_dep, 
		freertos_dep,
		freertos_plus_tcp_dep,
		lvgl_dep, 
		internal_deps
])
