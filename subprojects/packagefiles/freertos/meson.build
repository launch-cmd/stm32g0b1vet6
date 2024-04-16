project('freertos', 'c')

freertos_inc = include_directories([
    'include', 
    'portable/GCC/ARM_CM0'
])

freertos_src = files([
    'tasks.c',
    'queue.c',
    'list.c',
    'timers.c',
    'event_groups.c',
    'stream_buffer.c',
    'portable/GCC/ARM_CM0/port.c',
    'portable/MemMang/heap_4.c'
])