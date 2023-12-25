project(
    'icsim',
    'c',
    license: 'GPL-3',
    default_options: [
        'warning_level=2',
    ]
)

find_program('candump', required: true)
deps = [
    dependency('sdl2', required: true),
    dependency('SDL2_image', required: true)
]

bundled_lib = custom_target('copy-lib',
                            output: 'lib.o',
                            input: 'lib.o',
                            command : [ 'cp', '@INPUT@', '@OUTPUT@' ],
)

subdir('art')
subdir('data')

executable('icsim', ['icsim.c', bundled_lib], dependencies: deps)
executable('controls', 'controls.c', dependencies: deps)
