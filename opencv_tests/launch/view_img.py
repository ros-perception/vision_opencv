from launch.exit_handler import default_exit_handler, restart_exit_handler
from ros2run.api import get_executable_path


def launch(launch_descriptor, argv):
    ld = launch_descriptor
    package = 'image_tools'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='showimage'),
        '-t', '/opencv_tests/images'],
        name='showimage',
        exit_handler=restart_exit_handler,
    )
    package = 'opencv_tests'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='source.py')],
        name='source.py',
        exit_handler=restart_exit_handler,
    )

    return ld
