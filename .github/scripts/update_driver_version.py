"""
update_platformio_deps.py

Updates the motorgo-mini-driver dependency in a platformio.ini file to point to a specified new path.

Usage:
    python update_platformio_deps.py <path to platformio.ini> <new driver path>

Arguments:
    ini_file: The path to the platformio.ini file that needs to be updated.
    new_driver_path: The new path or URL to the motorgo-mini-driver dependency.

This script searches for the motorgo-mini-driver dependency in the specified platformio.ini file
and updates its path to the one provided. It handles the dependency specified as a GitHub URL.

Example:
    python update_platformio_deps.py ./platformio.ini "path/to/local/motorgo-mini-driver"
"""

import click
import configparser
from typing import AnyStr
import re

@click.command()
@click.argument('ini_file', type=click.Path(exists=True))
@click.argument('new_driver_path')
def update_lib_deps(ini_file: AnyStr, new_driver_path: AnyStr):
    """
    Main command function that updates the motorgo-mini-driver dependency in the platformio.ini file.

    :param ini_file: Path to the platformio.ini file.
    :param new_driver_path: New path for the motorgo-mini-driver dependency.
    """
    config = configparser.ConfigParser()
    config.read(ini_file)

    # Iterate through each section of the INI file to find and update the 'lib_deps' entry
    for section in config.sections():
        if 'lib_deps' in config[section]:
            lib_deps = config[section]['lib_deps']
            new_lib_deps = update_motorgo_mini_driver_version(lib_deps, new_driver_path)
            config[section]['lib_deps'] = new_lib_deps

    # Write the updated configuration back to the platformio.ini file
    with open(ini_file, 'w') as configfile:
        config.write(configfile)

def update_motorgo_mini_driver_version(lib_deps: AnyStr, new_driver_path: AnyStr) -> AnyStr:
    """
    Updates the motorgo-mini-driver version or path in the lib_deps string.

    :param lib_deps: The original lib_deps string from platformio.ini.
    :param new_driver_path: The new path to replace the motorgo-mini-driver dependency with.
    :return: Updated lib_deps string.
    """
    # Pattern to find the motorgo-mini-driver dependency, capturing HTTPS URLs specifically for GitHub
    pattern = re.compile(r"https?://github\.com/Every-Flavor-Robotics/motorgo-mini-driver(?:\.git)?(#\S+)?", re.IGNORECASE)

    # Print a message describing the replacement
    print(f"Updating motorgo-mini-driver from {re.search(pattern, lib_deps).group()} to {new_driver_path}")

    # Replace the matched motorgo-mini-driver URL or path with the new one provided
    updated_lib_deps = re.sub(pattern, new_driver_path, lib_deps)


    return updated_lib_deps

if __name__ == '__main__':
    update_lib_deps()
