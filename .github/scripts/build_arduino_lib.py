import os
import subprocess
import json
import click

def print_dependency_summary(dependencies):
    """Prints a formatted summary of dependencies and actions taken.

    Args:
        dependencies (list): A list of dependency dictionaries as parsed from library.json.
    """

    print("\n--- Dependency Summary ---")
    for dep in dependencies:
        name = dep.get("name")
        owner = dep.get("owner")
        version = dep.get("version")

        if name and (owner or version.startswith(("http", "git"))):
            dep_string = f"{owner}/{name}@{version}" if owner else version
            print(f'\033[92mInstalling: {dep_string}\033[0m')  # Green = Installing
        elif name:
            print(f'\033[93mSkipping: {name}  (Likely a system dependency)\033[0m')  # Yellow =  Skipping
        else:
            print(f'\033[91mWarning: Invalid dependency format: {dep}\033[0m')  # Red = Warning

    print("--------------------------\n")

def format_dependency_string(dependency):
    """Formats a single dependency entry for PlatformIO installation.

    Args:
        dependency (dict): A dictionary representing a single dependency
                           from the library.json file.

    Returns:
        str: The formatted dependency string ready for use with 'pio pkg install'.
        None:  If the dependency is likely a system dependency and should be skipped.
    """

    if dependency.get("version") and dependency["version"].startswith(("http", "git")):
        # Dependency is sourced from a Git repository
        return f'{dependency["version"]}'
    elif dependency.get("name") and dependency.get("owner"):
        # Name-version formatted dependency (e.g.,  "library_name@version")
        return f'{dependency["owner"]}/{dependency["name"]}@{dependency.get("version")}'
    elif dependency.get("name"):
        # Has a name but no owner or version --> Likely a system dependency
        print(f'Skipping dependency "{dependency["name"]}" - likely a system package.')
        return None  # Indicate we should skip installation of this dependency
    else:
        # Invalid dependency format
        print(f'Warning: Invalid dependency format: {dependency}')
        return None


@click.command()
@click.argument('path_to_library_json', type=click.Path(exists=True))
@click.option('--storage-dir', default="./temp_deps", help="The directory to install the dependencies to.")
def install_pio_dependencies(path_to_library_json, storage_dir):
    """Installs dependencies specified in a PlatformIO library.json file.

    Args:
        path_to_library_json (str): Path to the library.json file.
    """

    with open(path_to_library_json, 'r') as f:
        data = json.load(f)

    dependencies = data.get("dependencies", [])
    if not dependencies:
        print("No dependencies found in library.json")
        return

    # Create the storage directory if it doesn't exist
    if not os.path.exists(storage_dir):
        os.makedirs(storage_dir)

    # Construct the base PIO installation command list
    pio_install_command = ["pio", "pkg", "install", "--global", "--storage-dir", storage_dir]

    # Iterate and build dependency arguments
    for dep in (format_dependency_string(dep) for dep in dependencies):
        if dep:  # Check if dependency is valid (not skipped)
            print(dep)
            # Add -l and dependency to the command list
            pio_install_command.append("--library")
            pio_install_command.append(dep)

    print(f"Installing dependencies with command: {pio_install_command}")

    print_dependency_summary(dependencies)

    # Run PlatformIO package installation
    result = subprocess.run(pio_install_command, capture_output=True)

    # Check for errors
    if result.returncode != 0:
        print(f"Error: PlatformIO package installation failed with exit code {result.returncode}")
        print(result.stderr.decode())
        return
    else:
        # Print in Green
        print(f"\033[92mDependencies installed successfully!\033[0m")

if __name__ == "__main__":
    install_pio_dependencies()





