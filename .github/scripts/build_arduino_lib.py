import os
import subprocess
import json
import click
import shutil
from typing import Optional
import os
import json

# Iterate over installed dependency directories and package
def check_platform(lib_dir, safe = True):
    library_json_path = os.path.join(lib_dir, "library.json")

    if not os.path.exists(library_json_path):
        if safe:
            return False
        else:
            return True

    with open(library_json_path, 'r') as f:
        data = json.load(f)

    platform = data.get("platforms", [])
    if isinstance(platform, list):
        if "espressif32" in platform:
            return True
    elif platform == "espressif32":
        return True

    return False

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

def format_dependency_string(dependency: dict) -> Optional[str]:
    """Formats a single dependency entry for PlatformIO installation.

    Args:
        dependency (dict): A dictionary representing a single dependency
                                    from the library.json file.

    Returns:
        str: The formatted dependency string ready for use with 'pio pkg install'.
        None: If the dependency is likely a system dependency and should be skipped.
                This indicates that the installation of this dependency should be skipped.
                A message will be printed indicating that the dependency will be skipped.
                The formatted dependency string will not be returned.
        None: If the dependency has an invalid format.
                This indicates that the installation of this dependency should be skipped.
                A warning message will be printed indicating that the dependency has an invalid format.
                The formatted dependency string will not be returned.

    Notes:
        The dependency dictionary can have the following structures:
        1. If the dependency has a version and the version starts with "http" or "git",
            it means the dependency is sourced from a Git repository.
            Example: {"version": "https://github.com/owner/repo.git"}
        2. If the dependency has a repository URL and a version, format it as "repository#version".
            Example: {"repository": "https://github.com/owner/repo", "version": "1.0.0"}
        3. If the dependency has a repository URL but no version, use the repository URL as the dependency string.
            Example: {"repository": "https://github.com/owner/repo"}
        4. If the dependency has a name and an owner, format it as "owner/name@version".
            Example: {"name": "library", "owner": "owner", "version": "1.0.0"}
        5. If the dependency has a name but no owner or version, it is likely a system dependency.
            The installation of this dependency will be skipped.
            Example: {"name": "library"}
        6. If the dependency has an invalid format, a warning message will be printed.
          The installation of this dependency will be skipped.
          Example: {"invalid": "dependency"}
    """

    if dependency.get("version") and dependency["version"].startswith(("http", "git")):
        # If the dependency has a version and the version starts with "http" or "git",
        # it means the dependency is sourced from a Git repository.
        return f'{dependency["version"]}'
    elif dependency.get("repository"):
        # If the dependency has a repository URL, use it as the dependency string.
        version = dependency.get("version")
        if version and version != "*":
            return f'{dependency["repository"]}#{version}'
        else:
            return f'{dependency["repository"]}'
    elif dependency.get("name") and dependency.get("owner"):
        # If the dependency has a name and an owner, format it as "owner/name@version".
        return f'{dependency["owner"]}/{dependency["name"]}@{dependency.get("version")}'
    elif dependency.get("name"):
        # If the dependency has a name but no owner or version, it is likely a system dependency.
        # Print a message indicating that the dependency will be skipped.
        print(f'Skipping dependency "{dependency["name"]}" - likely a system package.')
        return None  # Indicate that we should skip installation of this dependency
    else:
        # If the dependency has an invalid format, print a warning message.
        print(f'Warning: Invalid dependency format: {dependency}')
        return None

def merge_directories(src, dst):
    if not os.path.exists(src):
        print("\033[93mWarning: Source directory '{}' does not exist.\033[0m".format(src))
        return

    for item in os.listdir(src):
        src_path = os.path.join(src, item)
        dst_path = os.path.join(dst, item)

        if os.path.isdir(src_path):
            shutil.copytree(src_path, dst_path, dirs_exist_ok=True)  # Merge nested directories
        else:
            shutil.copy2(src_path, dst_path)  # Copy individual files

def copy_source_files(pio_lib_path, package_output_dir):
    """Copies source files from a PlatformIO-style library into an output directory.

    Args:
        pio_lib_path (str): Path to the root of the PlatformIO library.
        package_output_dir (str):  The output directory where files will be placed.
    """

    src_dir = os.path.join(pio_lib_path, "src")
    include_dir = os.path.join(pio_lib_path, "include")
    output_src_dir = os.path.join(package_output_dir, "src")

    os.makedirs(output_src_dir, exist_ok=True)  # Ensure 'src' output directory exists

    merge_directories(include_dir, output_src_dir)
    merge_directories(src_dir, output_src_dir)


@click.command()
@click.argument('path_to_library_json', type=click.Path(exists=True))
@click.option('--storage-dir', default="./temp_deps", help="The directory to install the dependencies to.")
@click.option('--output-dir', default="./output", help="Output directory for packaging the driver.")  # New Click option
@click.argument('repo_name')  # The argument to receive the repository name
def install_pio_dependencies(path_to_library_json, storage_dir, output_dir, repo_name):
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

    # Packaging Logic
    package_output_dir = os.path.join(output_dir, repo_name)
    os.makedirs(package_output_dir, exist_ok=True)  # Ensure the main output directory exists



    # Iterate over installed dependency directories and package
    for lib_dir in os.listdir(storage_dir):
        lib_path = os.path.join(storage_dir, lib_dir)
        if os.path.isdir(lib_path) and check_platform(lib_path, safe = False):
            copy_source_files(lib_path, package_output_dir)
        else:
            # Print in Yellow
            print(f"\033[93mSkipping: {lib_dir}  (Not an ESP32 PlatformIO library or no longer exists)\033[0m")

    # Strip library.json from path_to_library_json
    current_repo_path = os.path.dirname(path_to_library_json)
    copy_source_files(current_repo_path, package_output_dir)

    # Copy library.json and library.properties to root of the output_dir
    shutil.copy2(path_to_library_json, package_output_dir)
    shutil.copy2(os.path.join(current_repo_path, "library.properties"), package_output_dir)

    print(f'\033[92mDependencies installed and packaged into: {package_output_dir}\033[0m')

if __name__ == "__main__":
    install_pio_dependencies()





