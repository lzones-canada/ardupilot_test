 #!/usr/bin/env python
import re

# Path to the generated binary file
VERSION_FILE_PATH = "ArduPlane/version.h"

def get_custom_version():
    """Reads the MID_MAJOR, MID_MINOR, MID_PATCH, MID_FW_TYPE from version.h."""
    with open(VERSION_FILE_PATH, 'r') as file:
        content = file.read()

    # Regular expression to match the version information
    version_pattern = re.compile(r'#define MID_MAJOR (\d+)\n#define MID_MINOR (\d+)\n#define MID_PATCH (\d+)')
    match = version_pattern.search(content)
    if match:
        major, minor, patch = map(int, match.groups())
        return major, minor, patch
    else:
        raise ValueError("Could not find version information in version.h")

# Example usage
if __name__ == "__main__":
    # Get the current version
    version = get_custom_version()  # Provide path to the hwdef.h file
    if version:
        # Convert the tuple (0, 5, 0) to a string '0.5.0'
        version_str = ".".join(map(str, version))
        print(f"Current version: {version_str}")
