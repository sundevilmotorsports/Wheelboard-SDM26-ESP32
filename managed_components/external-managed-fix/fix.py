#!/usr/bin/env python3
"""
ESP-IDF Component Manager Helper Script - Scan All External Libraries

Automatically generates minimal CMakeLists.txt files for all libraries
in managed_components/ that do not already have one.
"""

import os
import sys
from glob import glob

def create_cmake(component_path: str, src_dir: str = "src", include_dir: str = "inc", verbose: bool = True):
    """
    Creates a minimal CMakeLists.txt in the component folder.

    Args:
        component_path: Path to the library folder (managed_components/<lib>)
        src_dir: Subdirectory containing source files
        include_dir: Subdirectory containing headers
        verbose: Enables status messages
    """
    cmake_file = os.path.join(component_path, "CMakeLists.txt")

    if not os.path.isdir(component_path):
        if verbose:
            print(f"-- MANAGED FIX: Component path '{component_path}' does not exist. Skipping.")
        return

    if os.path.exists(cmake_file):
        if verbose:
            print(f"-- MANAGED FIX: CMakeLists.txt already exists for '{component_path}'. Skipping.")
        return

    src_path = os.path.join(component_path, src_dir)
    c_files = glob(os.path.join(src_path, "*.c"))

    c_files_line = ""
    for file in c_files:
        c_files_line += f"    SRCS \"{src_dir}/{file.split('src' + os.sep, 1)[1]}\"\n"

    with open(cmake_file, "w") as f:
        f.write("idf_component_register(\n")
        f.write(c_files_line)
        f.write(f'    INCLUDE_DIRS "{include_dir}"\n')
        f.write(")\n")

    if verbose:
        print(f"-- MANAGED FIX: Created CMakeLists.txt for '{component_path}'")

def scan_all_managed(root_path="managed_components", src_dir="src", include_dir="inc", verbose=False):
    """
    Scan all folders in managed_components and generate CMakeLists.txt if missing.
    """
    if not os.path.isdir(root_path):
        if verbose:
            print(f"-- MANAGED FIX: managed_components folder '{root_path}' does not exist.")
        return

    for folder in os.listdir(root_path):
        comp_path = os.path.join(root_path, folder)
        if os.path.isdir(comp_path):
            create_cmake(comp_path, src_dir, include_dir, verbose)

if __name__ == "__main__":
    scan_all_managed(verbose=True)