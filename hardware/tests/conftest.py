"""SKiDL writes <script>.erc/.log into the working directory when imported.

Move the test session into a throwaway directory before any test module
imports skidl, so the source tree stays clean.
"""
import atexit
import os
import shutil
import tempfile

_scratch = tempfile.mkdtemp(prefix="neohw-tests-")
os.chdir(_scratch)
atexit.register(shutil.rmtree, _scratch, ignore_errors=True)
