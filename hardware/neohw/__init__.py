"""NeoRadio hardware: circuits as code (SKiDL) + ngspice simulation."""
from skidl.logger import stop_log_file_output

# `import skidl` opens <script>.log and <script>.erc in the working directory;
# this closes and deletes them so neither the CLI nor the tests leave clutter.
stop_log_file_output()
