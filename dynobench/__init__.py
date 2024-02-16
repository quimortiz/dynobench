# load the .so file  (should be in the same directory as this file)
from .pydynobench import *

# Export the path to the data directory
from os.path import dirname, join as joinpath

# DATADIR = joinpath(dirname(__file__), "data/")
PKGDIR = dirname(__file__) + "/"
