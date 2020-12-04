#!/usr/bin/env python
"""
Script to test GNOME with long island sound data
"""

import os
import shutil
from datetime import datetime, timedelta

import numpy as np

from pykml import parser

from gnome import scripting
from gnome.basic_types import datetime_value_2d
from gnome.utilities.remote_data import get_datafile

from gnome.model import Model

from gnome.spill.substance import GnomeOil

from gnome.maps import MapFromBNA
from gnome.environment import Wind, Tide
from gnome.spill import (point_line_release_spill,
                         InitElemsFromFile,
                         SpatialRelease,
                         Spill,
                         grid_spill)

from gnome.spill.release import release_from_splot_data


from gnome.spill_container import SpillContainer

from gnome.spill.substance import GnomeOil
from gnome.spill.initializers import plume_initializers
from gnome.utilities.distributions import WeibullDistribution, UniformDistribution

from gnome import utilities

#from gnome.spill import (Release,
#                         PointLineRelease,
#                         GridRelease,
#                         InitElemsFromFile,
#                         Spill)

from gnome.movers import RandomMover, WindMover, CatsMover, GridCurrentMover,  GridWindMover

from gnome.outputters import Renderer, NetCDFOutput, KMZOutput, ShapeOutput

# define base directory
base_dir = os.path.dirname(__file__)

if __name__ == '__main__':
    print 'get contiguous'

    kml_file = os.path.join(base_dir, 'contigua.kml')
    with open(kml_file) as f:
        contiguous = parser.parse(f).getroot().Document

    coordinates = contiguous.Placemark.LineString.coordinates.text.split(' ')
    cont_coord=[]
    for x in coordinates:
        x = x.split(',')
        if len(x) > 1 and float(x[1]) > -11.5 and float(x[1]) < -8.5:
            cont_coord.append([float(x[0]), float(x[1])])

        
    with open('contiguous.txt', 'w') as out_file:
            
        for idx in range(0, len(cont_coord),2):
            out_file.write(str(cont_coord[idx][0]) + '\t' + str(cont_coord[idx][1]) + '\t' + '1' + '\n')