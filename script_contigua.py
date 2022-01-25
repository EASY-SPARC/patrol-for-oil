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

from gnome.movers import RandomMover, WindMover, CatsMover, GridCurrentMover,  GridWindMover

from gnome.outputters import Renderer, NetCDFOutput, KMZOutput, ShapeOutput

# define base directory
base_dir = os.path.dirname(__file__)

def make_model(images_dir=os.path.join(base_dir, 'images')):

    print('get contiguous')

    kml_file = os.path.join(base_dir, 'contigua.kml')
    with open(kml_file) as f:
        contiguous = parser.parse(f).getroot().Document

    coordinates = contiguous.Placemark.LineString.coordinates.text.split(' ')
    cont_coord=[]
    for x in coordinates:
        x = x.split(',')
        if len(x) > 1 and float(x[1]) > -12 and float(x[1]) < -3:
            cont_coord.append([float(x[0]), float(x[1])])

    print('initializing the model')

    start_time = datetime(2022, 1, 22, 12, 0)
    mapfile = get_datafile(os.path.join(base_dir, './brazil-coast.BNA'))

    gnome_map = MapFromBNA(mapfile, refloat_halflife=6)  # hours

    duration = timedelta(days=1)
    timestep = timedelta(minutes=5)
    end_time = start_time + duration


    steps = duration.total_seconds()/timestep.total_seconds()

    print("Total step: %.4i " % (steps))

    # one hour timestep
    model = Model(start_time=start_time,
                  duration=duration, time_step=timestep,
                  map=gnome_map, uncertain=False, cache_enabled=False)    

    oil_name = 'GENERIC MEDIUM CRUDE'

    wd = UniformDistribution(low=.0002,
                             high=.0002)

    subs = GnomeOil(oil_name,initializers=plume_initializers(distribution=wd))

    #model.spills += point_line_release_spill(release_time=start_time, start_position=(-35.153, -8.999, 0.0), num_elements=1000, end_release_time=end_time, substance= subs, units='kg')
    #model.spills += point_line_release_spill(release_time=start_time, start_position=(-35.176, -9.135, 0.0), num_elements=1000, end_release_time=end_time, substance= subs, units='kg')
    #model.spills += point_line_release_spill(release_time=start_time, start_position=(-35.062, -9.112, 0.0), num_elements=1000, end_release_time=end_time, substance= subs, units='kg')
    #model.spills += point_line_release_spill(release_time=start_time, start_position=(-34.994, -9.248, 0.0), num_elements=1000, end_release_time=end_time, substance= subs, units='kg')

    for idx in range(0, len(cont_coord)):
        model.spills += point_line_release_spill(num_elements=500, start_position=(cont_coord[idx][0],cont_coord[idx][1], 0.0),
                                         release_time=start_time,
                                         end_release_time=start_time + timedelta(days=1),
                                         amount=500,
                                         substance = subs,
                                        units='kg')
    
    #shp_file = os.path.join(base_dir, 'surface_concentration')
    #scripting.remove_netcdf(shp_file + ".zip")
    #model.outputters += ShapeOutput(shp_file,
    #                                zip_output=False,
    #                                surface_conc="kde",
    #                                )

    print('adding movers:')

    print('adding a RandomMover:')
    model.movers += RandomMover(diffusion_coef=10000)

    print('adding a current mover:')

    # # this is HYCOM currents
    curr_file = get_datafile(os.path.join(base_dir, 'currents.nc'))
    model.movers += GridCurrentMover(curr_file, num_method='Euler')

    print('adding a grid wind mover:')
    wind_file = get_datafile(os.path.join(base_dir, 'wind.nc'))
    #topology_file = get_datafile(os.path.join(base_dir, 'WindSpeedDirSubsetTop.dat'))
    #w_mover = GridWindMover(wind_file, topology_file)
    w_mover = GridWindMover(wind_file)
    w_mover.uncertain_speed_scale = 1
    w_mover.uncertain_angle_scale = 0.2  # default is .4
    w_mover.wind_scale = 2

    model.movers += w_mover

    print('adding outputters')

    renderer = Renderer(mapfile, images_dir, image_size=(900, 600),
                        output_timestep=timestep,
                        draw_ontop='forecast')
    #set the viewport to zoom in on the map:
    #renderer.viewport = ((-37, -11), (-34, -8)) #alagoas
    renderer.viewport = ((-36, -10), (-30, 5))
    model.outputters += renderer
    
    netcdf_file = os.path.join(base_dir, 'contigua.nc')
    scripting.remove_netcdf(netcdf_file)
    model.outputters += NetCDFOutput(netcdf_file, which_data='standard', surface_conc='kde')

    return model

if __name__ == '__main__':
    scripting.make_images_dir()

    model = make_model() 
    
    #for step in model:
    #    print model.spills.num_released
        #print model.spills.spill_by_index(0).num_elements


    for step in model:
        #print step
        print("step: %.4i -- memuse: %fMB" % (step['step_num'], utilities.get_mem_use()))

    #model.full_run()
    
    #model.save('maceio.zip')
    #model._save_spill_data('.', 'spills_data_arrays.nc')