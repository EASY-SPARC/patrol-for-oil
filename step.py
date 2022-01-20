#!/usr/bin/env python
"""
Script to test GNOME with long island sound data
"""

import os
import sys

import shutil
from datetime import datetime, timedelta

import numpy as np

from gnome import scripting
from gnome.basic_types import datetime_value_2d
from gnome.utilities.remote_data import get_datafile

from gnome.model import Model

from gnome.maps import MapFromBNA
from gnome.environment import Wind, Tide
from gnome.spill import (point_line_release_spill,
                         InitElemsFromFile,
                         Spill)

from gnome.spill.release import release_from_splot_data


from gnome.spill_container import SpillContainer

from gnome.persist import load

from gnome.spill.substance import GnomeOil

from gnome.spill.initializers import plume_initializers

from gnome.utilities.distributions import UniformDistribution


#from gnome.spill import (Release,
#                         PointLineRelease,
#                         GridRelease,
#                         InitElemsFromFile,
#                         Spill)

from gnome.movers import RandomMover, GridCurrentMover,  GridWindMover

from gnome.outputters import Renderer
from gnome.outputters import NetCDFOutput

from gnome import utilities

# define base directory
base_dir = os.path.dirname(__file__)

def make_model(images_dir=os.path.join(base_dir, 'images2')):
    print('initializing the model')

    start_time = datetime(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]))
    mapfile = get_datafile(os.path.join(base_dir, './brazil-coast.bna'))

    gnome_map = MapFromBNA(mapfile, refloat_halflife=6)  # hours

    # # the image output renderer
    # global renderer
    #duration = timedelta(minutes=5)
    #timestep = timedelta(minutes=5)
    duration = timedelta(minutes=5)
    timestep = timedelta(minutes=5)
    endtime = start_time + duration

    steps = duration.total_seconds()/timestep.total_seconds()

    print("Total step: %.4i " % (steps))

    model = Model(start_time=start_time,
                  duration=duration, time_step=timestep,
                  map=gnome_map, uncertain=False, cache_enabled=False)

    oil_name = 'GENERIC MEDIUM CRUDE'

    wd = UniformDistribution(low=.0002,
                             high=.0002)

    subs = GnomeOil(oil_name,initializers=plume_initializers(distribution=wd))

    #print 'adding a spill'
    #spill = point_line_release_spill(num_elements=122,
    #                                 start_position=(-35.14,
    #                                                 -9.40, 0.0),
    #                                 release_time=start_time)
    #model.spills += spill

    #spill2 = spatial_release_spill(-35.14,-9.40, 0.0, start_time)
    #model.spills += spill2

    #print 'load nc'
    #netcdf_file = os.path.join(base_dir, 'maceio.nc')
    #relnc = InitElemsFromFile(netcdf_file,release_time=start_time)
    #relnc = InitElemsFromFile(netcdf_file,index=5)
    #spillnc = Spill(release=relnc)
    #print spillnc.release.num_elements
    #print spillnc.release.name
    #print spillnc.substance
    #print relnc._init_data['age']
    #print relnc.release_time
 
    #model.spills += spillnc

    #model._load_spill_data()

    #for sc in model.spills.items():
    #    sc.prepare_for_model_run()

    #print(relnc.num_elements)
    #print(relnc.num_released)

    # add particles - it works
    print('adding particles')
    # Persistent oil spill in contiguous zone border
    if int(sys.argv[6]) == 1:
        release = release_from_splot_data(start_time,
                                          'contiguous.txt')
        print("Adding new particles")
        model.spills += Spill(release=release, substance=subs)

    # Particles from previows simulation step
    try:
        f = open('step.txt')
        f.close()
        release2 = release_from_splot_data(start_time,
                                        'step.txt')
        model.spills += Spill(release=release2, substance=subs)        
    except IOError:
        print('No previous step, using only contiguous.txt')

    #assert rel.num_elements == exp_num_elems
    #assert len(rel.start_position) == exp_num_elems
    #cumsum = np.cumsum(exp)
    #for ix in xrange(len(cumsum) - 1):
    #    assert np.all(rel.start_position[cumsum[ix]] ==
    #                  rel.start_position[cumsum[ix]:cumsum[ix + 1]])
    #assert np.all(rel.start_position[0] == rel.start_position[:cumsum[0]])

    #spnc = Spill(release=None)
    #spnc.release = relnc

    print('adding a RandomMover:')
    #model.movers += RandomMover(diffusion_coef=10000, uncertain_factor=2)
    model.movers += RandomMover(diffusion_coef=10000)

    print('adding a current mover:')

    # # this is HYCOM currents
    curr_file = get_datafile(os.path.join(base_dir, 'corrente15a28de09.nc'))
    model.movers += GridCurrentMover(curr_file, num_method='Euler')

    print('adding a grid wind mover:')
    wind_file = get_datafile(os.path.join(base_dir, 'vento15a28de09.nc'))
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
    renderer.viewport = ((-55, -34), (-30, 5)) #1/4 N alagoas
    model.outputters += renderer
    
    netcdf_file = os.path.join(base_dir, 'step.nc')
    scripting.remove_netcdf(netcdf_file)
    model.outputters += NetCDFOutput(netcdf_file, which_data='standard', surface_conc='kde')

    return model

if __name__ == '__main__':
    scripting.make_images_dir()

    model = make_model()

    for step in model:
        #print step
        print("step: %.4i -- memuse: %fMB" % (step['step_num'],
                                              utilities.get_mem_use()))

    # # the image output renderer
    # global renderer

    # one hour timestep
    
    #model = Model.load_savefile('maceio.zip')

    #print 'adding outputters'
    #images_dir=os.path.join(base_dir, 'images')
    #model.outputters += Renderer(mapfile, images_dir, image_size=(800, 600))

    #model.spills = {}

    #model.start_time=datetime(2020, 9, 15, 14, 0)
    #model.duration = timedelta(hours=1)
 
    #model.full_run()
    #post_run(model)
    #print model.current_time_step
    #model.save('maceiolloaded.zip')
    #model._save_spill_data('.', 'spills_data_arrays.nc')