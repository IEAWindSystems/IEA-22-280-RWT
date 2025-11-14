'''
Update the ROSCO tuning yaml minimum pitch schedule so it's consistent with the minimum pitch schedule of the HAWC2 controller
Run this before update_rosco_discons.py

'''
import os
from rosco.toolbox.ofTools.fast_io.update_discons import update_discons
import numpy as np
from rosco.toolbox.tune import yaml_to_objs
import matplotlib.pyplot as plt
from rosco.toolbox.ofTools.util.FileTools import save_yaml
from rosco.toolbox.inputs.validation import load_rosco_yaml
from rosco.toolbox.utilities import read_DISCON
import pandas as pd



# Add DTU min pitch values


if __name__=="__main__":

    # directory references
    this_dir = os.path.dirname(os.path.abspath(__file__))
    of_dir = os.path.realpath(os.path.join(this_dir,'..'))


    # For each yaml, incorporate the DTU min pitch with the desired level of peak shaving
    # paths relative to this OpenFAST directory
    # List of yamls
    iea22_yamls = [
        'IEA-22-280-RWT-Monopile/IEA-22-280-RWT-Monopile.yaml',
        'IEA-22-280-RWT-Semi/IEA-22-280-RWT-Semi.yaml',
    ]

    # Get DTU min pitch
    df_minpitch = pd.read_csv(os.path.join(of_dir,'../HAWC2/IEA-22-280-RWT-Semi/control/wpdata.100'),sep=' ',header=None)

    dtu_min_pitch_table = df_minpitch.iloc[1:15,0:2].to_numpy()

    # remove last entry of dtu table at 50 m/s
    dtu_min_pitch_table = dtu_min_pitch_table[:-1]

    for yaml in iea22_yamls:
        yaml_file = os.path.join(of_dir,yaml)
        controller, turbine, _ = yaml_to_objs(yaml_file)
        inps = load_rosco_yaml(yaml_file)

        # Change peak shaving
        controller.ps_percent = 0.8
        controller.tune_controller(turbine)   # re-tune

        # Plot minimum pitch schedule
        fig, ax = plt.subplots(1,1)
        # ax.plot(controller.v, controller.pitch_op,label='Steady State Operation')
        ax.plot(controller.v, np.degrees(controller.ps_min_bld_pitch), label='Original ROSCO Pitch Schedule')
        ax.plot(dtu_min_pitch_table[:,0],dtu_min_pitch_table[:,1], label='DTU Initial Conditions',linestyle=':')
        ax.legend()

        if False:  # Method 1: concatenate tables, somewhat manually
            vv = controller.v
            first_ps_ind = np.where(controller.ps_min_bld_pitch > 0)[0][0]



            last_dtu_ind = np.where(vv>dtu_min_pitch_table[-1][0])[0][0]
            dtu_min_pitch = np.radians(np.interp(vv,dtu_min_pitch_table[:,0],dtu_min_pitch_table[:,1])[:last_dtu_ind])
            # min_pitch = np.radians(dtu_min_pitch)
            ps_min_pitch = controller.ps_min_bld_pitch[last_dtu_ind:]

            controller.ps_min_bld_pitch = np.r_[dtu_min_pitch,ps_min_pitch]

            # make sure that controller.ps_min_bld_pitch increases after a certain wind speed
            # manually drop indices between the following values so min pitch is somewhat smooth:
            u_drop = [11.1,13]
            ind_drop = np.bitwise_and(vv > u_drop[0], vv < u_drop[1])
            controller.ps_min_bld_pitch = controller.ps_min_bld_pitch[~ind_drop] 
            controller.v = controller.v[~ind_drop]
        else:  # Method 2: merge lists, take maximum
            vv = controller.v   # rosco toolbox wind speeds
            use_dtu_inds = dtu_min_pitch_table[:,0] < turbine.v_rated   # only use DTU min pitch table up to rated, after that, the meaning is different        

            dtu_min_pitch = np.radians(np.interp(vv,dtu_min_pitch_table[use_dtu_inds,0],dtu_min_pitch_table[use_dtu_inds,1]))

            max_min_pitch = np.maximum(dtu_min_pitch,controller.ps_min_bld_pitch )
            controller.ps_min_bld_pitch = max_min_pitch

            # drop indices between the following values so min pitch is somewhat smooth:
            u_drop = [11.1,13]
            ind_drop = np.bitwise_and(vv > u_drop[0], vv < u_drop[1])
            controller.ps_min_bld_pitch = controller.ps_min_bld_pitch[~ind_drop] 
            controller.v = controller.v[~ind_drop]

        # check that controller.v is non-decreasing
        assert(all(np.diff(controller.v) > 0))
       
        ax.plot(controller.v, np.degrees(controller.ps_min_bld_pitch), label='Updated Pitch Schedule',linestyle='--')
        ax.set_xlabel('Wind speed (m/s)')
        ax.set_ylabel('Blade pitch (rad)')

        discon_vt = read_DISCON('/Users/dzalkind/Projects/IEA-22MW/IEA-22-280-RWT/OpenFAST/IEA-22-280-RWT-Semi/IEA-22-280-RWT-Semi_DISCON.IN')
        ax.plot(discon_vt['PS_WindSpeeds'], np.degrees(discon_vt['PS_BldPitchMin']), label='Previous DISCON Min. Pitch Schedule',linestyle='-.')
        # df = pd.read_csv('/Users/dzalkind/Projects/IEA-22MW/IEA-22-280-RWT/HAWC2/IEA-22-280-RWT-Semi/control/wpdata.100',sep=' ',header=None)

        
        ax.legend()
        plt.savefig(os.path.join(this_dir,os.path.split(yaml)[0]+f'{controller.ps_percent}.peakshave.png'))


        # Update yaml
        inps['controller_params']['DISCON']['PS_WindSpeeds'] = controller.v
        inps['controller_params']['DISCON']['PS_BldPitchMin'] = controller.ps_min_bld_pitch
        inps['controller_params']['DISCON']['PS_BldPitchMin_N'] = len(controller.v)
        save_yaml(os.path.dirname(yaml_file), os.path.split(yaml_file)[-1]+f'.{controller.ps_percent}', inps)

