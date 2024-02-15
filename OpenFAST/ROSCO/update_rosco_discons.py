'''
Update the DISCON.IN examples in the IEA-22MW repository using tuning .yamls in rosco conda environment

'''
import os
from rosco.toolbox.ofTools.fast_io.update_discons import update_discons


if __name__=="__main__":

    # directory references
    this_dir = os.path.dirname(os.path.abspath(__file__))
    of_dir = os.path.realpath(os.path.join(this_dir,'..'))

    # paths relative to this OpenFAST directory
    map_rel = {
        'IEA-22-280-RWT-Monopile/IEA-22-280-RWT-Monopile.yaml': 'IEA-22-280-RWT-Monopile/IEA-22-280-RWT_DISCON.IN',
        'IEA-22-280-RWT-Semi/IEA-22-280-RWT-Semi.yaml': 'IEA-22-280-RWT-Semi/IEA-22-280-RWT-Semi_DISCON.IN',
    }

    # Make paths absolute
    map_abs = {}
    for tune, test in map_rel.items():
        tune = os.path.join(of_dir,tune)
        map_abs[tune] = os.path.join(of_dir,test)

    # Make discons    
    update_discons(map_abs)
