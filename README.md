[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.10944127.svg)](https://doi.org/10.5281/zenodo.10944127)

# IEA-22-280-RWT
This repository contains the model data for the 22-MW offshore reference turbine developed within IEA Wind Task 55 [REFWIND](https://iea-wind.org/task55/).

The documentation for the turbine is available here: https://doi.org/10.11581/DTU.00000317.

Data in this repository includes:
* Documentation, including tabular data used in the figures from the technical report
* [OpenFAST](https://github.com/OpenFAST/openfast.git) aeroelastic model inputs
* [HAWC2](https://www.hawc2.dk/) aeroelastic model inputs
* [WISDEM](https://github.com/WISDEM/WISDEM) optimization files
* [WindIO](https://windio.readthedocs.io/en/latest/) turbine ontology .yaml files
* CAD modeling of turbine where available

## Requirements

*OpenFAST*:
* Please check the release notes for OpenFAST version compatibility.  OpenFAST can be compiled [from source here](https://github.com/OpenFAST/openfast.git) or precompiled Windows binaries are [available for download](https://github.com/OpenFAST/openfast/releases/latest/download/windows_openfast_binaries.zip). More information on installing and running OpenFAST is available in [OpenFAST documention](https://openfast.readthedocs.io/en/master/).
* NREL's Reference OpenSource Controller (ROSCO) is required.  This can be compiled [from source here](https://github.com/nrel/rosco) or precompiled binaries for all platforms are [available for download](https://github.com/NREL/ROSCO/releases/).  The version of the ROSCO controller can be found in the header of the [DISCON.IN](https://github.com/IEAWindTask37/IEA-22-280-RWT/blob/e76514e59c8a152c35a46248559e3787693a5ff5/OpenFAST/IEA-22-280-RWT-Monopile/IEA-22-280-RWT_DISCON.IN#L2)

*HAWC2*:
* HAWC2 can be acquired from its [homepage](https://www.hawc2.dk/).  The DTU Basic Controller can be obtained from its [repository](https://gitlab.windenergy.dtu.dk/OpenLAC/BasicDTUController).

*WISDEM*:
 * WISDEM can be installed from its Github [repository](https://github.com/WISDEM/WISDEM).
 * See the [documentation](https://wisdem.readthedocs.io) for installation and usage guides.

## Citations

If you use this model in your research or publications, please cite:

Zahle, F., Barlas, A., Lønbæk, K., Bortolotti, P., Zalkind, D., Wang, L., Labuschagne, C., Sethuraman, L., & Barter, G. (2024). Definition of the IEA Wind 22-Megawatt Offshore Reference Wind Turbine. Technical University of Denmark, International Energy Agency. DTU Wind Report E-0243, https://doi.org/10.11581/DTU.00000317

```
@techreport{iea22mwrwt,
    author = {Zahle, Frederik and Barlas, Athanasios and Lønbæk, Kenneth and Bortolotti, Pietro and Zalkind, Daniel and Wang, Lu and Labuschagne, Casper and Sethuraman, Latha and Barter, Garrett},
    title = "{Definition of the IEA Wind 22-Megawatt Offshore Reference Wind Turbine}",
    institution = "Technical University of Denmark, International Energy Agency",
    number = {DTU Wind Report E-0243, https://doi.org/10.11581/DTU.00000317},
    isbn = {978-87-87335-71-3},
    year = 2024
}

@dataset{iea22mwrwt-dataset,
  author       = {Zahle, Frederik and
                  Barlas, Thanasis and
                  Lønbæk, Kenneth and
                  Bortolotti, Pietro and
                  Zalkind, Daniel and
                  Lu Wang and
                  Labuschagne, Casper and
                  Sethuraman, Latha and
                  Garrett Barter and
                  Marten, David},
  title        = {IEAWindTask37/IEA-22-280-RWT: v1.0.1},
  month        = apr,
  year         = 2024,
  publisher    = {Zenodo},
  version      = {v1.0.1},
  doi          = {10.5281/zenodo.10944127},
  url          = {https://doi.org/10.5281/zenodo.10944127}
}

```


## Community Contributions

The authors of the IEA 22 MW RWT hope that the turbine is used widely. Notable studies adopting the turbine will be listed here, together with models of the turbine implemented in other codes:

*  A Bladed model was developed by [DNV](mailto:renewables.support@dnv.com) and is available as part of the Bladed v4.16 installation. DNV have also written a Python [converter tool](https://dnvgldocs.azureedge.net/BladedManual/4_16/workflow/preProcessingTools/WindIOtoBladed.html) for the WindIO blade definition to Bladed format for the IEA 22 MW RWT.
* A [QBlade](https://qblade.org) model was developed by researchers at [TU Berlin](https://qblade.org/contact/) and is available in this repository. 
* An OrcaFlex model of the [v1.0.1](https://github.com/IEAWindTask37/IEA-22-280-RWT/releases/tag/v1.0.1) 22 MW rotor, nacelle, and tower (no foundation) was developed by Orcina and verified against OpenFAST. The report and model are available on the validation [page](https://www.orcina.com/resources/validation/) of the Orcina website. 


## Acknowledgments

The IEA 22 offshore reference wind turbine was made possible thanks to countless inputs from a long list of collaborators. A non exhaustive list includes Mayank Chetan and Emmanuel Branlard at NREL, William Collier and Dilek Ors at DNVGL, David Marten at TU Berlin, and Georg Pirrung, David Verelst and Katherine Dykes at DTU.
DTU's participation in IEA Wind Task 37 and Task 55 is sponsored by the Danish Energy Agency under contract no. EUDP-64019-0588 and EUDP-134234-512032.
We also wish to acknowledge Equinor for sponsoring a co-financed research project, which formed the basis for the IEA 22 MW RWT rotor design.
A portion of this research was performed using computational resources sponsored by the Department of Energy's Office of Energy Efficiency and Renewable Energy and located at the National Renewable Energy Laboratory. This work was authored in part by the National Renewable Energy Laboratory, operated by Alliance for Sustainable Energy, LLC, for the U.S. Department of Energy (DOE) under Contract No. DE-AC36-08GO28308. Funding provided by the U.S. Department of Energy Office of Energy Efficiency and Renewable Energy, Wind Energy Technologies Office. 
