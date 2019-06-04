![alt text](https://www.parrot.com/files/s3fs-public/ps/3495-large-parrot-3495jpg.jpg "Parrot Bebop 2")
[source](https://www.parrot.com)
# System identification for Parrot Bebop 2
This repository contains MATLAB scripts as well as recorded flight data used for system identification purposes on the commercial Parrot Bebop 2 drone.

## Support
For support regarding the usage of this repository, please contact one of the authors

## Authors
Matthijs van der Boon - m.j.vanderboon@student.tudelft.nl  
Pim de Bruin - p.e.debruin@student.tudelft.nl  
Jasper Wiersema - j.d.wiersema@student.tudelft.nl  
Eduard Jongkees - e.p.jongkees@student.tudeft.nl  
Ewoud van Mourik - e.a.vanmourik@student.tudelft.nl

A special thank you is extended to [H. Zhu](https://github.com/hai-zhu) for providing a lot of the code base for running flight experiments.

## Usage

### Data recording
Files used for recording experiment data are located in `/data`. A guide for experiment setup in the DCSC lab can be found [here](https://docs.google.com/document/d/1Ap_kQQK0nCGyiXhGXnmifd_kB4u8_QHbuX7kWRNe9YM/edit). A troubleshoot document is online [here](www.example.com). Running `run_main` starts an experiment. Run `save_log` to save all recorded data after an experiment.

Roll, pitch and vertical velocity input vectors can be set in `modules/CDrone` by setting object properties `m_ppi_phi`, `m_ppi_theta`, and `m_ppi_vz`.

### Data processing
Scripts used for processing recorded data are located in `/processing`. Run `batch` to batch process all recorded data and save experiment data into separate data `.mat` files.
Other scripts in this folder are used to analyse recorded data.

### System identification
Files used for system identification are located in `/systemidentification`. The main workflow to identify new models is as follows.

1. use `batchIdentify` or `CCDInitial` to identify new models using either random initial values, or initial values based on central composite design.
2. For first order models `FONLIFnc` or `FOPTDNIFnc` are used for identification. `GONLIdentification` is used to identify models of higher order.
3. Generated models are saved as `.mat` files.

### Validation
Files used for system identification are located in `/validation`. The workflow for validation is as follows.

1. Pick the best identified model for each data set using `batchPicker`
2. Use `batchValidation` to validate the best models over validation data sets. Batch validation uses `simGOFull` to simulate model responses. The results of validation are saved in `.mat` files.
3. Run `[valddata, simdata] = simGOFull([],[])` to simulate the response of a desired model over a data set. UI dialogs are opened to select model and data.
4. Plot the response of the simulation by running `plotResultAndSave(valdata, simdata)`. Figures can be saved if desired.
