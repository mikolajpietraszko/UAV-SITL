# UAV-SITL
A 6 DOF aircraft model developed in Matlab. Interfaced with QGroundControl and Flight Gear via Simulink. Based on RCAM Model created by Christopher Lum. https://www.youtube.com/watch?v=bFFAL9lI2IQ

I made this software for my BEng Aerospace Systems Engineering dissertation. It was awarded "Best Project Award" by RAeS (Royal Aeronautical Society) at Coventry University in 2022. It has also been featured in an open-access article published in the MDPI Automation journal.
The article can be found here https://www.mdpi.com/2673-4052/3/3/25

The software requires Matlab R2021b. 
(It is possible to downgrade it to earlier versions with the GPS sensor subsystem commented out. Works on 2021a. Not sure about the older versions.)

<h1>UAV MODEL</h1>  

<h2>Simulation </h2>

To enjoy the full simulation of the system, you need to first launch QGroundControl and have installed FlightGear 2020.3 (needs to be installed separately). All of the custom FlightGear models are included in FlightGear models directory. Copy them to your Program Files/FlightGear/data/Aircraft directory. FlightGear must be launched by executing one of the .bat files.
Currently there are 3 aircrafts to choose from:

To use the default FlightGear Cessna 172 model execute the runfg.bat file. 

To use a small UAV Rascal 110 (electric version) model execute the runfgrascal.bat file. 

To use the HALE General Atomics RQ-9 Reaper model, execute the runfgmq9.bat file. 


Open the UAVSimulation.slx file to investigate the Simulink model and open the InitializeConstants.m file. The script loads up trim_values_straight_level.mat file which contains trim conditions used by the UAV_model.m as inputs. UAV_model.m is the main function implementing all the flight dynamics of the aircraft.   

<h2>Important! </h2>

The full simulation is only started by running the init.m script. If you try running the simulation by running the UAVSimulation.slx it won’t work as it doesn’t have the correct inputs. If you want to run a second simulation after that you will have the inputs already in the workspace but the simulation still has to be run through init.m script because it initiates the MAVLink HEARTBEAT message which establishes the connection with QGroundControl. If you get any UDP protocol related errors, please restart Matlab and try again. You will most likely get that error if you interrupt the simulation and try to run it again.  

The full simulation setup should look something like this. 


![figure1](https://user-images.githubusercontent.com/97880512/213925845-b6a20fe6-4ecc-48f9-9202-5bc0d4e7b107.png)


<h3>Figure 1 Simulation setup </h3>

After running the init.m it should successfully run. (please make sure GPS SENSOR subsystem is commented out if you use older version of Matlab). 

![figure2](https://user-images.githubusercontent.com/97880512/213925864-bd6b3eb8-5f02-4f73-85ac-6cf8b676dd63.png)


<h3>Figure 2 Running Simulation </h3>

The simulation is running in real time so keep that in mind if you want to use longer simulation time. After the simulation is complete you should receive plots with aircraft data. 

 ![figure3](https://user-images.githubusercontent.com/97880512/213925913-ecd5d77e-3aa5-486c-9133-e09fe0e329eb.png)


<h3>Figure 3 Complete Simulation </h3>

<h2>Trimming </h2>

To use the aircraft trimming open the UAV_trim.m file. Set the initialization value to zero for the first run. After you obtain the trim_values_straight_level.mat file change the initialization value to 1. You can now run the script repeatedly until you receive satisfactory results. Run the ValidateTrimPoint.m script to validate if the trimming works as expected. The trimming however only works with an aircraft in a steady state. It won’t work with the system in its current state because of the position values outputted by the model. However, you can observe how well the aircraft is trimmed in FlightGear by running the simulation in straight level flight (use constant zero as actuator inputs). 

 
