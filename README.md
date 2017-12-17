# extended-kalman-filter
Project 2 of term 2 of Udacity self-driving car nanodegree

## Setup

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

Follow instructions [here](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project) to set up Ubuntu and Mac environments. Also, copy src/main\_mac\_linux.cpp to src/main.cpp . 

Instructions for 
setting up a native Windows environment and Visual Studio project:

* follow steps 1-4 of the instructions [here](https://github.com/fkeidel/CarND-Term2-ide-profile-VisualStudio/tree/master/VisualStudio)
* change install-windows.bat :
    - replace "EKF" with "UKF"
    - delete row  "copy main.cpp ..\..\src" 
* run install-windows.bat 
* open solution in VisualStudio, remove from the solution all files from project 1, add all files from project 2
* run steps 6 and 7 from [here](https://github.com/fkeidel/CarND-Term2-ide-profile-VisualStudio/tree/master/VisualStudio)

## Files
* src - folder of source files.
* data - folder with input dataset.
* ide_profiles - folder with IDE profiles for Ecliple and XCode.
* Filter Calibration.ipynb - Jupyter notebook that shows graphs of normalized innovation score. 
* laser\_nis\_file.txt, radar\_nis\_file.txt - files with normalized innovation scores of laser and radar sensors.  

## Usage
1. Set up project environment for your operating system.
2. Compile the files to create the main executable of the project (the name of executable depends on the operating system).
3. Run Term 2 Simulator. 
4. Run the main executable of the project.
