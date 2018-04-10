RRRRRRRRRRRRRRRRR                    VVVVVVVV           VVVVVVVV  iiii  
R::::::::::::::::R                   V::::::V           V::::::V i::::i 
R::::::RRRRRR:::::R                  V::::::V           V::::::V  iiii  
RR:::::R     R:::::R                 V::::::V           V::::::V        
  R::::R     R:::::R   ooooooooooo    V:::::V           V:::::V iiiiiii 
  R::::R     R:::::R oo:::::::::::oo   V:::::V         V:::::V  i:::::i 
  R::::RRRRRR:::::R o:::::::::::::::o   V:::::V       V:::::V    i::::i 
  R:::::::::::::RR  o:::::ooooo:::::o    V:::::V     V:::::V     i::::i 
  R::::RRRRRR:::::R o::::o     o::::o     V:::::V   V:::::V      i::::i 
  R::::R     R:::::Ro::::o     o::::o      V:::::V V:::::V       i::::i 
  R::::R     R:::::Ro::::o     o::::o       V:::::V:::::V        i::::i 
  R::::R     R:::::Ro::::o     o::::o        V:::::::::V         i::::i 
RR:::::R     R:::::Ro:::::ooooo:::::o         V:::::::V         i::::::i
R::::::R     R:::::Ro:::::::::::::::o          V:::::V          i::::::i
R::::::R     R:::::R oo:::::::::::oo            V:::V           i::::::i
RRRRRRRR     RRRRRRR   ooooooooooo               VVV            iiiiiiii

*********************************************
***Content of project
*********************************************
This project file contains the following Folders
File name		Description
Robotics 
 - SamplePluginPA10	- Robwork plugin
 - PA10WorkCell		- Robwork workcell
 - data			- Experimental data 

Vision
 - Image-1		- Vision for marker 1
 - FeachureExtracton	- Vision for marker 3
 - Markers		- Markers and training data


*********************************************
***Setup Robwork plugin
*********************************************
First up open SamplePluginPA10/src/SamplePlugin.cpp and edit the variable PATH_TO_FILE so that i matches your complete path to the SamplePluginPA10 and end with a "/". 
To modify the Marker Sequence change the define SEQUENCE to either SLOWSEQ, MEDISEQ or FASTSEQ.   
To run the code as specified in the project description part 4, set the defines ENABLE_VI_SER to 0, and POINTS to either 1 or 3. 
To run the code as specified in the project description part 5, set the define ENABLE_VI_SER to 1, and POINTS 3.

Next up compile the robwork plugin navigate to SamplePluginPA10/build in the terminal and run the filowing commands:
	
	cmake ..	
	make

The plugin can now be imported thorough RoboWork by selecting the SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so. Now to run the simulation open the SamplePlugin by clicking the Sp Icon that now have appeared. A new window named DockWidget should now have appeare. Click the Initialize button to set up the invironment, and then click the Run button to start the simulation. 

*********************************************
***Setup Vision projects
*********************************************
Setting up Vision for marker 1
******************************


Setting up Vision for marker 3
******************************
Through the terminal navigate to FeachureExtracton/build, if no build folder exists, make one and go there. From this folder run the commands:

	cmake ..
	make

The code can now be run by the following command:

	./FeachureExtracton






















 


