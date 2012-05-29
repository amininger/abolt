-----------------------------------------------------------
- April Bolt
- 5/17/12
- Code organization of the april bolt system
- Edited by Aaron Mininger
-----------------------------------------------------------

<------------>
 Running Bolt
<------------>
To run:
make sure april.jar, lcm.jar, and abolt.jar are on $CLASSPATH

java abolt.bolt.Bolt -c /path/to/config

Arguments that can be passed to Bolt:
-h/--help       (bool)   Shows help information for Bolt
-c/--config     (file)   REQUIRED: the config file used by Bolt (files for classification and calibration)
-w/--world      (file)   File used to initialize the simulator
-s/--sim-config (file)   Config file used for the simulator
-k/--kinect     (bool)   If true, Bolt creates objects using data from the kinect
  /--seg        (bool)   If true, the display shows the camera feed and segmentation instead (only used if -k is true)
-d/--debug      (bool)   If true, extra debugging information is shown
  /--fps        (int)    The desired fps for the simulator to run


<----------------->
 Code Organization
<----------------->

abolt.arm 		- Code for controlling the robotic arm
abolt.bolt 		- Contains the main class (Bolt) and GUI's
abolt.classify 	- Code for performing the classification and feature extraction
abolt.collision - New code for intersections convex solids
abolt.kinect 	- Code for the perceptual system, including segmentation, tracking, and calibration
abolt.lcm 		- contains plugins for lcm-spy to use
abolt.lcmtypes 	- java interfaces for the lcm types used in the system
abolt.objects 	- object representations (both sim and real-world) and their collections
abolt.sim 		- objects and interfaces used in the simulator
abolt.util 		- useful utility classes
abolt.vis 		- other vis objects useful in user intefaces
  

<------------------->
 System Architecture
<------------------->
Bolt is the main class that controls how the system is created and what components are used
Bolt contains static methods used for getting itself (getSingleton()) and its components
Bolt also initializes the components for the arm, either the ArmSimulator or the actual BoltArmController
Bolt automatically creates and sends observations_t over LCM

Bolt Components:
	BoltObjectManager - A collection of the BoltObjects in the current environment
		Major tasks are updating the objects with new perceptual information and reclassifying
		Also constructs the object_data_t[] array used in observations_t
	SensableManager - A collection of the SimSensables in the current environment
		Major task is responding to actions performed on a sensable
		Also constructs the string[] array used in observations_t
	ClassifierManager - A collection of the various classifiers used for different visual features
		This is where the settings for each classifier can be tweaked
	IBoltGUI - The user interface being shown to the user
		Contains a VisWorld and handles various drawing functionality for that world
	IBoltCamera - The camera extracting point cloud data for the objects 
	Segment (optional) - The segmenter currently in use (only if using --kinect)

Object vs Sensable
	The distinction between an object and a sensible is somewhat contrived 
		and is mostly distinguished by how its represented in observations_t
	Object (BoltObject) 
		Right now objects are those that are created in the observations_t as object_data_t
		They are either objects found in the segmentation of kinect data, or those created in the simulator
		These you cannot perform actions on (like open or turn on) and are those that can be grabbed
	Sensable (SimSensable)
		A sensable is a purely simulated object which is created in observations_t as a sensable string
		These can have actions performed on them (like open or turn on) but cannot be grabbed



	



