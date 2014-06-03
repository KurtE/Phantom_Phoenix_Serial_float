Warning
=======

This is a WIP to try out converting the Phoenix code base to use floating point math instead of fixed point math.  This is a WIP and is not complete yet!  Currently testing is only being done using the Teensy 3.1.    This version assumes that we are using the standard Hardware Serial objects and not the Arbotix special cased version. 

I know that there are issues on Arduino/Arbotix with the current version as there are some inconsistencies with definitions/usage of variables that are defined PROGMEM. 

Anyone who is not playing along with this conversion should use one of the more stable version such as the Phantom_phoenix

This project is specific to the Trossen Robotics Phantom Phoenix Robots (both Hexapod a Quadrupod) 

This project is a split off of my main project (Arduino_Phoenix_Parts) that I do my work on to support a variety of Hex Robots.  The readme file associated with Quad support of that project gives more details about the different files and compile options (https://github.com/KurtE/Arduino_Phoenix_Parts/tree/Quad-Support).  Soon this the Quad Support branch will be migrated back into the main branch as this has all of the code updates. 
  
Actually this project is forked off of the Phantom_Phoenix project.  What makes this project different is that I am experimenting with talking to the Bioloid AX-12 servos using the Arduino hardware Serial objects (Serial1, Serial2), instead of using custom version of Arduino.  

This appears to work well using the Arduino 1.5.6 release which has updates to the hardware serial code base.  I have tried this out using standard 
Arduino Mega boards, as well as with the Arbotix controller.  I have a version of the Hardware definitions, which does not duplicate the core.   

I have uploaded a zip file with this to the Trossen forums before and may include a version here as well.

In addition, the BioloidSerial project has been updated to work with the Teensy 3.1 and I am using this with my own Teensy breakout boards. 

This project relies on installing one of my other projects BioloidSerial (https://github.com/KurtE/BioloidSerial) , which has the actual version of the Bioloid library that was updated 
to use the actual serial object.

Sometimes this project will be ahead of the other projects, other times it will be behind.  I will try to keep most of these projects in sync. 

General
=======

Most of the code in these libraries are in header files, which allows them to be compiled specifically for each project.   


Installation
============

Note: Hopefully this code is reasonably self-contained and can be located anywhere.  I personally keep this project in my Arduino Sketchbook, such that I can load it, using the Arduino File/Sketchbook menu.   

This code requires that you have also installed the Arbotix extensions to the Arduino environment as well as their libraries.


Major Contributors
==================

Jeroen Janssen [aka Xan] -  The original Phoenix code was written by to run on the Lynxmotion Phoenix 
(http://www.lynxmotion.com/c-117-phoenix.aspx). It was originally written in Basic for the Basic Atom Pro 28
processor by Basic Micro.  

Kare Halvorsen (aka Zenta) -  The Lynxmotion Phoenix was based on the original Phoenix that was developed by
him.  In addition a lot of the software was based off of his earlier Excel spreadsheet (PEP).  More details up on his 
Project page (http://www.lynxmotion.com/images/html/proj098.htm).

Me - I later ported the code to C/C++ and the Arduino environment and with the help of Kåre and Jeroen hopefully 
reduced the number of bugs I introduced as part of this port.   

Michael E. Ferguson (lnxfergy up on Trossen) - Arbotix Commander, Ax12.

Again Warning
=============

This is a WIP - No promises or guarantees!

