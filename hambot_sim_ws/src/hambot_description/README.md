This folder is a descriptor of the whole content of the robot (chassis, wheel, camera, lidar).
It contains all the measurements and offset for each of the elements of the robot.

If you want to just edit the elements that are already set, just edit section 0,
where the specific properties are. The rad (radius), len (length), width, hgt (height), 
joint_{axis} (offset having the center of the robot as the origin point) are the modifiable properties.

The wheels are already inverted in their section, so the front and rear wheels 
are well positioned (no need to change anything when modifying the wheels 
regarding their inversion, just change the offset in section 0). All these units are in meters. 

The mass of each thing can also be 
changed in the section 0 for wheels and chassis. But the others, you gotta go to 
their own links and change the m in xacro:box_inertia. Mass is in kg.