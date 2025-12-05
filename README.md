# spindle-stall-monitor
CNC SPINDLE STALL DETECTION SYSTEM

This is a DIY system to detect a stall condition by monitoring the CNC spindle rpm, and the inverter demand rpm, and then triggering an e-stop if the values are too far apart.
Hopefully this will prevent you breaking an end mill if it stalls in the workpiece!  I should mention 2 things at this point:

1- This is my first ever GitHub project so I'm learning how to upload and share things

2- Machine tools are dangerous so take all the usual precautions.  Monitoring the machine at all times and don't relying on this e-stop system.



INTENDED SPINDLE

As released the sensor and the 3D printed parts are intended for use on a 1.5kW water cooled 'Chinese' spindle with an 80mm diameter spindle main body.
For other spindles (2kW, 3kW etc.) it would be wise to download the 3D printed main sensor housing to confirm it will fit before you go ahead with the project (or decide if you need to modify parts of the housing to suit your application).  Along with the STEP files (for 3D printing ) I will try to upload the IGES files so you can load into your favourite CAD programme as a basis to create a custom part.



INTENDED INVERTER / VFD

The inverter signal is configured to run from a Huanyang 1.5kW VFD using the 10V PWM output from the V0 and ACM outputs.  Other similar low cost inverters with a 10V PWM output are likely to work.



SKILL LEVEL
I would place this as a medium/advanced skill level since you will need to be able to:

Order your own PCBs (using the Gerber files provided)

Order you own electronic components using the BOM list

Be able to solder components using the completed PCB pictures provided

Make up custom wiring looms to suit your application (cable between main board and display, cable between main board and sensor)

Be able to diagnose any issues along the way

Have access to a reasonable 3D printer



HELP

As a one man hobbyist with a day job and family I may be able to some help but it will be limited help (best posted on the YouTube video comments for this video series)


Have fun!
