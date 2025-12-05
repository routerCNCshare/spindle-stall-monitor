# spindle-stall-monitor
CNC SPINDLE STALL DETECTION SYSTEM
This is a DIY system to detect a stall condition by monitoring the CNC spindle rpm, and the inverter demand rpm, and then triggers an e-stop if the values are too far apart.
Hopefully this will prevent you breaking the end mill if stalls in the workpiece!

INTENDED SPINDLE
As released the sensor and the 3D printed parts are intended for use on a 1.5kW water cooled 'Chinese' spindle with an 80mm diameter spindle main body.
For other spindles (2kW, 3kW etc.) it would be wise to download the 3D printed main sensor housing to confirm it will fit (or decide if you need to modify parts of the housing to suit your application)
I will try to upload the STEP files (for 3D printing) and the IGES files (so you can load into your favourite CAD programme as a basis to create a custom part)

INTENDED INVERTER / VFD
The inverter signal is configured to run from a Hyuanyang 1.5kW VFD using the 10V PWM output from the V0 and ACM outputs.
Other similar low cost inverters with a 10V PWM output are likely to work
