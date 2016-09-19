sawOptoforceSensor
==================

cisst/SAW component to interface via USB (virtual COM port) to Optoforce force sensor.

*Windows:*

 + Requires a virtual COM driver from Microchip, which is available on the USB stick provided by Optoforce
   (see /DAQ Material/USB/Driver, if not automatically installed by Windows).

*Linux:*

 + Linux automatically creates serial device, such as `ttyACM0`.
 + Need to run as "root" to access serial port.
 + Example:

   ```c++
   sudo ./OptoforceExample /dev/ttyACM0  OMD-10-SE-10N.json
   ```

The software expects a JSON format configuration file, which should specify the force sensor scale and
(optionally) a calibration matrix, `cal-matrix`. See the sample JSON configuration files in the `share` folder.
Thus, it is advised that cisst be compiled with JSON support (`CISST_HAS_JSON` CMake option). The component
can still be used without JSON support, however, in which case `scale` is initialized to 1.0 and can be
changed after the component is started by invoking the `SetScale` write command.

This component supports both the semi-spherical and the flat-top 3D force sensors. Note, however, that
these are 3D sensors and only measure force.

For the flat-top sensors, if you attach a bracket and apply a force somewhere along the bracket, the sensor
measurement will be a coupling of the forces and torques. If you know where the force is applied, it is possible
to decouple the measurements and extract the forces. The software component supports this, assuming that
the JSON file specifies the correct `cal-matrix` and you specify where the forces are applied by invoking
the `SetLength` write command.

The final calibration  matrix for OMD-45-FE-1000N sensor can be found in share/calibration_matrix_OMD-45-FE-1000N_sensor.matrix_30.json

Links
=====

 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/
 
Dependencies
============
 * Linux, Mac OS, Windows, ...
 * cisst libraries: https://github.com/jhu-cisst/cisst
