# Chassis-Dynamics

## Overview
This is a chassis simulation tool written purely in MATLAB. It is currently intended to primarily simulate ride dynamics. Note all units are assumed to be in SI.

### Vehicle Model
The chassis itself is a 4-corner car model. It includes suspension components found in most road vehicles.

![Chassis Model](https://github.com/jscans12/chassis-dynamics/blob/main/docs/diagram.png)

The tire is modelled as a spring/damper in parallel. Tire outer diameter is intersected with the road profile. The point of greatest vertical penetration into the circle is used to define ground profile input to the model.

![Tire Model](https://github.com/jscans12/chassis-dynamics/blob/main/docs/tire_model.png)

The following parameters are needed to create a typical simple vehicle model, but some level of front/rear asymmetry is supported by the code if needed:

#### Dimensional
Wheelbase
Track width

#### Inertial
Sprung mass
Sprung moment of inertia
CG location from front axle and ground plane
Front unsprung mass
Rear unsprung mass

#### Aerodynamic
Aerodynamic center of pressure
Aerodynamic scaled coefficients

#### Suspension, Corners
Front spring rate
Rear spring rate
Front damping coefficient
Rear damping coefficient
Front motion ratio
Rear motion ratio

#### Suspension, Roll
Front antiroll rate
Rear antiroll rate

#### Tires
Tire radius
Tire spring rate
Tire damping coefficient

## Prerequisites
You must have MATLAB R2019a or later (earlier versions may work but are untested), and also must have the h5io package on your MATLAB path. The h5io package can be found either on [GitHub](https://github.com/jscans12/h5io) or the [MATLAB FileExchange](https://www.mathworks.com/matlabcentral/fileexchange/126235-h5io).
