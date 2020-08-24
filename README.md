# mc\_openrtm

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)

This package is an OpenRTM interface for mc\_rtc

## Dependencies

- [hrpsys-base](https://github.com/fkanehiro/hrpsys-base)
- [mc\_rtc](https://gite.lirmm.fr/multi-contact/mc_rtc)

This package comes with a sample choreonoid project using the JVRC1 robot model. To install this project you need to have [Choreonoid](https://github.com/s-nakaoka/choreonoid) installed as well.

## Usage

Run the `sim_mc.cnoid` project for JVRC, for example:

```bash
$ cd /usr/share/hrpsys/samples/JVRC1
$ choreonoid sim_mc.cnoid
```

The control component is started automatically when you start the dynamic simulation.

Other simulation files provided by default:
- `sim_mc.cnoid`: JVRC1 robot on a flat floor.
- `sim_mc_fixed.cnoid`: JVRC1 robot with a fixed base and no floor.
- `sim_mc_wall.cnoid`: JVRC1 robot in front of a vertical wall. See [the admittance sample tutorial](https://jrl-umi3218.github.io/mc_rtc/tutorials/samples/sample-admittance.html) for example usage.
- `sim_mc_comanoid_staircase.cnoid`: JVRC1 robot in front of a staircase with steps of (height 18.5cm, length: 24cm). See the [lipm_walking_controller](https://github.com/jrl-umi3218/lipm_walking_controller) for a stair climbing controller using this project.

## Making a new project

There are two important components to include in your Choreonoid project:

- The robot VRML model (plugin: Body)
- A simulation script (plugin: PythonSimScript)

The simulation script instantiates real-time components and binds their ports
together. Most of them are standard components from hrpsys (sequence player,
state holder, data logger, etc.).
