# Impedance and Admittance Control Simulation

This repository contains MATLAB simulations for impedance control and admittance control systems, which are fundamental control strategies in robotics and mechatronics.

## Overview

The project demonstrates two different force control approaches:

- **Impedance Control**: Regulates the relationship between force and position by controlling the mechanical impedance of the system
- **Admittance Control**: Regulates the relationship between position and force by controlling the mechanical admittance of the system

## Files Description

### `ImpedanceControlSim.m`
Simulation of impedance control system with the following features:
- Mass-spring-damper system dynamics
- Real-time force and position visualization
- Animated GIF generation showing system behavior
- Configurable impedance parameters (Md, Bd, Kd)

### `AdmittanceControlSim.m`
Simulation of admittance control system with the following features:
- External force interaction simulation
- Trajectory tracking with spring dynamics
- Real-time visualization of system states
- Animated GIF generation showing interaction

## System Parameters

### Impedance Control Parameters
- **Md**: Desired mass (kg)
- **Bd**: Desired damping (N·s/m)
- **Kd**: Desired stiffness (N/m)

### Admittance Control Parameters
- **Md**: Desired mass (kg)
- **Bd**: Desired damping (N·s/m)
- **Kd**: Desired stiffness (N/m)

## Usage

1. Open MATLAB
2. Run either simulation file
3. View the real-time plots
4. Press ENTER when prompted to generate animated GIF
5. Check the generated GIF files in the current directory

## Requirements

- MATLAB R2016b or later
- Image Processing Toolbox (for GIF generation)

## Output

Each simulation generates:
- Real-time plots of position, velocity, acceleration, and force
- Animated GIF showing system dynamics
- Console output showing simulation progress

## Theory

### Impedance Control
Impedance control regulates the relationship between force and position by controlling the mechanical impedance of the system. The control law is:

```
τ_f = m·ẍ_c - Md(ẍ_c - ẍ_d) - Bd(ẋ_c - ẋ_d) - Kd(x_c - x_d)
```

### Admittance Control
Admittance control regulates the relationship between position and force by controlling the mechanical admittance of the system. The control law is:

```
ẍ_f = (τ_c - τ_d - Bd·ẋ_f - Kd·x_f) / Md
```

## License

This project is open source and available under the MIT License.

## Contributing

Feel free to submit issues and enhancement requests!
