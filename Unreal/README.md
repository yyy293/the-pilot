# SKYBOUND

SKYBOUND is an aircraft and flight-simulation project. The repository contains a browser-based Three.js prototype (`index.html`) and an Unreal Engine 5 C++ project scaffold.

## Unreal Engine

Open `Skybound.uproject` with Unreal Engine 5.8. The C++ module is in `Source/Skybound`.

### First aircraft setup

1. Open the project and allow Unreal to compile the C++ module.
2. Create a Blueprint class derived from `SkyboundAircraft`.
3. Assign the EF-CESENA mesh to the `AircraftMesh` component.
4. Ensure the mesh has collision and a sensible pivot near the aircraft's centre of mass.
5. Place the aircraft Blueprint in a test level and set it as the GameMode's Default Pawn Class.
6. Start in a large, flat test area and tune the exposed aircraft properties for the imported model.

### Keyboard controls

| Key | Action |
|---|---|
| W / S | Pitch nose up / down |
| A / D | Roll left / right |
| Q / E | Yaw left / right |
| R / F | Increase / decrease throttle |

## Flight model v1

- Physics-driven rigid-body movement.
- Engine thrust along the aircraft's forward axis.
- Atmospheric density decreasing with altitude.
- Angle-of-attack-based lift.
- Smooth lift loss and increased drag beyond the critical angle of attack.
- Parasitic and induced drag.
- Pitch, roll and yaw control moments with angular-rate damping.
- Frame-rate-independent throttle input.

This is a game-oriented baseline, not a validated real-world flight-dynamics model. It does not yet include propeller RPM and blade forces, detailed airfoil tables, trim, flaps, control-surface geometry, ground effect, wind, landing-gear forces, icing, compressibility, or aircraft-specific engine maps. Values must be tuned to the mesh, mass, centre of mass and inertia tensor.

The existing browser prototype remains in `index.html`.
