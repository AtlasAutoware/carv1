# cad — carv1's physical build

Formerly the `atlasautoware-cad` repo, merged into carv1 with its full history.

- **Components/** — 3D models of the off-the-shelf parts on the car (Jetson Orin Nano, RPLIDAR C1, PCA9685 servo driver)
- **Mounts/** — the mounts and baseplate we designed for those parts. Every part is a Python (CadQuery) script with fit checks; `out/` and `gcode/` hold the exports. How they were made: [DESIGN.md](Mounts/DESIGN.md), conventions: [CONVENTIONS.md](Mounts/CONVENTIONS.md)
- **Templates/** — the original mounting system (baseplate, clip and clipless mounting pieces) the mounts are customized from
- **Electronics/** — AtlasPower-4S, the power board that runs the car's electronics off the 4S drive LiPo ([README](Electronics/README.md))
- **Jetson/** — JetPack rebuild and smoke-test scripts
