# stltostp
Command line utility to convert STL files to STEP (ISO 10303-21) files. The translation is a direct triangle-to-triangle conversion with tolerance-based edge merging. stltostp generates CAD-interoperable STEP files without depending on third-party libraries like OpenCASCADE or FreeCAD.

**Features:**
- ✅ STL to STEP conversion (ASCII and binary STL support)
- ✅ AP214 ISO 10303-214 standard compliance for CAD tool interoperability
- ✅ Automatic edge merging with configurable tolerance
- ✅ No external CAD library dependencies

![Image of stltostp input_output](https://github.com/slugdev/stltostp/blob/master/doc/input_output.jpg)

### Usage
stltostp <stl_file> <step_file> \[ tol \<value\> \]
![Image of stltostp usage](https://github.com/slugdev/stltostp/blob/master/doc/example.jpg)

### Build
```
mkdir build
cd build
cmake ..
make clean all && sudo make install
```
### License 
BSD

## Windows Installer
[stltostp-1.0.1-win64.msi](https://github.com/slugdev/stltostp/releases/download/v1.0.1/stltostp-1.0.1-win64.msi)
