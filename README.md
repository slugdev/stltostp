# stltostp
Command line utility to convert STL files to STEP (ISO 10303-21) files. The translation is a direct triangle-to-triangle conversion with tolerance-based edge merging. stltostp generates CAD-interoperable STEP files without depending on third-party libraries like OpenCASCADE or FreeCAD.

**Features:**
- STL to STEP conversion (ASCII and binary STL support)
- Selectable output units (mm, cm, m, in) and STEP schema (AP203, AP214)
- Automatic edge merging with configurable tolerance
- No external CAD library dependencies

![Image of stltostp input_output](https://github.com/slugdev/stltostp/blob/master/doc/input_output.jpg)

### Usage
stltostp <stl_file> <step_file> \[ tol \<value\> \] \[ units \<mm|cm|m|in\> \] \[ schema \<203|214\> \] \[ mergeplanar \]

- `tol` - minimum edge length; shorter edges are merged (default 1e-6)
- `units` - length unit written to the STEP file: `mm` (default), `cm`, `m`, or `in`
- `schema` - STEP schema declared in the file header: `203` (default) or `214`
- `mergeplanar` - merge edge-connected coplanar triangles into single planar faces

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
[stltostp-1.0.3-win64.exe](https://github.com/slugdev/stltostp/releases/download/v1.0.3/stltostp-1.0.3-win64.exe)
