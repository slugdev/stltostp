# stltostp
Command line utility to convert STL files to STEP (ISO 10303-21) files. The translation is a direct triangle-to-triangle conversion with tolerance-based edge merging. stltostp generates CAD-interoperable STEP files without depending on third-party libraries like OpenCASCADE or FreeCAD.

**Features:**
- STL to STEP conversion (ASCII and binary STL support)
- Selectable output units (mm, cm, m, in) and STEP schema (AP203, AP214)
- Automatic edge merging with configurable tolerance
- No external CAD library dependencies

![Image of stltostp input_output](https://github.com/slugdev/stltostp/blob/master/doc/input_output.jpg)

### Usage
<<<<<<< HEAD
stltostp <stl_file> <step_file> [tol <value>] [units <mm|cm|m|in>] [schema <203|214>] [mergeplanar]

- `tol <value>` - minimum edge tolerance used for edge merging
- `units <mm|cm|m|in>` - output units (default: mm)
- `schema <203|214>` - STEP schema to write (default: 203)
- `mergeplanar` - merge edge-connected coplanar triangles into single planar faces, simplifying the output geometry
=======
stltostp <stl_file> <step_file> \[ tol \<value\> \] \[ units \<mm|cm|m|in\> \] \[ schema \<203|214\> \]

- `tol` - minimum edge length; shorter edges are merged (default 1e-6)
- `units` - length unit written to the STEP file: `mm` (default), `cm`, `m`, or `in`
- `schema` - STEP schema declared in the file header: `203` (default) or `214`
>>>>>>> origin/master

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
