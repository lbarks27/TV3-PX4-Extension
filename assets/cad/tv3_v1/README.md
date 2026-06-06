# TV3 v1 Gazebo CAD Visuals

Drop renderable Gazebo visual meshes for `tv3_v1` in this directory.

Gazebo does not render STEP files directly. Keep STEP files as source CAD references only, and export visual meshes as one of:

- `.glb`
- `.gltf`
- `.dae`
- `.obj`
- `.stl`

The `tv3_v1` manifest is configured as CAD-only for vehicle structure. If a mesh listed in `config/vehicles/tv3_v1.yaml` is missing, the generator omits that visual instead of drawing a procedural fallback.

Expected first-pass files:

- `tv3_v1_static_structure.glb`: fixed airframe, nose, fins, avionics, and fixed TVC housing. Exclude moving nozzle/TVC geometry if separate animation should be visible.
- `tv3_v1_engine_nozzle.glb`: moving nozzle/TVC visual named `engine_nozzle_0`.

Mesh frame expectations:

- Use the vehicle reference frame declared in the manifest: origin at nozzle exit center, +X forward along the airframe, +Y vehicle right, +Z vehicle down.
- Static structure mesh origin should match the vehicle reference origin and use an identity pose in the manifest.
- Nozzle mesh origin should be at the TVC pivot, with local +X along the thrust axis.

The procedural `thrust_cue_0` visual is not vehicle structure. It remains as a command-driven flame cue for future thrust visualization.
