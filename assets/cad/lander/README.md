# TV3 Lander Gazebo CAD Visuals

Drop renderable Gazebo visual meshes for `tv3_lander_v1` in this directory.

Gazebo does not render STEP files directly. Keep STEP files as source CAD references only, and export visual meshes as one of:

- `.glb`
- `.gltf`
- `.dae`
- `.obj`
- `.stl`

The lander manifest is configured as CAD-only for vehicle structure. If a mesh listed in `config/vehicles/tv3_lander_v1.yaml` is missing, the generator omits that visual instead of drawing a procedural fallback.

Expected first-pass files:

- `tv3_lander_static_structure.glb`: fixed vehicle structure. Exclude moving nozzles if separate nozzle animation should be visible.
- `tv3_lander_engine_nozzle.glb`: reusable nozzle mesh for `engine_nozzle_0`, `engine_nozzle_1`, and `engine_nozzle_2`.

Mesh frame expectations:

- Use the vehicle reference frame declared in the manifest: origin at nozzle exit center, +X forward along the airframe, +Y vehicle right, +Z vehicle down.
- Static structure mesh origin should match the vehicle reference origin and use an identity pose in the manifest.
- Nozzle mesh origin should be at the engine pivot, with local +X along the thrust axis. `Tv3RocketSystem` moves visuals named `engine_nozzle_<index>` from command-truth pitch, yaw, and splay values.

The procedural `thrust_cue_<index>` visuals are not vehicle structure. They remain as command-driven flame cues so thrust is visible during BOOST.
