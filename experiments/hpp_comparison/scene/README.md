# HPP scene files

Hand-written models for HPP. Each URDF goes to `urdf.loadModelFromString` together with its SRDF, a prefix
and a pose.

| Model | URDF | SRDF | Prefix | Parent frame | Pose |
|---|---|---|---|---|---|
| Arm | `my_sdfs/panda_arm.urdf` | `panda_arm.srdf` | `panda` | world | identity |
| Hand | `my_sdfs/panda_hand.urdf` | `panda_hand.srdf` | `hand` | `panda/panda_link8` | RotZ(-45 deg) |
| Each cap | `bottle_cap.urdf` | `bottle_cap.srdf` | `cap<k>` | world | `SCENES[scene]["caps"][k]` |
| Each obstacle | `obstacle.urdf` | `empty.srdf` | `obstacle<k>` | world | `SCENES[scene]["obstacles"][k]` |
| Floor | `floor.urdf` | `empty.srdf` | `floor` | world | identity |
| Walls, 8caps_3obs only | `walls.urdf` | `empty.srdf` | `walls` | world | identity |

`SCENES` is in `multicap_scene.py`. The numbers in `bottle_cap.urdf`, `obstacle.urdf`, `floor.urdf` and
`walls.urdf` are copies of values in `my_sdfs/` and `multicap_scene.py`, so a change there needs the same
change here.

## Meshes

The arm and hand URDFs use `package://drake_models/...` visual meshes, and loading fails if HPP can't find
them. Copy them once, after building HPP. `env/install/` is on the container's search path and ignored by git.

```bash
DM=$(conda run -n obmp python -c "from pydrake.multibody.parsing import PackageMap; print(PackageMap().GetPath('drake_models'))" | tail -1)
T=experiments/hpp_comparison/env/install/share/drake_models/franka_description/meshes
mkdir -p "$T" && cp -r "$DM/franka_description/meshes/visual" "$T/"
```

### Meshes update
Now, we are taking away drake meshes and replacing them with meshes from example-robot-data. This is because drake meshes were rotated in a way that did not match the visualization in HPP. The new meshes are also in formar .dae, instead of .gltf.
