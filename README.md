## About

An importer for loading glTF files with rigid body information into Babylon.js.

For more information on how to specify rigid body information in your glTF files, see the [MSFT_Physics](https://github.com/eoineoineoin/glTF_Physics) repository. Alternatively, if you're using Blender, you can add the [Blender glTF physics](https://github.com/eoineoineoin/glTF_Physics_Blender_Exporter) exporter to export physics information with your existing Blender files.

## Usage

Add the `@babylongltfphysics/loader` package in `packages/loader` to your Babylon project.

Register the extension with Babylon's glTF loader:

```
import { GLTF2 } from "@babylonjs/loaders";

GLTF2.GLTFLoader.RegisterExtension(
   "MSFT_rigid_bodies", function (loader) {
       return new MSFT_RigidBodies_Plugin(loader);
   });
```

Ensure that you've called `scene.enablePhysics()` on your scene before loading any assets containing physics information.

Load your glTF assets as normal!

