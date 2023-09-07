## About

An importer for loading glTF files with rigid body information into Babylon.js.

For more information on how to specify rigid body information in your glTF files, see the [KHR_rigid_bodies](https://github.com/eoineoineoin/glTF_Physics) repository. This package supports two extensions for being able to specify rigid bodies - the `KHR_rigid_bodies` extension is under active development and, as such, may experience name changes which requires assets to be re-exported. If you want a more stable experience, the `MSFT_rigid_bodies` plugin has a frozen feature set, and won't be experiencing any breaking changes. Both plugins can be used simultaneously or independently.

If you're using Blender, you can add the [Blender glTF physics](https://github.com/eoineoineoin/glTF_Physics_Blender_Exporter) exporter addon to export physics information with your existing Blender files. Use the [hk_users branch](https://github.com/eoineoineoin/glTF_Physics_Blender_Exporter/tree/hk_users) if you wish to export using the `MSFT_rigid_bodies` extension.

## Usage

Install the package:

```
npm -i babylon-gltf-rigid-body-loader
```

In your project, register the extension with Babylon's glTF loader:

```
import { GLTF2 } from "@babylonjs/loaders";

// Latest, development plugin; might change in the future, which might break assets!
import { KHR_RigidBodies_Plugin } from "babylon-gltf-rigid-body-loader";

GLTF2.GLTFLoader.RegisterExtension(
   "KHR_rigid_bodies", function (loader) {
       return new KHR_RigidBodies_Plugin(loader);
   });


// Stable, feature-frozen plugin; won't make any breaking changes to assets:
import { MSFT_RigidBodies_Plugin } from "babylon-gltf-rigid-body-loader";

GLTF2.GLTFLoader.RegisterExtension(
   "MSFT_rigid_bodies", function (loader) {
       return new MSFT_RigidBodies_Plugin(loader);
   });
```

Ensure that you've called `scene.enablePhysics()` on your scene before loading any assets containing physics information.

Load your glTF assets as normal!

## Demo

You can try out a [live demo](https://eoineoineoin.github.io/glTF_Physics_Babylon/packages/demo/dist/) with samples served from [the specification repository](https://github.com/eoineoineoin/glTF_Physics/tree/master/samples/) - other files can also be dragged-and-dropped into the window to load them, for easy testing.

