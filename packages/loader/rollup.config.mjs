// rollup.config.js
import typescript from "@rollup/plugin-typescript";
import terser from '@rollup/plugin-terser';
// a little hack to support global variables AND cjs using @babylonjs/core
const importsArray = {
  "@babylonjs/core": [
    "Meshes/mesh",
    "Meshes/mesh.vertexData",
    "Meshes/meshBuilder",
    "Physics/joinedPhysicsEngineComponent",
    "Maths/math.vector",
    "Physics/v2/IPhysicsEnginePlugin",
    "Physics/v2/physicsShape",
    "Physics/v2/physicsMaterial",
    "Physics/v2/physicsBody",
    "Physics/v2/physicsEngineComponent",
    "Physics/v2/physicsConstrain",
    "Physics/v2/Plugins/havokPlugin",
    "Physics/v2/physicsConstraint"
  ],
};
const globals = {

};
importsArray["@babylonjs/core"].forEach((i) => {
  globals["@babylonjs/core/" + i] = "BABYLON";
});
const config = {
  input: "src/index.ts",
  external: importsArray["@babylonjs/core"].map((i) => "@babylonjs/core/" + i),
  output: {
    file: "dist/babylon-gltf-rigid-body-loader.js",
    format: "umd",
    name: "GLTFRigidBodyLoader",
    sourcemap: true,
    globals,
  },
  plugins: [typescript({ sourceMap: true, lib: ["es5", "es6", "dom"], target: "es6", inlineSourceMap: true }), terser()],
};

export default config;
