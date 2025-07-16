import { GLTF2 } from "@babylonjs/loaders/glTF";
import { IGLTFLoaderExtension } from "@babylonjs/loaders/glTF/2.0";

export namespace KHR_implicit_shapes
{
    export class Sphere
    {
        radius : number = 0.5;

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class Box
    {
        size : [number, number, number] = [1, 1, 1];

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class Capsule
    {
        height: number = 0.5;
        radiusTop: number = 0.25;
        radiusBottom: number = 0.25;

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class Cylinder
    {
        height: number = 0.5;
        radiusTop: number = 0.25;
        radiusBottom: number = 0.25;

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class Shape
    {
        type? : string;
        sphere? : Sphere;
        box? : Box;
        capsule? : Capsule;
        cylinder? : Cylinder;

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class SceneExt
    {
        shapes: Array<Shape> = [];
    }
}

export class KHR_ImplicitShapes_Plugin implements IGLTFLoaderExtension  {
    public name : string = "KHR_implicit_shapes";
    public enabled : boolean = true;

    public constructor(loader : GLTF2.GLTFLoader)
    {
    }

    public dispose() : void
    {
    }
}
