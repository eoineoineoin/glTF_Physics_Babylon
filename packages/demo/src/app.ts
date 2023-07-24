import "@babylonjs/core/Debug/debugLayer";
import { Vector3 } from "@babylonjs/core";
import { Engine, Scene, SceneLoader, FreeCamera, TransformNode } from "@babylonjs/core";
import { ShadowGenerator, IShadowLight } from "@babylonjs/core";
import { Observable } from "@babylonjs/core";
import { FilesInput } from "@babylonjs/core";

import HavokPhysics from "@babylonjs/havok";
import { PhysicsBody, PhysicsMotionType, HavokPlugin } from "@babylonjs/core";

import { KeyboardEventTypes } from "@babylonjs/core";

import "@babylonjs/loaders/glTF";
import { GLTF2 } from "@babylonjs/loaders";
import { MSFT_RigidBodies_Plugin } from "@babylongltfphysics/loader";

const g_havokInterface = await HavokPhysics();
MSFT_RigidBodies_Plugin.s_havokInterface = g_havokInterface;
GLTF2.GLTFLoader.RegisterExtension(
   "MSFT_rigid_bodies", function (loader) { return new MSFT_RigidBodies_Plugin(loader); } );

import { PhysicsMouseSpring } from './physicsUtils';
import { CubeTexture } from "@babylonjs/core";

// Allows pausing the simulation while we're loading in assets. Avoids visible hitches
// in the framerate.
class HavokPluginWithPausing extends HavokPlugin {
    public isPaused: boolean = false;
    public totalIntegratedTime: number = 0;
    public prePhysicsStepObservable = new Observable();

    public constructor(_useDeltaForWorldStep: boolean = true, hpInjection: any) {
        super(_useDeltaForWorldStep, hpInjection);
    }

    public executeStep(delta: number, physicsBodies: Array<PhysicsBody>): void {
        if (!this.isPaused) {
            this.totalIntegratedTime += delta;
            this.prePhysicsStepObservable.notifyObservers(delta);
            super.executeStep(delta, physicsBodies);
        }
    }
}

interface SceneInfo {
    asset: string,
    title: string,
    description: string,
    link: string
}

//<todo.eoin Ideally this list should come from the samples repo itself!
var sceneInfos: SceneInfo[] = [
    { 
        asset: "https://raw.githubusercontent.com/eoineoineoin/glTF_Physics/master/samples/ShapeTypes/ShapeTypes.glb",
        link: "https://github.com/eoineoineoin/glTF_Physics/tree/master/samples/ShapeTypes",
        title: "Basic shapes", description: "Shows the basic collision shapes",
    },
    { 
        asset: "https://raw.githubusercontent.com/eoineoineoin/glTF_Physics/master/samples/Filtering/Filtering.glb",
        link: "https://github.com/eoineoineoin/glTF_Physics/tree/master/samples/Filtering",
        title: "Filtering", description: "Shows the effect of collision filters",
    },
    {
        asset: "https://raw.githubusercontent.com/eoineoineoin/glTF_Physics/master/samples/JointTypes/JointTypes.glb",
        link: "https://github.com/eoineoineoin/glTF_Physics/tree/master/samples/JointTypes",
        title: "Joints", description: "Various types of joints",
    },
    {
        asset: "https://raw.githubusercontent.com/eoineoineoin/glTF_Physics/master/samples/Materials_Friction/Materials_Friction.glb",
        link: "https://github.com/eoineoineoin/glTF_Physics/tree/master/samples/Materials_Friction",
        title: "Materials - Friction", description: "Shows effect of friction",
    },
    {
        asset: "https://raw.githubusercontent.com/eoineoineoin/glTF_Physics/master/samples/Materials_Restitution/Materials_Restitution.glb", 
        link: "https://github.com/eoineoineoin/glTF_Physics/tree/master/samples/Materials_Restitution",
        title: "Materials - Restitution", description: "Shows effect of restitution",
    },
    {
        asset: "https://raw.githubusercontent.com/eoineoineoin/glTF_Physics/master/samples/MotionProperties/MotionProperties.glb",
        link: "https://github.com/eoineoineoin/glTF_Physics/tree/master/samples/MotionProperties",
        title: "Motion properties", description: "How motion properties can control behaviour",
    },
];

class App {
    private _engine: Engine;
    private _currentScene: Scene;
    private _hkPlugin: HavokPluginWithPausing;

    constructor() {
        var canvas = <HTMLCanvasElement>document.getElementById("sceneCanvas");
        this._engine = new Engine(canvas, true);
        this.setupSceneSelection();
    }

    private async loadSceneUrl(url: string) {
        if (this._currentScene) {
            this._currentScene.dispose();
        }

        let scene = this.setupEmptyScene();
        await this.setupPhysics(scene);
        await SceneLoader.ImportMeshAsync("", url, "", scene);

        if (scene.cameras.length > 1) {
            // If the scene we loaded has added a camera, let's use it
            scene.activeCamera = scene.cameras[1];
            var canvas = <HTMLCanvasElement>document.getElementById("sceneCanvas");
            this.addCameraControls(canvas, <FreeCamera>scene.cameras[1]);
        }

        this.setupShadows(scene);
    }

    private setupSceneSelection() {
        let ul = document.createElement("ul");
        for (let si of sceneInfos) {
            let li = document.createElement("li");
            let title = document.createElement("span");
            title.innerText = si.title;
            title.setAttribute("class", "sceneSelectTitle");
            let loadScene = () => {
                this.unselectActiveScene();
                li.setAttribute("class", "selectedScene");
                this.loadSceneUrl(si.asset);
            };
            title.onclick = loadScene;
            if (si === sceneInfos[0]) {
                loadScene();
            }
            var desc = document.createElement("span");
            desc.innerText = si.description;
            desc.setAttribute("class", "sceneSelectDescription");

            var link = document.createElement("a");
            link.innerText = "🔗";
            link.setAttribute("href", si.link);
            link.setAttribute("class", "sceneSelectLink");

            li.appendChild(title);
            li.appendChild(link);
            li.appendChild(desc);
            ul.appendChild(li);
        }

        document.getElementById("sceneSelect").appendChild(ul);
    }

    private setupEmptyScene() {
        var scene = new Scene(this._engine);
        window.addEventListener("resize", () => { this._engine.resize(); });
        this._engine.runRenderLoop(() => {
            scene.render();
        });

        var canvas = <HTMLCanvasElement>document.getElementById("sceneCanvas");
        this.setupCamera(canvas, scene);
        this.setupEnvironmentTex(scene);
        this._currentScene = scene;
        return scene;
    }

    private setupEnvironmentTex(scene: Scene) {
        const cubeTex = CubeTexture.CreateFromPrefilteredData("https://assets.babylonjs.com/environments/environmentSpecular.env", scene);
        scene.environmentTexture = cubeTex;
        scene.createDefaultSkybox(scene.environmentTexture, true, 1000, 0.2, false);
    }

    private setupCamera(canvas: HTMLCanvasElement, scene: Scene) {
        let camera = new FreeCamera("hkCamera", new Vector3(0.1, 1.8, 1.3), scene);
        camera.speed *= 0.1;
        camera.setTarget(new Vector3(-0.2, 0.8, -0.3));
        camera.minZ = 0.01;
        camera.maxZ = 100;
        this.addCameraControls(canvas, camera);
    }

    private addCameraControls(canvas: HTMLCanvasElement, camera: FreeCamera) {
        camera.attachControl(canvas, true);
        camera.keysUp.push(87);
        camera.keysDown.push(83);
        camera.keysLeft.push(65);
        camera.keysRight.push(68);
        camera.keysDownward.push(69);
        camera.keysUpward.push(81);
    }

    private setupShadows(scene: Scene) {
        // Make a guess about shadows based on the state of rigid bodies.
        // This isn't a great guess, but not sure if there's something more
        // clever we could be doing -- cameras lights and the layout of the
        // meshes could have any arbitary configuration..
        let isDynamicBody = (t: TransformNode): boolean => {
            while (t) {
                if (t.physicsBody) {
                    return !(t.physicsBody.getMotionType() == PhysicsMotionType.STATIC);
                }
                t = <TransformNode>t.parent;
            }
            return false;
        }

        let shadowGenerators: ShadowGenerator[] = [];
        for (let l of scene.lights) {
            const shadowGenerator = new ShadowGenerator(1024, <IShadowLight>l);
            //shadowGenerator.useExponentialShadowMap = true;
            shadowGenerator.usePoissonSampling = true;
            shadowGenerators.push(shadowGenerator);
            for (let m of scene.meshes) {
                if (isDynamicBody(m)) {
                    shadowGenerator.addShadowCaster(m, true);
                } else {
                    m.receiveShadows = true;
                }
            }
        }
    }


    private async setupPhysics(scene: Scene) {
        if (!scene.getPhysicsEngine()) {
            let gravityVector = new Vector3(0, -9.81 * 1, 0);
            this._hkPlugin = new HavokPluginWithPausing(true, g_havokInterface);
            scene.enablePhysics(gravityVector, this._hkPlugin);
        }

        let mouseSpringUtil = new PhysicsMouseSpring;
        scene.onKeyboardObservable.add((kbInfo) => {
            switch (kbInfo.type) {
                case KeyboardEventTypes.KEYDOWN:
                    if (kbInfo.event.key == " ") {
                    mouseSpringUtil.pickCamera(scene, scene.activeCamera);
                }
                break;
                case KeyboardEventTypes.KEYUP:
                    if (kbInfo.event.key == " ") {
                    mouseSpringUtil.release();
                }
                break;
            }
        });
        scene.getEngine().runRenderLoop(() => {
            mouseSpringUtil.stepCamera(scene, scene.activeCamera);
        });
    }

    private unselectActiveScene() {
        let activeEntries = document.getElementsByClassName("selectedScene");
        for (let i = 0; i < activeEntries.length; i++) {
            activeEntries[i].removeAttribute("class");
        }
    }
}

new App();
