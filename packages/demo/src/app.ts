import "@babylonjs/core/Debug/debugLayer";
import { Vector3 } from "@babylonjs/core";
import { Engine, Scene, SceneLoader, FreeCamera, TransformNode } from "@babylonjs/core";
import { ShadowGenerator, IShadowLight } from "@babylonjs/core";
import { Observable, PointerEventTypes } from "@babylonjs/core";
import { FilesInput } from "@babylonjs/core";

import HavokPhysics from "@babylonjs/havok";
import { PhysicsBody, PhysicsMotionType, HavokPlugin } from "@babylonjs/core";

import { KeyboardEventTypes } from "@babylonjs/core";

import "@babylonjs/loaders/glTF";
import { GLTF2 } from "@babylonjs/loaders";
import { KHR_RigidBodies_Plugin } from "babylon-gltf-rigid-body-loader";
import { MSFT_RigidBodies_Plugin } from "babylon-gltf-rigid-body-loader";

const g_havokInterface = await HavokPhysics();
KHR_RigidBodies_Plugin.s_havokInterface = g_havokInterface;
GLTF2.GLTFLoader.RegisterExtension(
   "KHR_rigid_bodies", function (loader) { return new KHR_RigidBodies_Plugin(loader); } );
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

class App {
    private _engine: Engine;
    private _hkPlugin: HavokPluginWithPausing;
    private _defaultCamera: FreeCamera;

    constructor() {
        var canvas = <HTMLCanvasElement>document.getElementById("sceneCanvas");
        this._engine = new Engine(canvas, true);
        this.setupEmptyScene();

        const assetListUrl = "https://raw.githubusercontent.com/eoineoineoin/glTF_Physics/master/samples/samplelist.json";
        fetch(assetListUrl).then((r: Response) => {
            r.json().then((j: SceneInfo[]) => {
                this.setupSceneSelection(j);
            });
        });

        const sceneDropped = (_sceneFile: File, scene: Scene) => {
            /*
            if (scene.cameras.length == 0) {
                this.setupCamera(scene);
            } else {
                scene.setActiveCameraByName(scene.cameras[0].name);
                if (scene.cameras[0] instanceof FreeCamera) {
                    this.addCameraControls(canvas, scene.cameras[0]);
                }
            }
            */

            scene.skipFrustumClipping = true;
            this.setupEnvironmentTex(scene);
            this.setupPhysics(scene);
            this.setupShadows(scene);
            this.createDefaultCamera(scene);
            this.setupCameraSelection(scene);
        }

        var filesInput = new FilesInput(this._engine, null, sceneDropped, null, null, null, null, null, null);
        filesInput.monitorElementForDragNDrop(canvas)
    }

    private async loadSceneUrl(url: string) {
        let scene = this.setupEmptyScene();
        await this.setupPhysics(scene);
        await SceneLoader.ImportMeshAsync("", url, "", scene);

        this.setupCameraSelection(scene);
        this.setupShadows(scene);
    }

    private setupSceneSelection(sceneInfos: SceneInfo[]) {
        let ul = document.createElement("ul");
        let selectedSceneIndex = 0;
        
        let params = this.locationHashParams();
        if ("sceneIndex" in params) {
            selectedSceneIndex = +params["sceneIndex"];
        }

        if ("sceneUrl" in params) {
            selectedSceneIndex = -1;
            this.loadSceneUrl(params["sceneUrl"]);
        }

        for (let i = 0; i < sceneInfos.length; i++) {
            let si = sceneInfos[i];
            let li = document.createElement("li");
            let title = document.createElement("span");
            title.innerText = si.title;
            title.setAttribute("class", "selectableItem");
            let loadScene = () => {
                let activeEntries = document.getElementsByClassName("selectedScene");
                for (let i = 0; i < activeEntries.length; i++) {
                    activeEntries[i].removeAttribute("class");
                }

                li.setAttribute("class", "selectedScene");
                this.loadSceneUrl(si.asset);
                window.location.hash = '#sceneIndex=' + i;
            };
            title.onclick = loadScene;
            if (i == selectedSceneIndex) {
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

    private setupCameraSelection(scene: Scene) {
        if (scene.cameras.length > 1) {
            // If the scene contained any cameras, let's default to that one
            scene.activeCamera = scene.cameras[0] == this._defaultCamera ? scene.cameras[1] : scene.cameras[0];
        } else {
            scene.activeCamera = scene.cameras[0];
        }

        let camList = document.createElement("ul");
        for (let cam of scene.cameras) {
            this.addCameraKeyboardControl(<FreeCamera>cam);
            if (cam != this._defaultCamera) {
                (<FreeCamera>cam).speed *= 0.1; // Default is too fast for my liking
                (<FreeCamera>cam).angularSensibility *= 2;
            }

            let li = document.createElement("li");
            let title = document.createElement("span");
            title.innerText = "🎥 " + cam.name;
            title.setAttribute("class", "selectableItem");
            let changeCamera = () => {
                scene.activeCamera.detachControl();
                let activeEntries = document.getElementsByClassName("selectedCamera");
                for (let i = 0; i < activeEntries.length; i++) {
                    activeEntries[i].removeAttribute("class");
                }
                scene.activeCamera = cam;
                li.setAttribute("class", "selectedCamera");
                scene.activeCamera.attachControl(undefined, true);
            };
            title.onclick = changeCamera;
            li.appendChild(title);
            camList.appendChild(li);
            if (cam == scene.activeCamera) {
                li.setAttribute("class", "selectedCamera");
            }
        }

        let cameraSelect = document.getElementById("cameraSelect");
        cameraSelect.replaceChildren(camList);
        scene.activeCamera.attachControl(undefined, true);
    }

    private setupEmptyScene() {
        var scene = new Scene(this._engine);
        window.addEventListener("resize", () => { this._engine.resize(); });
        this._engine.runRenderLoop(() => {
            scene.render();
        });

        this.createDefaultCamera(scene);
        this.setupEnvironmentTex(scene);
        scene.skipFrustumClipping = true;
        return scene;
    }

    private setupEnvironmentTex(scene: Scene) {
        const cubeTex = CubeTexture.CreateFromPrefilteredData("https://assets.babylonjs.com/environments/environmentSpecular.env", scene);
        scene.environmentTexture = cubeTex;
        scene.createDefaultSkybox(scene.environmentTexture, true, 1000, 0.2, false);
    }

    private createDefaultCamera(scene: Scene): FreeCamera {
        let camera = new FreeCamera("Default Camera", new Vector3(0.1, 1.8, 1.3), scene);
        camera.speed *= 0.1;
        camera.angularSensibility *= 2;
        camera.setTarget(new Vector3(-0.2, 0.8, -0.3));
        camera.minZ = 0.01;
        camera.maxZ = 100;
        this._defaultCamera = camera;
        return camera;
    }

    private addCameraKeyboardControl(camera: FreeCamera) {
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

        scene.onPointerObservable.add((pointerInfo) => {
            if (pointerInfo.type == PointerEventTypes.POINTERWHEEL) {
                //@ts-ignore
                if (pointerInfo.event.wheelDelta) {
                    //@ts-ignore
                    if (pointerInfo.event.wheelDelta > 0) {
                        mouseSpringUtil.reelOutOnce();
                    } else {
                        mouseSpringUtil.reelInOnce();
                    }
                }
            }
        });

        if (navigator.maxTouchPoints) {
            scene.onPointerObservable.add((pointerInfo) => {
                if(pointerInfo.type == PointerEventTypes.POINTERDOWN) {
                    mouseSpringUtil.pickCamera(scene, scene.activeCamera);
                } else if (pointerInfo.type == PointerEventTypes.POINTERUP) {
                    mouseSpringUtil.release();
                }
            });
        }
        scene.getEngine().runRenderLoop(() => {
            mouseSpringUtil.stepCamera(scene, scene.activeCamera);
        });
    }

    private locationHashParams(): {[key: string]: string} {
        let ret = {};
        for (let p of window.location.hash.slice(1).split("&")) {
            let kv = p.split("=");
            ret[kv[0]] = kv[1];
        }
        return ret;
    }
}

new App();
