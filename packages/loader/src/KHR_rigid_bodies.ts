import { TransformNode } from "@babylonjs/core/Meshes/transformNode";
import { IGLTFLoaderExtension } from "@babylonjs/loaders/glTF";
import { GLTF2 } from "@babylonjs/loaders/glTF";
import { AbstractMesh, Mesh, VertexData, MeshBuilder, PhysicsConstraintMotorType } from "@babylonjs/core";
import { Nullable } from "@babylonjs/core";
import { Scene, Matrix, Quaternion, Vector3 } from "@babylonjs/core";

import { PhysicsShape,
    PhysicsShapeBox,
    PhysicsShapeCapsule,
    PhysicsShapeContainer,
    PhysicsShapeConvexHull,
    PhysicsShapeCylinder,
    PhysicsShapeSphere } from "@babylonjs/core";
import { PhysicsShapeType,
    PhysicsConstraintAxis,
    PhysicsMassProperties,
    PhysicsMotionType } from "@babylonjs/core";
import { PhysicsMaterialCombineMode } from "@babylonjs/core";
import { PhysicsBody } from "@babylonjs/core";
import { Physics6DoFConstraint,
    Physics6DoFLimit } from "@babylonjs/core";

import HavokPhysics from "@babylonjs/havok";
import { HavokPlugin } from "@babylonjs/core";

namespace KHR_collision_shapes
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

    export class Convex
    {
        mesh: number = -1;

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class TriMesh
    {
        mesh: number = -1;

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
        convex? : Convex;
        trimesh? : TriMesh;
    }

    export class SceneExt
    {
        shapes: Array<Shape> = [];
    }
}

namespace KHR_rigid_bodies
{
    export class RigidMotion
    {
        isKinematic : boolean = false;
        mass : number = 1;
        centerOfMass : [number, number, number] = [0, 0, 0];
        inertiaOrientation: [number, number, number, number] = [0, 0, 0, 1];
        inertiaDiagonal: [number, number, number] = [1, 1, 1];
        linearVelocity : [number, number, number] = [0, 0, 0];
        angularVelocity : [number, number, number] = [0, 0, 0];
        gravityFactor : number = 1;

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class CollisionFilter
    {
        collisionSystems? : Array<string>;
        collideWithSystems? : Array<string>;
        notCollideWithSystems? : Array<string>;
    }

    export class ConstraintDrive
    {
        positionTarget: number = 0;
        velocityTarget: number = 0;
        maxForce?: number;
        damping: number = 0;
        stiffness: number = 0;
    }

    export class Constraint
    {
        min? : number;
        max? : number;
        springConstant? : number;
        springDamping? : number;

        linearAxes? : Array<number>;
        angularAxes? : Array<number>;

        drive? : ConstraintDrive

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class JointLimitSet
    {
        limits: Array<Constraint> = [];
    }

    export class Joint
    {
        connectedNode : number = -1;
        jointLimits : number = -1;
        enableCollision : boolean = false;
    }

    export class Collider
    {
        shape? : number;
        physicsMaterial? : number;
        collisionFilter? : number;
    }

    export class Trigger
    {
        shape? : number;
        collisionFilter? : number;
    }

    export class NodeExt
    {
        motion? : RigidMotion;
        collider? : Collider;
        trigger? : Trigger;
        joint? : Joint;

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class PhysicsMaterial
    {
        staticFriction? : number;
        dynamicFriction? : number;
        restitution? : number;
        restitutionCombine? : string;
        frictionCombine? : string;
    }

    export class SceneExt
    {
        physicsMaterials? : Array<PhysicsMaterial>;
        physicsJointLimits? : Array<JointLimitSet>;
        collisionFilters? : Array<CollisionFilter>;
    }
}

class DeferredJoint
{
    jointInfo? : KHR_rigid_bodies.Joint;
    pivotA? : TransformNode;
    pivotB? : TransformNode;
}

export class KHR_RigidBodies_Plugin implements IGLTFLoaderExtension  {
    public static s_havokInterface: any;
    public name : string = "KHR_rigid_bodies";
    public enabled : boolean = true;
    private loader : GLTF2.GLTFLoader;
    private _deferredJoints : Array<DeferredJoint> = []
    private _deferredBodies : Array<() => void> = [];
    private _deferredColliders : Array<() => Promise<void>> = [];
    private _physicsVersion : number;
    private _layerNames: string[] = [];
    private _userMassProperties: Map<PhysicsBody, PhysicsMassProperties> = new Map<PhysicsBody, PhysicsMassProperties>();
    private _babylonScene: Scene;

    public constructor(loader : GLTF2.GLTFLoader)
    {
        this.loader = loader;

        this._babylonScene = loader.babylonScene;
        let physicsEngine = loader.babylonScene.getPhysicsEngine();
        this._physicsVersion = physicsEngine ? physicsEngine.getPluginVersion() : -1;
    }

    public dispose() : void
    {
    }

    protected async _constructPhysicsShape(
        context: string, sceneNode: AbstractMesh, gltfNode: GLTF2.INode,
        shapeData: KHR_collision_shapes.Shape,
        filterData : Nullable<KHR_rigid_bodies.CollisionFilter>,
        materialData:  Nullable<KHR_rigid_bodies.PhysicsMaterial>,
        assign: ((babylonMesh: TransformNode) => void)) : Promise<Nullable<PhysicsShape>> {

        let scene = this.loader.babylonScene;
        let physicsShape: Nullable<PhysicsShape> = null;
        if (shapeData.sphere != undefined) {
            var sphere = new PhysicsShapeSphere(Vector3.Zero(), shapeData.sphere.radius, scene);
            physicsShape = sphere;
        }
        else if (shapeData.box != undefined) {
            const size = Vector3.FromArray(shapeData.box.size);
            var box = new PhysicsShapeBox(Vector3.Zero(), Quaternion.Identity(), size, scene);
            physicsShape = box;
        }
        else if (shapeData.cylinder != undefined) {
            const pointA = new Vector3(0, 0.5 * shapeData.cylinder.height, 0);
            const pointB = new Vector3(0, 0.5 * -shapeData.cylinder.height, 0);

            if (shapeData.cylinder.radiusTop == shapeData.cylinder.radiusBottom) {
                var cylinder = new PhysicsShapeCylinder(pointA, pointB, shapeData.cylinder.radiusTop, scene);
                physicsShape = cylinder;
            } else {
                //<todo We're approximating this with a convex hull - should get the physics engine
                // to expose an explicit version of this, so it can take advantage of radial symmetry
                var cylinderMesh = new Mesh("custom", scene);

                var positions : Array<number> = [];
                const numDivisions = 30;
                for(let i = 0; i < numDivisions; i++) {
                    const c = Math.cos(2 * Math.PI * i / (numDivisions - 1));
                    const s = Math.sin(2 * Math.PI * i / (numDivisions - 1));
                    positions.push(c * shapeData.cylinder.radiusTop);
                    positions.push(0.5 * shapeData.cylinder.height);
                    positions.push(s * shapeData.cylinder.radiusTop);

                    positions.push(c * shapeData.cylinder.radiusBottom);
                    positions.push(-0.5 * shapeData.cylinder.height);
                    positions.push(s * shapeData.cylinder.radiusBottom);
                }
                var vertexData = new VertexData();
                vertexData.positions = positions;
                vertexData.applyToMesh(cylinderMesh);
                physicsShape = new PhysicsShapeConvexHull(cylinderMesh, scene);
            }
        }
        else if (shapeData.capsule != undefined) {
            const capsuleData = shapeData.capsule;
            const pointA = new Vector3(0, 0.5 * capsuleData.height, 0);
            const pointB = new Vector3(0, -0.5 * capsuleData.height, 0);

            if (capsuleData.radiusTop == capsuleData.radiusBottom) {
                var capsule = new PhysicsShapeCapsule(pointA, pointB, capsuleData.radiusTop, scene);
                physicsShape = capsule;
            } else {
                var templateMesh = MeshBuilder.CreateCapsule("tempCapsule", {
                    capSubdivisions: 64,
                    radiusTop: capsuleData.radiusTop,
                    radiusBottom: capsuleData.radiusBottom,
                    height: capsuleData.height + capsuleData.radiusTop + capsuleData.radiusBottom}, scene);
                physicsShape = new PhysicsShapeConvexHull(templateMesh, scene);
                templateMesh.dispose();
            }
        }
        else if (shapeData.convex != undefined) {
            var meshData = this.loader.gltf.meshes![shapeData.convex.mesh];
            //<todo I just want to access the mesh object here; not create one in the scene
            //@ts-ignore _loadMeshAsync is private:
            var convexMesh = await this.loader._loadMeshAsync(context.concat("/collider"), gltfNode, meshData, assign) as Mesh;
            convexMesh.parent = null;
            convexMesh.position = Vector3.Zero();
            convexMesh.rotationQuaternion = Quaternion.Identity();
            convexMesh.scaling = Vector3.One();

            physicsShape = new PhysicsShape({ type: PhysicsShapeType.CONVEX_HULL, parameters: { mesh: convexMesh, includeChildMeshes: true }}, scene);
            convexMesh.dispose();
        }
        else if (shapeData.trimesh != undefined) {
            var meshData = this.loader.gltf.meshes![shapeData.trimesh.mesh];
            //@ts-ignore _loadMeshAsync is private:
            var meshShape = await this.loader._loadMeshAsync(context.concat("/collider"), gltfNode, meshData, assign) as Mesh;
            meshShape.parent = null;
            meshShape.position = Vector3.Zero();
            meshShape.rotationQuaternion = Quaternion.Identity();
            meshShape.scaling = Vector3.One();

            physicsShape = new PhysicsShape({ type: PhysicsShapeType.MESH, parameters: { mesh: meshShape, includeChildMeshes: true }}, scene);
            meshShape.dispose();
        }

        if (physicsShape == undefined) {
            return null;
        }

        // Add collision filter info
        if (filterData) {
            let filterMembership = 0;
            let filterCollideWith = 0;
            let hasFilterInfo = false;
            if (filterData.collisionSystems) {
                filterMembership = this._layerNamesToMask(filterData.collisionSystems);
                hasFilterInfo = true;
            }
            if (filterData.collideWithSystems) {
                filterCollideWith = this._layerNamesToMask(filterData.collideWithSystems);
                hasFilterInfo = true;
            } else if (filterData.notCollideWithSystems) {
                filterCollideWith = ~this._layerNamesToMask(filterData.notCollideWithSystems);
                hasFilterInfo = true;
            }

            if (physicsShape && hasFilterInfo) {
                physicsShape.filterMembershipMask = filterMembership;
                physicsShape.filterCollideMask = filterCollideWith;
            }
        }

        if (materialData) {
            physicsShape.material = {
                friction: materialData.dynamicFriction?? 0.5,
                staticFriction: materialData.staticFriction?? 0.5,
                restitution: materialData.restitution?? 0.0,
                frictionCombine: this._materialCombineModeToNative(materialData.frictionCombine)?? PhysicsMaterialCombineMode.MAXIMUM,
                restitutionCombine: this._materialCombineModeToNative(materialData.restitutionCombine) ?? PhysicsMaterialCombineMode.MINIMUM
            };
        }

        return physicsShape;
    }

    private _materialCombineModeToNative(combine: string | undefined): PhysicsMaterialCombineMode | undefined {
        if (!combine) {
            return undefined;
        } else if (combine == "AVERAGE") {
            return PhysicsMaterialCombineMode.ARITHMETIC_MEAN;
        } else if (combine == "MINIMUM") {
            return PhysicsMaterialCombineMode.MINIMUM;
        } else if (combine == "MAXIMUM") {
            return PhysicsMaterialCombineMode.MAXIMUM;
        } else if (combine == "MULTIPLY") {
            return PhysicsMaterialCombineMode.MULTIPLY;
        }
        return undefined;
    }

    private _layerNamesToMask(names: Array<string>): number {
        let mask = 0;
        for (const name of names) {
            const idx = this._layerNames.indexOf(name);
            if (idx == -1) {
                mask |= 1 << this._layerNames.length;
                this._layerNames.push(name);
            } else {
                mask |= 1 << idx;
            }
        }
        return mask;
    }

    //<todo Split this up properly
    protected async _constructNodeObjects(
        context : string, sceneNode : AbstractMesh, gltfNode : GLTF2.INode,
        assign : ((babylonMesh: TransformNode) => void)) {
        var extData = gltfNode.extensions!.KHR_rigid_bodies as KHR_rigid_bodies.NodeExt;

        if (extData.collider != null) //<todo Also handle triggers, once exposed
        {
            let ext : KHR_collision_shapes.SceneExt = this.loader.gltf.extensions!.KHR_collision_shapes;
            var rbExt : KHR_rigid_bodies.SceneExt = this.loader.gltf.extensions!.KHR_rigid_bodies;
            let collider : KHR_collision_shapes.Shape = ext.shapes[extData.collider.shape!];
            let filter : Nullable<KHR_rigid_bodies.CollisionFilter> = null;
            let material : Nullable<KHR_rigid_bodies.PhysicsMaterial> = null;
            if (extData.collider.collisionFilter != null) {
                filter = rbExt.collisionFilters![extData.collider.collisionFilter];
            }
            if (extData.collider.physicsMaterial != null) {
                material = rbExt.physicsMaterials![extData.collider.physicsMaterial];
            }
            this._deferredColliders.push(async () => {
                let physicsShape = await this._constructPhysicsShape(context, sceneNode, gltfNode, collider, filter, material, assign);

                if (physicsShape == null) {
                    return;
                }

                let rigidBodyNode = this._getParentRigidBody(sceneNode);
                if (rigidBodyNode == null) {
                    // This is a static collider. Just make a new body
                    //<todo Maybe could just put a body on the root?
                    sceneNode.physicsBody = new PhysicsBody(sceneNode, PhysicsMotionType.STATIC, false, sceneNode.getScene());
                    rigidBodyNode = sceneNode;
                }

                // Now, add the shape to the body:
                if (!rigidBodyNode.physicsBody!.shape) {
                    rigidBodyNode.physicsBody!.shape = new PhysicsShapeContainer(sceneNode.getScene());
                    //<todo Optimize the case of a single collider with identity transform
                }

                let containerShape = <PhysicsShapeContainer>rigidBodyNode.physicsBody!.shape;

                const childToWorld = sceneNode.computeWorldMatrix(true);
                const parentToWorld = rigidBodyNode.computeWorldMatrix(true);
                const absScale = Matrix.Scaling(
                    rigidBodyNode.absoluteScaling.x,
                    rigidBodyNode.absoluteScaling.y,
                    rigidBodyNode.absoluteScaling.z);
                let childToParent = childToWorld.multiply(Matrix.Invert(parentToWorld)).multiply(absScale);
                const translation = new Vector3();
                const rotation = new Quaternion();
                const scale = new Vector3();
                childToParent.decompose(scale, rotation, translation);
                containerShape.addChild(physicsShape, translation, rotation, scale);

                //<todo Only do this once per body
                const massProps = this._userMassProperties.get(rigidBodyNode.physicsBody!);
                if (massProps) {
                    rigidBodyNode.physicsBody!.setMassProperties(massProps);
                }
                else {
                    rigidBodyNode.physicsBody!.setMassProperties(rigidBodyNode.physicsBody!.computeMassProperties());
                }
            });
        }

        if (extData.motion != null) {
            this._deferredBodies.push(() => {

                if (!extData.motion)
                    return;

                const motionType = extData.motion.isKinematic ? PhysicsMotionType.ANIMATED : PhysicsMotionType.DYNAMIC;
                sceneNode.physicsBody = new PhysicsBody(sceneNode, motionType, false, this.loader.babylonScene);

                if (extData.motion.linearVelocity != null) {
                    sceneNode.physicsBody.setLinearVelocity(Vector3.FromArray(extData.motion.linearVelocity));
                }

                if (extData.motion.angularVelocity != null) {
                    sceneNode.physicsBody.setAngularVelocity(Vector3.FromArray(extData.motion.angularVelocity));
                }

                var massProps : PhysicsMassProperties = {};

                if (extData.motion!.centerOfMass != null) {
                    massProps.centerOfMass = Vector3.FromArray(extData.motion!.centerOfMass);
                    massProps.centerOfMass.multiplyInPlace(sceneNode.absoluteScaling);
                }

                if (extData.motion.inertiaOrientation != null) {
                    massProps.inertiaOrientation = Quaternion.FromArray(extData.motion.inertiaOrientation);
                }

                if (extData.motion.inertiaDiagonal != null) {
                    const it = extData.motion.inertiaDiagonal;
                    massProps.inertia = new Vector3(it[0], it[1], it[2]);
                }

                if (extData.motion.mass != null) {
                    massProps.mass = extData.motion.mass;
                }

                sceneNode.physicsBody.setMassProperties(massProps);

                // Save the user-provided mass properties, so we can recalculate any
                // missing ones later, based on the shape
                this._userMassProperties.set(sceneNode.physicsBody, massProps);

                if (extData.motion.gravityFactor != null) {
                    sceneNode.physicsBody.setGravityFactor(extData.motion.gravityFactor);
                }
            });
        }

        if (extData.joint != null)
        {
            // Can load some joint info, but this can cause circular awaits...
            var jointInfo = {jointInfo: extData.joint, pivotA: sceneNode,
                pivotB: this.loader.gltf.nodes![extData.joint.connectedNode]._babylonTransformNode!};
            this._deferredJoints.push(jointInfo);
        }
     }


    public async loadNodeAsync(context : string, node : GLTF2.INode, assign : ((babylonMesh: TransformNode) => void))
    {
        if (node.extensions != undefined &&
            node.extensions.KHR_rigid_bodies != undefined &&
            this._physicsVersion == 2)
        {
            //<todo Can this really ever return a transform node? Will need to handle
            var loaded : AbstractMesh = <AbstractMesh>await this.loader.loadNodeAsync(context, node, assign);
            await this._constructNodeObjects(context, loaded, node, assign);

            return loaded;
        }

        return this.loader.loadNodeAsync(context, node, assign);
    }

    public async loadSceneAsync(context : string, scene : GLTF2.IScene) : Promise<void> {
        if (!this._babylonScene.getPhysicsEngine()) {
            var gravityVector = new Vector3(0, -9.81 * 1, 0);
            const hkPlugin = new HavokPlugin(true, KHR_RigidBodies_Plugin.s_havokInterface);
            this._babylonScene.enablePhysics(gravityVector, hkPlugin);
            let physicsEngine = this._babylonScene.getPhysicsEngine();
            this._physicsVersion = physicsEngine!.getPluginVersion();
        }

        return this.loader.loadSceneAsync(context, scene).then( async () => {
            await this.finishLoading();
        });
    }

    public async finishLoading() {
        if (this._physicsVersion != 2) {
            return;
        }

        for (let f of this._deferredBodies)
            f();

        for (let f of this._deferredColliders)
            await f();

        for(let joint of this._deferredJoints) {
            this.make6DoFJoint(joint);
        }
    }

    protected make6DoFJoint(joint : DeferredJoint) {
        // Get transform from parent RB?
        var rbA = this._getParentRigidBody(joint.pivotA!)!;
        var rbB = this._getParentRigidBody(joint.pivotB ?? null);

        var rbAToWorld = rbA.computeWorldMatrix(true);
        var rbAScale = Vector3.One();
        rbAToWorld.decompose(rbAScale);

        var pivotAToWorld = joint.pivotA!.computeWorldMatrix(true);
        var pivotAToBodyA = pivotAToWorld.multiply(Matrix.Invert(rbAToWorld));
        var pivotOrientationInA = Quaternion.Identity();
        pivotAToBodyA.decompose(undefined, pivotOrientationInA);
        var pivotTranslationInA = Vector3.TransformCoordinates(Vector3.Zero(), pivotAToBodyA).multiply(rbAScale);


        var rbBToWorld = (rbB ?? rbA).computeWorldMatrix(true);
        var rbBScale = Vector3.One();
        rbBToWorld.decompose(rbBScale);

        var pivotBToWorld = (joint.pivotB ?? joint.pivotA!).computeWorldMatrix(true);
        var pivotBToBodyB = pivotBToWorld.multiply(Matrix.Invert(rbBToWorld));
        var pivotOrientationInB = Quaternion.Identity();
        pivotBToBodyB.decompose(undefined, pivotOrientationInB);
        var pivotTranslationInB = Vector3.TransformCoordinates(Vector3.Zero(), pivotBToBodyB).multiply(rbBScale);

        var sceneExt : KHR_rigid_bodies.SceneExt = this.loader.gltf.extensions!.KHR_rigid_bodies;

        var limitSet = sceneExt.physicsJointLimits![joint.jointInfo!.jointLimits].limits;
        const nativeLimits: Physics6DoFLimit[] = []
        let motors: {axis: PhysicsConstraintAxis, drive: KHR_rigid_bodies.ConstraintDrive}[] = [];
        for (const l of limitSet) {
            if (l.linearAxes) {
                if (l.linearAxes.length == 3) {
                    nativeLimits.push({axis: PhysicsConstraintAxis.LINEAR_DISTANCE, minLimit: l.min, maxLimit: l.max});
                }
                //<todo Interface doesn't expose explicit 2D limits - they're inferred automatically.
                //< Should expose these and be more explicit about creating them
                else {
                    for (const axIdx of l.linearAxes) {
                        let axisNative = this._linearIdxToNative(axIdx);
                        nativeLimits.push({
                            axis: axisNative,
                            minLimit: l.min,
                            maxLimit: l.max,
                            stiffness: l.springConstant,
                            damping: l.springDamping
                        });

                        if (l.drive) {
                            motors.push({ axis: axisNative, drive: l.drive });
                        }
                    }
                }
            } else if (l.angularAxes) {
                //<todo Interface doesn't expose explicit 2D limits - they're inferred automatically.
                //< Should expose these and be more explicit about creating them
                for (const axIdx of l.angularAxes) {
                    let axisNative = this._angularIdxToNative(axIdx);
                    nativeLimits.push({
                        axis: axisNative,
                        minLimit: l.min,
                        maxLimit: l.max,
                        stiffness: l.springConstant,
                        damping: l.springDamping
                    });

                    if (l.drive) {
                        motors.push({ axis: axisNative, drive: l.drive });
                    }
                }
            }
        }


        const axisA = (new Vector3(1,0,0)).applyRotationQuaternion(pivotOrientationInA);
        const axisB = (new Vector3(1,0,0)).applyRotationQuaternion(pivotOrientationInB);

        const perpAxisA = (new Vector3(0,1,0)).applyRotationQuaternion(pivotOrientationInA);
        const perpAxisB = (new Vector3(0,1,0)).applyRotationQuaternion(pivotOrientationInB);
        var constraintInstance = new Physics6DoFConstraint( {
            pivotA: pivotTranslationInA, pivotB: pivotTranslationInB,
            axisA: axisA, axisB: axisB, perpAxisA: perpAxisA, perpAxisB: perpAxisB}, nativeLimits, this.loader.babylonScene);
        //<todo addConstraint() should allow for a null body
        rbA.physicsBody!.addConstraint(rbB!.physicsBody!, constraintInstance);
        constraintInstance.isCollisionsEnabled = !!joint.jointInfo!.enableCollision;

        for (let m of motors) {
            if (m.drive.velocityTarget != undefined) {
                constraintInstance.setAxisMotorType(m.axis, PhysicsConstraintMotorType.VELOCITY);
                constraintInstance.setAxisMotorTarget(m.axis, m.drive.velocityTarget);
                if (m.drive.maxForce) {
                    constraintInstance.setAxisMotorMaxForce(m.axis, m.drive.maxForce);
                } else {
                    constraintInstance.setAxisMotorMaxForce(m.axis, 3.4e38);
                }
            }
            if (m.drive.positionTarget != undefined) {
                constraintInstance.setAxisMotorType(m.axis, PhysicsConstraintMotorType.POSITION);
                constraintInstance.setAxisMotorTarget(m.axis, m.drive.positionTarget);
                if (m.drive.maxForce) {
                    constraintInstance.setAxisMotorMaxForce(m.axis, m.drive.maxForce);
                } else {
                    constraintInstance.setAxisMotorMaxForce(m.axis, 3.4e38);
                }
            }
        }
    }

    protected _linearIdxToNative(idx: number) : PhysicsConstraintAxis
    {
        if (idx == 0) return PhysicsConstraintAxis.LINEAR_X;
        if (idx == 1) return PhysicsConstraintAxis.LINEAR_Y;
        if (idx == 2) return PhysicsConstraintAxis.LINEAR_Z;
        throw new RangeError("Axis index out of range");
    }

    protected _angularIdxToNative(idx: number) : PhysicsConstraintAxis
    {
        if (idx == 0) return PhysicsConstraintAxis.ANGULAR_X;
        if (idx == 1) return PhysicsConstraintAxis.ANGULAR_Y;
        if (idx == 2) return PhysicsConstraintAxis.ANGULAR_Z;
        throw new RangeError("Axis index out of range");
    }

    protected _getParentRigidBody(node : TransformNode | null) : Nullable<TransformNode>
    {
        var highestParentRb : Nullable<TransformNode> = null;
        while(node != null)
        {
            if('physicsBody' in node && node.physicsBody != null)
            {
                highestParentRb = node;
            }
            node = <TransformNode>node.parent;
        }

        return highestParentRb;
    }
}
