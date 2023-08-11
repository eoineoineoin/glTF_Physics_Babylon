import { Scene, Camera, Mesh, TransformNode, Vector3, Vector4, Matrix, Ray, Quaternion} from "@babylonjs/core";
import { PhysicsEngine, PhysicsBody, PhysicsMotionType,
    PhysicsMassProperties, PhysicsRaycastResult } from "@babylonjs/core";
import '@babylonjs/core/Physics/v2/physicsEngineComponent';
import { HighlightLayer } from "@babylonjs/core";

export class PhysicsMouseSpring
{
    private hitBody? : PhysicsBody = null;
    private hitInstanceIndex: number = -1;
    private pickInBodySpace : Vector3 = Vector3.Zero();
    private hitDistance : number = 0;
    public springConstant : number = 500;
    public reelIn = false;
    public raycastResult = new PhysicsRaycastResult;
    private highlight: HighlightLayer = null;

    PhysicsMouseSpring()
    {
    }

    public pickCamera(scene : Scene, camera : Camera) {
        var ray = scene.createPickingRay(scene.pointerX, scene.pointerY, null, camera);
        this.pickRay(scene, ray);
    }

    public pickRay(scene : Scene, ray : Ray)
    {
        if (this.hitBody != null)
            return; // Can be hit due to key repeats

        let rayEnd = ray.origin.add(ray.direction.scale(Math.min(ray.length, 1e3)));
        (scene.getPhysicsEngine() as PhysicsEngine).raycastToRef(
            ray.origin, rayEnd, this.raycastResult);

        if (this.raycastResult.hasHit) {
            if (this.raycastResult.body.getMotionType(this.raycastResult.bodyIndex) == PhysicsMotionType.DYNAMIC) {
                this.hitBody = this.raycastResult.body;
                this.hitInstanceIndex = this.raycastResult.bodyIndex;

                var bodyFromWorld = Matrix.Invert(this._getWorldFromHit());
                this.pickInBodySpace = Vector3.TransformCoordinates(
                    this.raycastResult.hitPointWorld, bodyFromWorld);
                this.hitDistance = this.raycastResult.hitDistance;

                scene.defaultCursor = "pointer";
            }

            if (!this.highlight) {
                this.highlight = new HighlightLayer("hl1", scene, {
                    isStroke: true,
                }); 
            }
            // disabled; this only works with Mesh not InstanceMesh, so is inconsistent
            /*
            for (let m of this.hitBody.transformNode.getChildMeshes()) {
                if (m instanceof Mesh) {
                    this.highlight.addMesh(m, new Color3(0.91, 0.75, 0.09));
                }
            }*/
        }
        else {
            this.hitBody = null;
        }
    }

    public release() {
        if (this.hitBody) {
            this.hitBody.transformNode.getScene().defaultCursor = "auto";
        }
        this.hitBody = null;
    }

    public stepCamera(scene : Scene, camera : Camera) {
        var ray = scene.createPickingRay(scene.pointerX, scene.pointerY, null, camera);
        this.stepRay(scene, ray);
    }

    public stepRay(scene : Scene, ray : Ray) {
        if (this.hitBody == null)
            return;

        if (this.reelIn && this.hitDistance > 0.5) {
            this.hitDistance *= 0.9;
        }

        var desiredPickEnd = ray.origin.add(ray.direction.scale(this.hitDistance));
        var currentPickEnd = Vector3.TransformCoordinates(
            this.pickInBodySpace, this._getWorldFromHit());
        var forceDir = desiredPickEnd.subtract(currentPickEnd);

        if (this.hitBody!= null) {
            const dt = scene.deltaTime / 1000.0; // Scene.deltaTime is in milliseconds
            const invdt = 1.0 / dt;
            const stiffness = 0.1;
            const damping = 0.5;
            const bodyDamping = 3;

            // Damp current velocity
            {
                const gain = 1 - bodyDamping * dt;
                let lv = new Vector3();
                this.hitBody.getLinearVelocityToRef(lv, this.hitInstanceIndex);
                let av = new Vector3();
                this.hitBody.getAngularVelocityToRef(av, this.hitInstanceIndex);

                this.hitBody.setLinearVelocity(lv.scale(gain), this.hitInstanceIndex);
                this.hitBody.setAngularVelocity(av.scale(gain), this.hitInstanceIndex);
            }

            let mp = this.hitBody.getMassProperties(this.hitInstanceIndex);

            let lv = new Vector3();
            this.hitBody.getLinearVelocityToRef(lv, this.hitInstanceIndex);
            let av = new Vector3();
            this.hitBody.getAngularVelocityToRef(av, this.hitInstanceIndex);
            let comWorld = Vector3.TransformCoordinates(
                this.hitBody.getMassProperties().centerOfMass,
                this._getWorldFromHit());

            let relPos = currentPickEnd.subtract(comWorld);
            let arm = av.cross(relPos);
            let pointVel = arm.add(lv);

            let delta = forceDir.scale(stiffness * invdt);
            delta = delta.subtract(pointVel.scale(damping));

            let effMass = this._buildMassMatrixAt(
                currentPickEnd, comWorld, this.hitBody.transformNode, mp);
            let impulse = Vector3.TransformNormal(delta, effMass);
            if (mp.mass > 0) {
                const maxAccel = 250;
                let maxImpulse = mp.mass * dt * maxAccel;
                if (impulse.lengthSquared() > maxImpulse * maxImpulse) {
                    impulse.scaleInPlace(maxImpulse / impulse.length());
                }
            }
            this.hitBody.applyImpulse(impulse, currentPickEnd, this.hitInstanceIndex);
        }
    }

    protected _getWorldFromHit(): Matrix {
        if (this.hitBody == null) {
            return Matrix.Identity();
        }

        const tn = this.hitBody.transformNode;
        if (tn instanceof Mesh && tn.hasThinInstances) {
            const matrixData = tn._thinInstanceDataStorage.matrixData;
            let ret = new Matrix();
            const baseIdx = 16 * this.hitInstanceIndex;
            for (let r = 0; r < 4; r++) {
                ret.setRowFromFloats(r, 
                    matrixData[baseIdx + r * 4 + 0],
                    matrixData[baseIdx + r * 4 + 1],
                    matrixData[baseIdx + r * 4 + 2],
                    matrixData[baseIdx + r * 4 + 3]);
            }
            return ret;
        }

        return tn.computeWorldMatrix();
    }

    protected _buildMassMatrixAt(position: Vector3, com: Vector3, tn: TransformNode, mp: PhysicsMassProperties): Matrix {
        const invMass = mp.mass == 0 ? 1 : 1.0 / mp.mass;
        const r = position.subtract(com);
        const rhat = this._crossSkewSymmetricMat(r);
        const inertiaInvWorld = this._getInvInertiaWorld(tn, mp);
        const invMassMat = this._diagMat(new Vector3(invMass, invMass, invMass));

        const temp = rhat.multiply(inertiaInvWorld.multiply(rhat));
        return Matrix.Invert(this._subtract(invMassMat, temp));
    }

    protected _subtract(m1: Matrix, m2:Matrix): Matrix {
        let ret = new Matrix();
        for (let i = 0; i < 3; i++) {
            ret.setRow(i, m1.getRow(i).subtract(m2.getRow(i)));
        }
        ret.setRowFromFloats(3, 0, 0, 0, 1);
        return ret;
    }

    protected _diagMat(v: Vector3): Matrix {
        let ret = Matrix.Zero();
        ret.setRowFromFloats(0, v.x, 0, 0, 0);
        ret.setRowFromFloats(1, 0, v.y, 0, 0);
        ret.setRowFromFloats(2, 0, 0, v.z, 0);
        ret.setRowFromFloats(3, 0, 0, 0, 1);
        return ret;
    }

    protected _getInvInertiaWorld(tn: TransformNode, mp: PhysicsMassProperties): Matrix {
        const rotationQ = mp.inertiaOrientation.multiply(tn.rotationQuaternion);
        let rotation = new Matrix();
        Matrix.FromQuaternionToRef(rotationQ, rotation);

        const mass = mp.mass == 0 ? 1 : mp.mass;
        const inv_ix = mp.inertia.x == 0 ? 0 : 1.0 / (mp.inertia.x * mass);
        const inv_iy = mp.inertia.y == 0 ? 0 : 1.0 / (mp.inertia.y * mass);
        const inv_iz = mp.inertia.z == 0 ? 0 : 1.0 / (mp.inertia.z * mass);
        const invInertiaLocal = new Vector4(inv_ix, inv_iy, inv_iz, 1);

        let inm = new Matrix();
        inm.setRow(0, invInertiaLocal.multiply(rotation.getRow(0)));
        inm.setRow(1, invInertiaLocal.multiply(rotation.getRow(1)));
        inm.setRow(2, invInertiaLocal.multiply(rotation.getRow(2)));
        inm.setRow(3, invInertiaLocal.multiply(rotation.getRow(3)));

        return inm.multiply(Matrix.Invert(rotation));
    }

    protected _crossSkewSymmetricMat(r: Vector3): Matrix {
        let m = Matrix.Zero();
        m.setRowFromFloats(0, 0, -r.z, r.y, 0);
        m.setRowFromFloats(1, r.z, 0, -r.x, 0);
        m.setRowFromFloats(2, -r.y, r.x, 0, 0);
        m.setRowFromFloats(3, 0, 0, 0, 1);
        return m;
    }
}
