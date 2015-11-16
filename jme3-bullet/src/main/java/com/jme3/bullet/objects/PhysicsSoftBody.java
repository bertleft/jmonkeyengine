/*
 * Copyright (c) 2009-2015 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.jme3.bullet.objects;

import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.objects.infos.SoftBodyWorldInfo;
import com.jme3.bullet.util.DebugMeshCallback;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author dokthar
 */
public class PhysicsSoftBody extends PhysicsCollisionObject {

    private final Config config = new Config(this);

    @Deprecated
    public PhysicsSoftBody(Vector3f[] vertices, float[] masses) {
        objectId = ctr_PhysicsSoftBody(vertices.length, vertices, masses);
        postRebuild(false);
    }

    public PhysicsSoftBody() {
        objectId = ctr_PhysicsSoftBody();
        initDefault();
        postRebuild(false);
    }

    public PhysicsSoftBody(Mesh triMesh) {
// must check if the mesh have %3 vertices
        createFromTriMesh(triMesh);
    }

    // SoftBodies need to have a direct access to JME's Mesh buffer in order a have a more
    // efficient way when updating the Mesh each frame
    private long createFromTriMesh(Mesh triMesh) {
        IntBuffer indexBuffer = BufferUtils.createIntBuffer(triMesh.getTriangleCount() * 3);
        FloatBuffer positionBuffer = triMesh.getFloatBuffer(Type.Position);

        IndexBuffer indices = triMesh.getIndicesAsList();
        int indicesLength = triMesh.getTriangleCount() * 3;
        for (int i = 0; i < indicesLength; i++) { // done because mesh indexs can use short or i am wrong
            indexBuffer.put(indices.get(i));
        }
        //Add
        objectId = createFromTriMesh(indexBuffer, positionBuffer, triMesh.getTriangleCount(), false);
        postRebuild(false);
        return objectId;
    }

    private native long createFromTriMesh(IntBuffer triangles, FloatBuffer vertices, int numTriangles, boolean randomizeConstraints);

    private native long ctr_PhysicsSoftBody();

    private native long ctr_PhysicsSoftBody(int size, Vector3f[] vertices, float[] mass);

    protected void rebuildFromTriMesh(Mesh mesh) {
        // {mesh != null} => {old Native object is removed & destroyed; new Native object is created & added}
        boolean wasInWorld = isInWorld();
        preRebuild();
        objectId = createFromTriMesh(mesh);
        postRebuild(wasInWorld);
    }

    protected final void preRebuild() {
        // {} = > {remove the body from the physics space and detroy the native object}

        /* if (collisionShape instanceof MeshCollisionShape && mass != 0) {
         throw new IllegalStateException("Dynamic rigidbody can not have mesh collision shape!");
         }*/
        if (objectId != 0) {
            if (isInWorld()) {
                PhysicsSoftSpace.getPhysicsSoftSpace().remove(this);
            }
            Logger.getLogger(this.getClass().getName()).log(Level.FINE, "Clearing RigidBody {0}", Long.toHexString(objectId));
            finalizeNative(objectId);
        }
    }

    protected final void postRebuild(boolean wasInWorld) {
        // {} => {initUserPoint on the native object, if this wasInWorld add this into the physicsSpace}
        initUserPointer();
        if (wasInWorld) {
            PhysicsSoftSpace.getPhysicsSoftSpace().add(this);
        }
    }

    private void initDefault() {
        initDefault(objectId);
    }

    private native void initDefault(long objectId);

    /**
     * Set the SoftBodyWorldInfo of this physics softbody. This is automatically
     * called when the softbody is added into a SoftPhysicsSpace.
     *
     * @param worldinfo , the world info to set.
     */
    public void setSoftBodyWorldInfo(SoftBodyWorldInfo worldinfo) {
        setSoftBodyWorldInfo(objectId, worldinfo.getWorldInfoId());
    }

    private native void setSoftBodyWorldInfo(long objectId, long worldinfoId);

    /**
     * Return the SoftBodyWorldInfo used by this softbody. By default this
     * SoftBodyWorld info is shared by all softBodies into the SoftPhysicsSpace.
     *
     * @return the used SoftBodyWorldInfo
     */
    public SoftBodyWorldInfo getSoftBodyWorldInfo() {
        long worldInfoId = getSoftBodyWorldInfo(objectId);
        SoftBodyWorldInfo worldInfo = new SoftBodyWorldInfo(worldInfoId); // <-point on the same native object
        return worldInfo;
    }

    private native long getSoftBodyWorldInfo(long objectId);

    /**
     * Create a new material for this softbody (natively).
     *
     * @return the created material.
     */
    public Material appendMaterial() {
        long matId = appendMaterial(objectId);
        return new Material(matId);
    }

    private native long appendMaterial(long objectId);

    /**
     * Get the list of all materials on this softbody.
     *
     * @return a ArrayList of Material
     */
    public ArrayList<Material> getMaterialList() {
        long matIds[] = getMaterials(objectId);
        ArrayList<Material> matList = new ArrayList<Material>(matIds.length);
        for (int i = 0; i < matIds.length; i++) {
            matList.add(new Material(matIds[i]));
        }
        return matList;
    }

    private native long[] getMaterials(long bodyId);


    /* Append anchor */
 /* public void appendAnchor(int node, PhysicsRigidBody rigidBody, boolean collisionBetweenLinkedBodies, float influence) {

     }

     public void appendAnchor(int node, PhysicsRigidBody rigidBody, Vector3f localPivot, boolean collisionBetweenLinkedBodies, float influence) {

     }*/

 /* Append linear joint	*/
 /*public void appendLinearJoint(const LJoint::Specs& specs,Cluster* body0,Body body1){
    
     }
     public void appendLinearJoint(const LJoint::Specs& specs,Body body=Body()) {
    
     }
     public void appendLinearJoint(const LJoint::Specs& specs,btSoftBody* body) {
    
     }
     */
 /* Append linear joint	*/
// public void appendAngularJoint(const AJoint::Specs& specs,Cluster* body0,Body body1);
// public void appendAngularJoint(const AJoint::Specs& specs,Body body=Body());
// public void appendAngularJoint(const AJoint::Specs& specs,btSoftBody* body);
    /**
     * Add force (or gravity) to the entire body.
     *
     * @param force the force to add.
     */
    public void addForce(Vector3f force) {
        addForce(objectId, force);
    }

    private native void addForce(long objectId, Vector3f force);

    /**
     * Add force (or gravity) to specific node (vertex) of the body.
     *
     * @param force the force to add.
     * @param node the vertex index
     */
    public void addForce(Vector3f force, int node) {
        addForce(objectId, force, node);
    }

    private native void addForce(long objectId, Vector3f force, int node);

    /**
     * Add aero force to specific node (vertex) of the body.
     *
     * @param windVelocity the aero force to add.
     * @param nodeIndex the vertex index
     */
    public void addAeroForceToNode(Vector3f windVelocity, int nodeIndex) {
        addAeroForceToNode(objectId, windVelocity, nodeIndex);
    }

    private native void addAeroForceToNode(long objectId, Vector3f windVelocity, int nodeIndex);

    /* Add aero force to a face of the body */
    public void addAeroForceToFace(Vector3f windVelocity, int faceIndex) {
        throw new UnsupportedOperationException("Not supported yet. No face index availiable");
//        addAeroForceToFace(objectId, windVelocity, faceIndex);
    }

    private native void addAeroForceToFace(long objectId, Vector3f windVelocity, int faceIndex);

    /**
     * Add velocity to the entire body.
     *
     * @param velocity the value to add.
     */
    public void addVelocity(Vector3f velocity) {
        addVelocity(objectId, velocity);
    }

    private native void addVelocity(long objectId, Vector3f velocity);

    /**
     * Set velocity for the entire body
     *
     * @param velocity the value to set.
     */
    public void setVelocity(Vector3f velocity) {
        setVelocity(objectId, velocity);
    }

    private native void setVelocity(long objectId, Vector3f velocity);

    /**
     * Add velocity to a node (vertex) of the body.
     *
     * @param velocity the value to add.
     * @param node the vertex index.
     */
    public void addVelocity(Vector3f velocity, int node) {
        addVelocity(objectId, velocity, node);
    }

    private native void addVelocity(long objectId, Vector3f velocity, int nodeId);

    /**
     * Set the mass for a given node (vertex) of the body.
     *
     * @param node
     * @param mass
     */
    public void setMass(int node, float mass) {
        setMass(objectId, node, mass);
    }

    private native void setMass(long objectId, int node, float mass);

    /**
     * Get the mass of a specific node (vertex).
     *
     * @param node the vertex index
     * @return the mass of the node
     */
    public float getMass(int node) {
        return getMass(objectId, node);
    }

    private native float getMass(long objectId, int node);

    /**
     * Get the total mass of the body.
     *
     * @return the total mass.
     */
    public float getTotalMass() {
        return getTotalMass(objectId);
    }

    private native float getTotalMass(long objectId);

    /**
     * Set the total mass (weighted by previous masses) for the body.
     *
     * @param mass the total mass to set
     * @param fromfaces, weighted by faces surface instead of previous masses.
     */
    public void setTotalMass(float mass, boolean fromfaces) {
        setTotalMass(objectId, mass, fromfaces);
    }

    /**
     * Set the total mass (weighted by previous masses) for the body.
     *
     * @param mass the total mass to set
     */
    public void setTotalMass(float mass) {
        setTotalMass(mass, false);
    }

    private native void setTotalMass(long objectId, float mass, boolean fromFaces);

    /**
     * Set the total mass of the body based on the density. (weighted by the
     * density * volume)
     *
     * @param density
     */
    public void setTotalDensity(float density) {
        setTotalDensity(objectId, density);
    }

    private native void setTotalDensity(long objectId, float density);

    /**
     * Set volume mass (using tetrahedrons)
     *
     * @param mass
     */
    public void setVolumeMass(float mass) {
        setVolumeMass(objectId, mass);
    }

    private native void setVolumeMass(long objectId, float mass);

    /**
     * Set volume density (using tetrahedrons)
     *
     * @param density
     */
    public void setVolumeDensity(float density) {
        setVolumeDensity(objectId, density);
    }

    private native void setVolumeDensity(long objectId, float density);

    /* Native transform / rotation / translation / scale*/
    /**
     * Transform the body.
     *
     * @param trs the transform to apply.
     */
    public void applyPhysicsTransform(Transform trs) {
        applyPhysicsTransform(objectId, trs);
    }

    private native void applyPhysicsTransform(long objectId, Transform trs);

    /**
     * Translate (or move) the body.
     *
     * @param vec the translation to apply.
     */
    public void applyPhysicsTranslate(Vector3f vec) {
        applyPhysicsTranslate(objectId, vec);
    }

    private native void applyPhysicsTranslate(long objectId, Vector3f vec);

    /**
     * Rotate the body.
     *
     * @param rot the rotation to apply.
     */
    public void applyPhysicsRotation(Quaternion rot) {
        applyPhysicsRotation(objectId, rot);
    }

    private native void applyPhysicsRotation(long objectId, Quaternion rot);

    /**
     * Scale the body. To use carefully.
     *
     * @param vec the scaling to apply.
     */
    public void applyPhysicsScale(Vector3f vec) {
        applyPhysicsScale(objectId, vec);
    }

    private native void applyPhysicsScale(long objectId, Vector3f vec);

    /* =======================
     * These methods are not native. ( set-getPhysicsTransform/Location)
     * and will only work if a center can be determinated
     * ======================= */

 /* Transform */
    public void setPhysicsTransform(Transform trs) {
        setPhysicsTransform(objectId, trs);
    }

    private native void setPhysicsTransform(long objectId, Transform trs);

    public Transform getPhysicsTransform() {
        Transform trs = new Transform();
        getPhysicsTransform(objectId, trs);
        return trs;
    }

    private native void getPhysicsTransform(long objectId, Transform trs);

    /* Translate */
    public void setPhysicsLocation(Vector3f vec) {
        setPhysicsLocation(objectId, vec);
    }

    private native void setPhysicsLocation(long objectId, Vector3f vec);

    public Vector3f getPhysicsLocation(Vector3f store) {
        if (store == null) {
            store = new Vector3f();
        }
        getPhysicsLocation(objectId, store);
        return store;
    }

    public Vector3f getPhysicsLocation() {
        return getPhysicsLocation(null);
    }

    private native void getPhysicsLocation(long objectId, Vector3f vec);

    /* Rotate */
    public void setPhysicsRotation(Quaternion rot) {
        setPhysicsRotation(objectId, rot);
    }

    private native void setPhysicsRotation(long objectId, Quaternion rot);

    public Quaternion getPhysicsRotation(Quaternion store) {
        if (store == null) {
            store = new Quaternion();
        }
        getPhysicsRotation(objectId, store);
        return store;
    }

    public Quaternion getPhysicsRotation() {
        return getPhysicsRotation(null);
    }

    private native void getPhysicsRotation(long objectId, Quaternion rot);

    /**
     * Get link resting lengths scale.
     *
     * @return the resting lengths scale.
     */
    public float getRestLengthScale() {
        return getRestLenghtScale(objectId);
    }

    private native float getRestLenghtScale(long objectId);

    /**
     * Scale resting length of all springs
     *
     * @param scale
     */
    public void setRestLengthScale(float scale) {
        setRestLenghtScale(objectId, scale);
    }

    private native void setRestLenghtScale(long objectId, float scale);

    /**
     * Set current state of the softbody as "default pose" or "lowest energy
     * state".
     *
     * @param bvolume, boolean volume pose
     * @param bframe, boolean frame pose
     */
    public void setPose(boolean bvolume, boolean bframe) {
        setPose(objectId, bvolume, bframe);
    }

    private native void setPose(long objectId, boolean bvolume, boolean bframe);

    /**
     * Set current link lengths as the resting lengths.
     */
    public void resetLinkRestLengths() {
        resetLinkRestLengths(objectId);
    }

    private native void resetLinkRestLengths(long objectId);

    /**
     * Get the volume of the body.
     *
     * @return the volume.
     */
    public float getVolume() {
        return getVolume(objectId);
    }

    private native float getVolume(long objectId);

    /**
     * Get the cluster count, the number of cluster used by this body.
     *
     * @return the number of cluster.
     */
    public int getClusterCount() {
        return getClusterCount(objectId);
    }

    private native int getClusterCount(long objectId);

    /**
     * Generate bending constraints based on distance in the adjency graph.
     *
     * @param distance, greater than 1 (else do nothing)
     * @param mat , the material to append links
     */
    public void generateBendingConstraints(int distance, Material mat) {
        generateBendingConstraints(objectId, distance, mat.materialId);
    }

    private native void generateBendingConstraints(long bodyId, int distance, long matId);

    /**
     * Randomize constraints to reduce solver bias.
     */
    public void randomizeConstraints() {
        randomizeConstraints(objectId);
    }

    private native void randomizeConstraints(long objectId);

    /**
     * Release a cluster.
     *
     * @param index, the index of the cluster to remove.
     */
    public void releaseCluster(int index) {
        releaseCluster(objectId, index);
    }

    private native void releaseCluster(long objectId, int index);

    /**
     * Release all the clusters.
     */
    public void releaseClusters() {
        releaseClusters(objectId);
    }

    private native void releaseClusters(long objectId);

    /**
     * Generate clusters (K-mean) : generateClusters with k=0 will create a
     * convex cluster for each tetrahedron or triangle, otherwise an
     * approximation will be used (better performance). By default maxiterations
     * is set to 8192.
     *
     * @param k, the number of cluster to create, can't be bigger than the
     * number of nodes.
     */
    public void generateClusters(int k) {
        generateClusters(k, 8192);
    }

    /**
     * Generate clusters (K-mean) : generateClusters with k=0 will create a
     * convex cluster for each tetrahedron or triangle, otherwise an
     * approximation will be used (better performance).
     *
     * @param k, the number of cluster to create, can't be bigger than the
     * number of nodes.
     * @param maxiterations, the maximum of iterations, used for the generation
     * of cluster. By default maxiterations is set to 8192.
     */
    public void generateClusters(int k, int maxiterations) {
        generateClusters(objectId, k, maxiterations);
    }

    private native void generateClusters(long objectId, int k, int maxiterations);

    /**
     * Tell if the current softBody is in added into a physicsSpace.
     *
     * @return true if the softBody is in a physicsSpace.
     */
    public boolean isInWorld() {
        return isInWorld(objectId);
    }

    private native boolean isInWorld(long objectId);

    /**
     * Get the center of the bouding volume of this body. This "center" may not
     * be exactly in the center of the object as it's move. The value of the
     * boundingCenter isn't updated each frame.
     *
     * @return the position of the bounding center.
     */
    public Vector3f getBoundingCenter() {
        Vector3f result = new Vector3f();
        getBoundingCenter(objectId, result);
        return result;
    }

    private native void getBoundingCenter(long bodyId, Vector3f store);

    /**
     * Get the config object which hold methods to access to the native config
     * fields.
     *
     * @return the config object of this softbody.
     */
    public Config config() {
        return this.config;
    }

    /**
     * This class hold methods to access (get and set) natives fields of the
     * Config struct (of btSoftBody). This class is final. And the constructor
     * is private, in order to only have the access through a PhysicsSoftBody,
     * each hysicsSoftBody have only one Config object associated with.
     */
    public final class Config {

        private final PhysicsSoftBody body;

        // /!\ the objectId field from softbody is directly used here because it's a protected field.
        private Config(PhysicsSoftBody body) {
            super();
            this.body = body;
        }

        /**
         * Copy the values of the Config conf, into this Config.
         *
         * @param conf the Config with the used values.
         */
        public void copyValues(Config conf) {
            copyValues(objectId, conf.body.objectId);
        }

        private native void copyValues(long thisId, long bodyId);

        /**
         * Set the velocities correction factor (Baumgarte), (aka kVCF).
         *
         * @param factor the value to set.
         */
        public void setVelocitiesCorrectionFactor(float factor) {
            setVelocitiesCorrectionFactor(objectId, factor);
        }

        private native void setVelocitiesCorrectionFactor(long bodyId, float factor);

        /**
         * Get the velocities correction factor (Baumgarte), (aka kVCF).
         *
         * @return the factor value.
         */
        public float getVelocitiesCorrectionFactor() {
            return getVelocitiesCorrectionFactor(objectId);
        }

        private native float getVelocitiesCorrectionFactor(long bodyId);

        /**
         * Set the damping coefficient (aka kDP).
         *
         * @param coefficient the value to set, between [0,1].
         */
        public void setDampingCoef(float coefficient) {
            setDampingCoef(objectId, coefficient);
        }

        private native void setDampingCoef(long bodyId, float coefficient);

        /**
         * Get the damping coefficient (aka kDP).
         *
         * @return the coefficient value.
         */
        public float getDampingCoef() {
            return getDampingCoef(objectId);
        }

        private native float getDampingCoef(long bodyId);

        /**
         * Set the drag coefficient (aka kDG).
         *
         * @param coefficient the value to set, between [0,+inf].
         */
        public void setDragCoef(float coefficient) {
            setDragCoef(objectId, coefficient);
        }

        private native void setDragCoef(long bodyId, float coefficient);

        /**
         * Get the drag coefficient.
         *
         * @return the coefficient value.
         */
        public float getDragCoef() {
            return getDragCoef(objectId);
        }

        private native float getDragCoef(long bodyId);

        /**
         * Set the lift coefficient (aka kLF).
         *
         * @param coefficient the value to set, between [O,+inf].
         */
        public void setLiftCoef(float coefficient) {
            setLiftCoef(objectId, coefficient);
        }

        private native void setLiftCoef(long bodyId, float coefficient);

        /**
         * Get the lift coefficient (aka kLF).
         *
         * @return the coefficient value.
         */
        public float getLiftCoef() {
            return getLiftCoef(objectId);
        }

        private native float getLiftCoef(long bodyId);

        /**
         * Set the pressure coefficient (aka kPR).
         *
         * @param coefficient the value to set, between [-inf,+inf].
         */
        public void setPressureCoef(float coefficient) {
            setPressureCoef(objectId, coefficient);
        }

        private native void setPressureCoef(long bodyId, float coefficient);

        /**
         * Get the pressure coefficient (aka kPR).
         *
         * @return the coefficient value.
         */
        public float getPressureCoef() {
            return getPressureCoef(objectId);
        }

        private native float getPressureCoef(long bodyId);

        /**
         * Set the volume conservation coefficient (aka kVC).
         *
         * @param coefficient the value to set, between [0,+inf].
         */
        public void setVolumeConservationCoef(float coefficient) {
            setVolumeConservationCoef(objectId, coefficient);
        }

        private native void setVolumeConservationCoef(long bodyId, float coefficient);

        /**
         * Get the volume conservation coefficient.
         *
         * @return the coefficient value.
         */
        public float getVolumeConservationCoef() {
            return getVolumeConservationCoef(objectId);
        }

        private native float getVolumeConservationCoef(long bodyId);

        /**
         * Set the dynamic friction coefficient (aka kDF).
         *
         * @param coefficient the value to set, between [0,1].
         */
        public void setDynamicFrictionCoef(float coefficient) {
            setDynamicFrictionCoef(objectId, coefficient);
        }

        private native void setDynamicFrictionCoef(long objectId, float coefficient);

        /**
         * Get the dynamic friction coefficient (aka kDF).
         *
         * @return the coefficient value.
         */
        public float getDynamicFrictionCoef() {
            return getDynamicFrictionCoef(objectId);
        }

        private native float getDynamicFrictionCoef(long objectId);

        /**
         * Set the pose matching coefficient (aka kMT). This coefficient
         * correspond how much the softbody will try to match his "pose".
         *
         * @param coefficient the value to set, between [0,1].
         */
        public void setPoseMatchingCoef(float coefficient) {
            setPoseMatchingCoef(objectId, coefficient);
        }

        private native void setPoseMatchingCoef(long objectId, float coefficient);

        /**
         * Get the pose matching coefficient (aka kMT).
         *
         * @return the value of the coefficient.
         */
        public float getPoseMatchingCoef() {
            return getPoseMatchingCoef(objectId);
        }

        private native float getPoseMatchingCoef(long bodyId);

        /**
         * Set the rigid contacts hardness coefficient (aka kCHR).
         *
         * @param hardness the value to set, between [0,1].
         */
        public void setRigidContactsHardness(float hardness) {
            setRigidContactsHardness(objectId, hardness);
        }

        private native void setRigidContactsHardness(long bodyId, float hardness);

        /**
         * Get the rigid contacts hardness coefficient (aka kCHR).
         *
         * @return the hardness coefficient value.
         */
        public float getRigidContactsHardness() {
            return getRigidContactsHardness(objectId);
        }

        private native float getRigidContactsHardness(long bodyId);

        /**
         * Set the kinectic contacts hardness coefficient (aka kKHR).
         *
         * @param hardness the value to set, between [0,1].
         */
        public void setKineticContactsHardness(float hardness) {
            setKineticContactsHardness(objectId, hardness);
        }

        private native void setKineticContactsHardness(long bodyId, float hardness);

        /**
         * Get the kinetic contacts hardness coefficient (aka kKHR).
         *
         * @return the hardness coefficient value.
         */
        public float getKineticContactsHardness() {
            return getKineticContactsHardness(objectId);
        }

        private native float getKineticContactsHardness(long bodyId);

        /**
         * Set the soft contacts hardness coefficient (aka kSHR).
         *
         * @param hardness the value to set, between [0,1].
         */
        public void setSoftContactsHardness(float hardness) {
            setSoftContactsHardness(objectId, hardness);
        }

        private native void setSoftContactsHardness(long bodyId, float hardness);

        /**
         * Get the soft contact hardness coefficient (aka kSHR).
         *
         * @return the hardness coefficient value.
         */
        public float getSoftContactsHardness() {
            return getSoftContactsHardness(objectId);
        }

        private native float getSoftContactsHardness(long bodyId);

        /**
         * Set the anchors hardness coefficient (aka kAHR).
         *
         * @param hardness the value to set, between [0,1].
         */
        public void setAnchorsHardness(float hardness) {
            setAnchorsHardness(objectId, hardness);
        }

        private native void setAnchorsHardness(long bodyId, float hardness);

        /**
         * Get the anchors hardness coefficient (aka kAHR).
         *
         * @return the hardness coefficient value.
         */
        public float getAnchorsHardness() {
            return getAnchorsHardness(objectId);
        }

        private native float getAnchorsHardness(long bodyId);

        /**
         * Set the soft cluster versus rigid hardness coefficient(aka kSRHR_CL).
         *
         * @param hardness the value to set, between [0,1].
         */
        public void setClusterRigidHardness(float hardness) {
            setClusterRigidHardness(objectId, hardness);
        }

        private native void setClusterRigidHardness(long bodyId, float hardness);

        /**
         * Get the soft cluster versus rigid hardness coefficient(aka kSRHR_CL).
         *
         * @return the hardness coefficient value.
         */
        public float getClusterRigidHardness() {
            return getClusterRigidHardness(objectId);
        }

        private native float getClusterRigidHardness(long bodyId);

        /**
         * Set the soft cluster versus kinetic hardness coefficient (aka
         * kSKHR_CL).
         *
         * @param hardness the value to set, between [0,1].
         */
        public void setClusterKineticHardness(float hardness) {
            setClusterKineticHardness(objectId, hardness);
        }

        private native void setClusterKineticHardness(long bodyId, float hardness);

        /**
         * Get the cluster versus kinetic hardness coefficient (aka kSKHR_CL).
         *
         * @return the hardness coefficient value.
         */
        public float getClusterKineticHardness() {
            return getClusterKineticHardness(objectId);
        }

        private native float getClusterKineticHardness(long bodyId);

        /**
         * Set the soft cluster versus soft hardness coefficient (aka kSSHR_CL).
         *
         * @param hardness the value to set, between [0,1]
         */
        public void setClusterSoftHardness(float hardness) {
            setClusterSoftHardness(objectId, hardness);
        }

        private native void setClusterSoftHardness(long bodyId, float hardness);

        /**
         * Get the soft cluster versus soft hardness coefficient (aka kSSHR_CL).
         *
         * @return the hardness coefficient value.
         */
        public float getClusterSoftHardness() {
            return getClusterSoftHardness(objectId);
        }

        private native float getClusterSoftHardness(long bodyId);

        /**
         * Set the soft cluster versus rigid, impulse split coefficient (aka
         * kSR_SPLT_CL).
         *
         * @param coef the value to set, between [0,1].
         */
        public void setClusterRigidImpulseSplitCoef(float coef) {
            setClusterRigidImpulseSplitCoef(objectId, coef);
        }

        private native void setClusterRigidImpulseSplitCoef(long bodyId, float coef);

        /**
         * Get the soft cluster versus rigid, impulse split coefficient (aka
         * kSR_SPLT_CL).
         *
         * @return the impulse split coefficient value.
         */
        public float getClusterRigidImpulseSplitCoef() {
            return getClusterRigidImpulseSplitCoef(objectId);
        }

        private native float getClusterRigidImpulseSplitCoef(long bodyId);

        /**
         * Set the soft cluster versus kinetic, impulse split coefficient (aka
         * kSK_SPLT_CL).
         *
         * @param coef the value to set, between [0,1].
         */
        public void setClusterKineticImpulseSplitCoef(float coef) {
            setClusterKineticImpulseSplitCoef(objectId, coef);
        }

        private native void setClusterKineticImpulseSplitCoef(long bodyId, float coef);

        /**
         * Get the soft cluster versus kinetic, impulse split coefficient (aka
         * kSK_SPLT_CL).
         *
         * @return the impulse split coefficient value.
         */
        public float getClusterKineticImpulseSplitCoef() {
            return getClusterKineticImpulseSplitCoef(objectId);
        }

        private native float getClusterKineticImpulseSplitCoef(long bodyId);

        /**
         * Set the soft cluster versus soft, impulse split coefficient (aka
         * kSS_SPLT_CL).
         *
         * @param coef the value to set, between [0,1].
         */
        public void setClusterSoftImpulseSplitCoef(float coef) {
            setClusterSoftImpulseSplitCoef(objectId, coef);
        }

        private native void setClusterSoftImpulseSplitCoef(long bodyId, float coef);

        /**
         * Get the soft cluster versus soft, impulse split coefficient (aka
         * kSS_SPLT_CL).
         *
         * @return the impulse split coefficient value.
         */
        public float getClusterSoftImpulseSplitCoef() {
            return getClusterSoftImpulseSplitCoef(objectId);
        }

        private native float getClusterSoftImpulseSplitCoef(long bodyId);

        /**
         * Set the maximum volume ratio for pose.
         *
         * @param ratio the value to set.
         */
        public void setMaximumVolumeRatio(float ratio) {
            setMaximumVolumeRatio(objectId, ratio);
        }

        private native void setMaximumVolumeRatio(long bodyId, float ratio);

        /**
         * Get the maximum volume ratio for pose.
         *
         * @return the maximum volume ratio value.
         */
        public float getMaximumVolumeRatio() {
            return getMaximumVolumeRatio(objectId);
        }

        private native float getMaximumVolumeRatio(long bodyId);

        /**
         * Set the time scale.
         *
         * @param scale the value to set.
         */
        public void setTimeScale(float scale) {
            setTimeScale(objectId, scale);
        }

        private native void setTimeScale(long bodyId, float scale);

        /**
         * Get the time scale.
         *
         * @return the value of the time scale.
         */
        public float getTimeScale() {
            return getTimeScale(objectId);
        }

        private native float getTimeScale(long bodyId);

        /**
         * Set the velocities solver iterations (aka viterations).
         *
         * @param iterations the value to set.
         */
        public void setVelocitiesIterations(int iterations) {
            setVelocitiesIterations(objectId, iterations);
        }

        private native void setVelocitiesIterations(long objectId, int iteration);

        /**
         * Get the velocities solver iterations (aka viterations).
         *
         * @return the velocities solver iterations value.
         */
        public int getVelocitiesIterations() {
            return getVelocitiesIterations(objectId);
        }

        private native int getVelocitiesIterations(long objectId);

        /**
         * Set the positions solver iterations (aka piterations).
         *
         * @param iterations the value to set.
         */
        public void setPositionIterations(int iterations) {
            setPositionIterations(objectId, iterations);
        }

        private native void setPositionIterations(long objectId, int iteration);

        /**
         * Get the positions solver iterations (aka piterations).
         *
         * @return the positions solver iterations value.
         */
        public int getPositionIterations() {
            return getPositionIterations(objectId);
        }

        private native int getPositionIterations(long objectId);

        /**
         * Set the drift solver iterations (aka diterations).
         *
         * @param iterations the value to set.
         */
        public void setDriftIterations(int iterations) {
            setDriftIterations(objectId, iterations);
        }

        private native void setDriftIterations(long objectId, int iteration);

        /**
         * Get the drift solver iterations (aka diterations).
         *
         * @return the drift solver iterations value.
         */
        public int getDriftIterations() {
            return getDriftIterations(objectId);
        }

        private native int getDriftIterations(long objectId);

        /**
         * Set the cluster solver iterations (aka citerations).
         *
         * @param iterations the value to set.
         */
        public void setClusterIterations(int iterations) {
            setClusterIterations(objectId, iterations);
        }

        private native void setClusterIterations(long objectId, int iteration);

        /**
         * Get the cluster solver iterations (aka citerations).
         *
         * @return the cluster solver iterations value.
         */
        public int getClusterIterations() {
            return getClusterIterations(objectId);
        }

        private native int getClusterIterations(long objectId);

        /**
         * Config collision flag : Rigid versus soft mask.
         */
        public final static int RVSmask = 0x000f;

        /**
         * Config collision flag : SDF based rigid vs soft.
         */
        public final static int SDF_RS = 0x0001;

        /**
         * Config collision flag : Cluster vs convex rigid vs soft.
         */
        public final static int CL_RS = 0x0002;

        /**
         * Config collision flag : Soft versus soft mask.
         */
        public final static int SVSmask = 0x0030;

        /**
         * Config collision flag : Vertex vs face soft cluster vs soft handling.
         */
        public final static int VF_SS = 0x0010;

        /**
         * Config collision flag : Cluster vs cluster soft vs soft handling.
         */
        public final static int CL_SS = 0x0020;

        /**
         * Config collision flag : Cluster soft body self collision.
         */
        public final static int CL_SELF = 0x0040;

        /**
         * Config collision flag : Default value = SDF_RS. (SDF based rigid vs
         * soft)
         */
        public final static int Default = SDF_RS;

        /**
         * Set the collision flag as the value of flags. Avaliable flags values
         * are defined into the Config class.
         *
         * @param flag the flag value to set.
         * @param flags ... optional values of flags, if there are values they
         * will be added to the flag value by a bitwise inclusive OR.
         */
        public void setCollisionsFlags(int flag, int... flags) {
            int allFlags = flag;
            for (int i = 0; i < flags.length; i++) {
                allFlags |= flags[i];
            }
            setCollisionsFlags(objectId, allFlags);
        }

        private native void setCollisionsFlags(long bodyId, int flags);

        /**
         * Get the collisions flags value.
         *
         * @return the value of the collisions flags.
         */
        public int getCollisionsFlags() {
            return getCollisionsFlags(objectId);
        }

        private native int getCollisionsFlags(long bodyId);

    };

    /*====================*  
     SoftBody to Mesh - UTILS 
     *====================*/ /*
     Since bullet SoftBody don't use btCollisionShape, its not possible to use the DebugShapeFactory.
     The following code is almost the same , but specially for SoftBody. 
     Theses methods are static (same as in DebugShapeFactory) so the code can be easily moved somewhere else.
     */

    public static Geometry createDebugShape(PhysicsSoftBody softBody) {
        if (softBody == null) {
            return null;
        }
        Geometry debugShape = new Geometry();
        debugShape.setMesh(getDebugMesh(softBody));
        debugShape.updateModelBound();
        debugShape.updateGeometricState();
        return debugShape;
    }

    public static Mesh getDebugMesh(PhysicsSoftBody softBody) {
        Mesh mesh = new Mesh();
        mesh.setBuffer(Type.Position, 3, getVertices(softBody));
        mesh.setBuffer(Type.Index, 3, getIndexes(softBody));
        mesh.getFloatBuffer(Type.Position).clear();
        return mesh;
    }

    private static FloatBuffer getVertices(PhysicsSoftBody softBody) {
        DebugMeshCallback callback = new DebugMeshCallback();
        getVertices(softBody.getObjectId(), callback);
        return callback.getVertices();
    }

    private static native void getVertices(long bodyId, DebugMeshCallback buffer);

    private static IntBuffer getIndexes(PhysicsSoftBody softBody) {
        IntBuffer indexes = BufferUtils.createIntBuffer(getNumTriangle(softBody.getObjectId()) * 3);
        getIndexes(softBody.getObjectId(), indexes);
        return indexes;
    }

    private static native void getIndexes(long bodyId, IntBuffer buffer);

    private static native int getNumTriangle(long bodyId);

    public static void updateMesh(PhysicsSoftBody softBody, Mesh store, boolean meshInLocalOrigin, boolean updateNormals) {
        FloatBuffer positionBuffer = store.getFloatBuffer(Type.Position);
        FloatBuffer normalBuffer = store.getFloatBuffer(Type.Normal);
        updateMesh(softBody.getObjectId(), positionBuffer, meshInLocalOrigin, updateNormals, normalBuffer);
        store.getBuffer(Type.Position).setUpdateNeeded();
        if (updateNormals) {
            store.getBuffer(Type.Normal).setUpdateNeeded();
        }
    }

    private static native void updateMesh(long bodyId, FloatBuffer vertices, boolean meshInLocalOrigin, boolean updateNormals, FloatBuffer normals);

    /**
     * The Material class hold methods to access (get and set) natives fields of
     * the Material struct (of btSoftBody). This class is final.
     * <p>
     * The constructor is protected and should only used internally. The only
     * way to create new Material is to use the method <code> appendMaterial()
     * </code> from a PhysicsSoftBody. This way a new native Material will be
     * created and added to the body (as done in native).</p>
     * <p>
     * Native materials have a field m_flags, this fields is not available
     * through this binding because the only flag is DebugDraw and it's will be
     * disable by default when appending a new Material (done with the binding
     * not native).</p>
     */
    public final class Material {

        private long materialId;

        protected Material(long nativeId) {
            this.materialId = nativeId;
        }

        @Override
        public boolean equals(Object obj) {
            if (obj.getClass() == Material.class) {
                return this.materialId == ((Material) obj).materialId;
            } else {
                return false;
            }
        }

        /**
         * Get the linear stiffness coefficient (aka m_KLST).
         *
         * @return the linearStiffnessFactor
         */
        public float getLinearStiffnessFactor() {
            return getLinearStiffnessFactor(materialId);
        }

        private native float getLinearStiffnessFactor(long materialId);

        /**
         * Set the linear stiffness coefficient (aka m_kLST).
         *
         * @param linearStiffnessFactor the value to set, between [0,1]
         */
        public void setLinearStiffnessFactor(float linearStiffnessFactor) {
            setLinearStiffnessFactor(materialId, linearStiffnessFactor);
        }

        private native void setLinearStiffnessFactor(long materialId, float linearStiffnessFactor);

        /**
         * Get the angular stiffness factor (aka m_kAST).
         *
         * @return the angularStiffnessFactor
         */
        public float getAngularStiffnessFactor() {
            return getAngularStiffnessFactor(materialId);
        }

        private native float getAngularStiffnessFactor(long materialId);

        /**
         * Set the angular stiffness factor (aka m_kAST).
         *
         * @param angularStiffnessFactor the value to set, between [0,1].
         */
        public void setAngularStiffnessFactor(float angularStiffnessFactor) {
            setAngularStiffnessFactor(materialId, angularStiffnessFactor);
        }

        private native void setAngularStiffnessFactor(long materialId, float angularStiffnessFactor);

        /**
         * Get the volume stiffness factor (aka m_kVST).
         *
         * @return the volumeStiffnessFactor
         */
        public float getVolumeStiffnessFactor() {
            return getVolumeStiffnessFactor(materialId);
        }

        private native float getVolumeStiffnessFactor(long materialId);

        /**
         * Set the volume stiffness factor (aka m_kVST).
         *
         * @param volumeStiffnessFactor the value to set, between [0,1].
         */
        public void setVolumeStiffnessFactor(float volumeStiffnessFactor) {
            setVolumeStiffnessFactor(materialId, volumeStiffnessFactor);
        }

        private native void setVolumeStiffnessFactor(long materialId, float volumeStiffnessFactor);

    }
}
