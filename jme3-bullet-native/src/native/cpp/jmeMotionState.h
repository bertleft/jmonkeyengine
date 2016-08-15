/*
 * Copyright (c) 2009-2012 jMonkeyEngine
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
#include <jni.h>

/**
 * Author: Normen Hansen
 */

#include "btBulletDynamicsCommon.h"
//#include "btBulletCollisionCommon.h"

class jmeDebugCallback;

class jmeMotionState : public btMotionState {
private:
    bool dirty;
    btTransform* trans;
    
    btVector3 linearVel;
    btVector3 angularVel;
    btVector3 linearImpulse;
    btVector3 angularImpulse;
    btVector3 totalForce;
    btVector3 totalTorque;
    btScalar appliedLinearDamping;
    btScalar appliedAngularDamping;
    
    int updateId;
    
    jmeDebugCallback * debugCallback;
public:
    jmeMotionState();
    virtual ~jmeMotionState();
    
    void setDebugCallback(jmeDebugCallback * callback);

    btTransform worldTransform;
    virtual void getWorldTransform(btTransform& worldTrans) const;
    virtual void setWorldTransform(const btTransform& worldTrans);
    void setKinematicTransform(const btTransform& worldTrans);
    void setKinematicLocation(JNIEnv*, jobject);
    void setKinematicRotation(JNIEnv*, jobject);
    void setKinematicRotationQuat(JNIEnv*, jobject);
    bool applyTransform(JNIEnv* env, jobject location, jobject rotation);
    
    
    virtual void setLinearVelocity(const btVector3 & vel, const btVector3 * cause, btScalar tStep);
    virtual void setAngularVelocity(const btVector3 & vel, const btVector3 * cause, btScalar tStep);
    virtual void forcesCleared();
    virtual void setTotalForce(const btVector3 & force);
    virtual void setTotalTorque(const btVector3 & torque);

    const btVector3 & getLinearVelocity() const { return linearVel; }
    const btVector3 & getAngularVelocity() const { return angularVel; }
    const btVector3 & getLinearImpulse() const { return linearImpulse; }
    const btVector3 & getAngularImpulse() const { return angularImpulse; }
    const btVector3 & getTotalForce() const { return totalForce; }
    const btVector3 & getTotalTorque() const { return totalTorque; }
    const btScalar getAppliedLinearDamping() const { return appliedLinearDamping; }
    const btScalar getAppliedAngularDamping() const { return appliedAngularDamping; }
    
    int getUpdateId() const { return updateId; }
};
