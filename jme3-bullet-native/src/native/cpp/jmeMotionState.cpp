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
#include "jmeMotionState.h"
#include "jmeBulletUtil.h"
#include "jmeDebugCallback.h"

/**
 * Author: Normen Hansen
 */

jmeMotionState::jmeMotionState() {
    trans = new btTransform();
    trans -> setIdentity();
    worldTransform = *trans;
    appliedLinearDamping = 0;
    appliedAngularDamping = 0;
    updateId = 0;
    debugCallback = 0;
    dirty = true;
}

void jmeMotionState::setDebugCallback(jmeDebugCallback * callback) {
    if (callback != debugCallback) {
        if (debugCallback) {
            delete debugCallback;
        }
        debugCallback = callback;
    }
}

void jmeMotionState::getWorldTransform(btTransform& worldTrans) const {
    worldTrans = worldTransform;
}

void jmeMotionState::setWorldTransform(const btTransform& worldTrans) {
    jmeDebugCallback::callbackFromBullet(debugCallback, jmeDebugCallback::RIGID_BODY_MOTION_STATE_SET_WORLD_TRANSFORM, true);
    worldTransform = worldTrans;
    dirty = true;
    jmeDebugCallback::callbackFromBullet(debugCallback, jmeDebugCallback::RIGID_BODY_MOTION_STATE_SET_WORLD_TRANSFORM, false);
}

void jmeMotionState::setKinematicTransform(const btTransform& worldTrans) {
    worldTransform = worldTrans;
    dirty = true;
}

void jmeMotionState::setKinematicLocation(JNIEnv* env, jobject location) {
    jmeBulletUtil::convert(env, location, &worldTransform.getOrigin());
    dirty = true;
}

void jmeMotionState::setKinematicRotation(JNIEnv* env, jobject rotation) {
    jmeBulletUtil::convert(env, rotation, &worldTransform.getBasis());
    dirty = true;
}

void jmeMotionState::setKinematicRotationQuat(JNIEnv* env, jobject rotation) {
    jmeBulletUtil::convertQuat(env, rotation, &worldTransform.getBasis());
    dirty = true;
}

bool jmeMotionState::applyTransform(JNIEnv* env, jobject location, jobject rotation) {
    if (dirty) {
        //        fprintf(stdout, "Apply world translation\n");
        //        fflush(stdout);
        jmeBulletUtil::convert(env, &worldTransform.getOrigin(), location);
        jmeBulletUtil::convertQuat(env, &worldTransform.getBasis(), rotation);
        dirty = false;
        return true;
    }
    return false;
}

jmeMotionState::~jmeMotionState() {
    free(trans);
    setDebugCallback(0);
}

void jmeMotionState::setLinearVelocity(const btVector3 & vel, const btVector3 * cause, btScalar tStep) {
    int reason;
    appliedLinearDamping = 0;
    if (cause) {
        if (tStep > 0) {
            totalForce = *cause;
            reason = jmeDebugCallback::RIGID_BODY_INTEGRATE_LINEAR_VELOCITY;
        }
        else {
            linearImpulse = *cause;
            reason = jmeDebugCallback::RIGID_BODY_APPLY_LINEAR_IMPULSE;
        }
    }
    else {
        if (tStep != 0) {
            appliedLinearDamping = tStep;
            reason = jmeDebugCallback::RIGID_BODY_DAMP_LINEAR_VELOCITY;
        }
        else {
            reason = jmeDebugCallback::RIGID_BODY_SET_LINEAR_VELOCITY;
        }
    }
    jmeDebugCallback::callbackFromBullet(debugCallback, reason, true);
    ++updateId;
    linearVel = vel;
    jmeDebugCallback::callbackFromBullet(debugCallback, reason, false);
}

void jmeMotionState::setAngularVelocity(const btVector3 & vel, const btVector3 * cause, btScalar tStep) {
    int reason;
    appliedAngularDamping = 0;
    if (cause) {
        if (tStep > 0) {
            totalTorque = *cause;
            reason = jmeDebugCallback::RIGID_BODY_INTEGRATE_ANGULAR_VELOCITY;
        }
        else {
            angularImpulse = *cause;
            reason = jmeDebugCallback::RIGID_BODY_APPLY_ANGULAR_IMPULSE;
        }
    }
    else {
        if (tStep != 0) {
            appliedAngularDamping = tStep;
            reason = jmeDebugCallback::RIGID_BODY_DAMP_ANGULAR_VELOCITY;
        }
        else {
            reason = jmeDebugCallback::RIGID_BODY_SET_ANGULAR_VELOCITY;
        }
    }
    jmeDebugCallback::callbackFromBullet(debugCallback, reason, true);
    ++updateId;
    angularVel = vel;
    jmeDebugCallback::callbackFromBullet(debugCallback, reason, false);
}

void jmeMotionState::forcesCleared() {
    jmeDebugCallback::callbackFromBullet(debugCallback, jmeDebugCallback::RIGID_BODY_FORCES_CLEARED, true);

    ++updateId;
    totalForce.setZero();
    totalTorque.setZero();
    appliedLinearDamping = 0;
    appliedAngularDamping = 0;

    jmeDebugCallback::callbackFromBullet(debugCallback, jmeDebugCallback::RIGID_BODY_FORCES_CLEARED, false);
}
