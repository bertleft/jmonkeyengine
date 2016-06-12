/*
 * Copyright (c) 2009-2016 jMonkeyEngine
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

#include "jmeDebugCallback.h"
#include "jmeClasses.h"

/**
 * Author: Albert Santos
 */
jmeDebugCallback::jmeDebugCallback(JNIEnv * env, jobject callbackObject, jobject objectArg)
{
    this->env = env;
    this->callbackObject = env->NewGlobalRef(callbackObject);
    this->objectArg = env->NewGlobalRef(objectArg);
}

jmeDebugCallback::~jmeDebugCallback()
{
    if (env) {
        if (callbackObject) {
            env->DeleteGlobalRef(callbackObject);
        }
        if (objectArg) {
            env->DeleteGlobalRef(objectArg);
        }
    }
}

void jmeDebugCallback::callCallbackFromBullet(int reason, bool isPreAction) const {
    env->CallVoidMethod(callbackObject, jmeClasses::BulletDebugCallback_callbackFromBullet, reason, objectArg, isPreAction);
}

int jmeDebugCallback::RIGID_BODY_MOTION_STATE_SET_WORLD_TRANSFORM       = 0;
int jmeDebugCallback::RIGID_BODY_SET_LINEAR_VELOCITY                    = 1;
int jmeDebugCallback::RIGID_BODY_SET_ANGULAR_VELOCITY                   = 2;
int jmeDebugCallback::RIGID_BODY_APPLY_LINEAR_IMPULSE                   = 3;
int jmeDebugCallback::RIGID_BODY_APPLY_ANGULAR_IMPULSE                  = 4;
int jmeDebugCallback::RIGID_BODY_INTEGRATE_LINEAR_VELOCITY              = 5;
int jmeDebugCallback::RIGID_BODY_INTEGRATE_ANGULAR_VELOCITY             = 6;
int jmeDebugCallback::RIGID_BODY_DAMP_LINEAR_VELOCITY                   = 7;
int jmeDebugCallback::RIGID_BODY_DAMP_ANGULAR_VELOCITY                  = 8;
int jmeDebugCallback::RIGID_BODY_FORCES_CLEARED                         = 9;
