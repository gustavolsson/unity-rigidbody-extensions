// MIT License
//
// Copyright (c) 2020 Gustav Olsson
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using System.Collections.Generic;
using System.Linq;

namespace RopeMinikit
{
    public static class RigidbodyExtensions
    {
        public static void GetLocalInertiaTensor(this Rigidbody rb, out float3x3 localInertiaTensor)
        {
            localInertiaTensor = math.mul(new float3x3(rb.inertiaTensorRotation), float3x3.Scale(rb.inertiaTensor));
        }

        public static void GetInertiaTensor(this Rigidbody rb, out float3x3 inertiaTensor)
        {
            var rbRotation = new float3x3(rb.transform.rotation);
            var rbRotationInv = new float3x3(math.inverse(rb.transform.rotation));

            rb.GetLocalInertiaTensor(out float3x3 localInertiaTensor);

            inertiaTensor = math.mul(math.mul(rbRotation, localInertiaTensor), rbRotationInv);
        }

        public static void ApplyImpulseNow(this Rigidbody rb, ref float3x3 inverseInertiaTensor, float3 point, float3 impulse)
        {
            if (rb.mass == 0.0f)
            {
                return;
            }

            var relativePoint = point - (float3)rb.worldCenterOfMass;
            var angularMomentumChange = math.cross(relativePoint, impulse);
            var angularVelocityChange = math.mul(inverseInertiaTensor, angularMomentumChange);

            rb.velocity += (Vector3)impulse / rb.mass;
            rb.angularVelocity += (Vector3)angularVelocityChange;
        }

        public static void ApplyImpulseNow(this Rigidbody rb, float3 point, float3 impulse)
        {
            rb.GetInertiaTensor(out float3x3 inertiaTensor);
            var invInertiaTensor = math.inverse(inertiaTensor);
            rb.ApplyImpulseNow(ref invInertiaTensor, point, impulse);
        }

        public static void SetPointVelocityNow(this Rigidbody rb, ref float3x3 inverseInertiaTensor, float3 point, float3 normal, float desiredSpeed, float damping = 1.0f)
        {
            if (rb.mass == 0.0f)
            {
                return;
            }

            var velocityChange = desiredSpeed - math.dot(rb.GetPointVelocity(point), normal) * damping;
            var relativePoint = point - (float3)rb.worldCenterOfMass;

            var denominator = (1.0f / rb.mass) + math.dot(math.cross(math.mul(inverseInertiaTensor, math.cross(relativePoint, normal)), relativePoint), normal);
            if (denominator == 0.0f)
            {
                return;
            }

            var j = velocityChange / denominator;
            rb.ApplyImpulseNow(ref inverseInertiaTensor, point, j * normal);
        }

        public static void SetPointVelocityNow(this Rigidbody rb, float3 point, float3 normal, float desiredSpeed, float damping = 1.0f)
        {
            rb.GetInertiaTensor(out float3x3 inertiaTensor);
            var invInertiaTensor = math.inverse(inertiaTensor);
            rb.SetPointVelocityNow(ref invInertiaTensor, point, normal, desiredSpeed, damping);
        }
    }
}
