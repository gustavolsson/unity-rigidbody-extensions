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
