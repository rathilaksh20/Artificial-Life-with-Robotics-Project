using System;
using UnityEngine;

public class RobotController : MonoBehaviour
{
    [Header("Wheel Colliders")]
    [SerializeField] private WheelCollider frontLeftWheelCollider;
    [SerializeField] private WheelCollider frontRightWheelCollider;
    [SerializeField] private WheelCollider rearLeftWheelCollider;
    [SerializeField] private WheelCollider rearRightWheelCollider;

    [Header("Wheel Transforms")]
    [SerializeField] private Transform frontLeftWheelTransform;
    [SerializeField] private Transform frontRightWheelTransform;
    [SerializeField] private Transform rearLeftWheelTransform;
    [SerializeField] private Transform rearRightWheelTransform;

    [Header("Sensors")]
    [SerializeField] private Transform RaycastSensorFront;
    [SerializeField] private Transform RaycastSensorLeft;
    [SerializeField] private Transform RaycastSensorRight;
    [SerializeField] private Transform EulerAnglesSensor;

    [Header("Driving")]
    [SerializeField] private float carForce = 50f;
    [SerializeField] private float maxSpeed = 7f;

    [Header("Parameters of rays: 3 rays per sensor")]
    [SerializeField] private float fronttheta = 12f;
    [SerializeField] private float sidethetaspread = 10f;
    [SerializeField] private float spreaddeg = 22f;
    [SerializeField] private float forwardOffset = 1.2f;
    [SerializeField] private float downOffset = 0.35f;
    [SerializeField] private bool castingRays = true;
    [SerializeField] private float downhillBrakeStrength = 800f;

    [Header("Classification")]
    [SerializeField] private string Parentroad = "Track";
    [SerializeField] private string[] RoadName = { "MT_Road", "MT_Turn", "Track" };
    [SerializeField] private string[] NameGrass = { "Ground", "Grass" };
    [SerializeField] private string[] NameKerb = { "Kerb", "Curb", "Border", "RedWhite" };
    [SerializeField] private string[] NameObstacle = { "Obstacle", "Block", "Cube", "Box" };

    [Header("PID Brake Control")]
    [SerializeField] private float hazardSpeedMultiplier = 0.5f;
    [SerializeField] private float Kpconstant = 450f;
    [SerializeField] private float Kiconstant = 40f;
    [SerializeField] private float Kdconstant = 120f;
    [SerializeField] private float integralconstant = 3f;

    private Rigidbody rb;
    private float brakeIntegral = 0f;
    private float brakingPError = 0f;
    private float steerc = 0f;
    private float motorCurrent = 0f;
    private bool isBreaking = false;
    private float lastOT = -999f;
    private enum HitType { None, Road, Grass, Kerb, Obstacle }
    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        motorCurrent = carForce;
        if (RaycastSensorFront) RaycastSensorFront.localRotation = Quaternion.Euler(15f, 0f, 0f);
        if (RaycastSensorLeft) RaycastSensorLeft.localRotation = Quaternion.Euler(15f, -35f, 0f);
        if (RaycastSensorRight) RaycastSensorRight.localRotation = Quaternion.Euler(15f, +35f, 0f);
        if (EulerAnglesSensor) EulerAnglesSensor.localRotation = Quaternion.Euler(0f, 180f, 0f);
    }
    //Casting rays from sensors
    private void CastFan3Rays(Transform sensor, float dist, bool hazard)
    {
        if (!sensor) return;
        Vector3 origin = sensor.position + sensor.up * 0.05f;
        Vector3 baseDir = (sensor.forward * 50f + Vector3.down * downOffset).normalized;
        float[] y = { -spreaddeg, 0f, +spreaddeg };
        foreach (float x in y)
        {
            Vector3 dir = (Quaternion.AngleAxis(x, Vector3.up) * baseDir).normalized;
            Color c = Color.cyan;
            float drawLen = dist;
            if (Physics.Raycast(origin, dir, out RaycastHit hit, dist, ~0, QueryTriggerInteraction.Ignore))
            {
                drawLen = hit.distance;
                HitType t = Classify(hit.collider);

                // if anything except gray track road i.e Hazard, turn color to red
                if (hazard) c = Color.red;
                else c = (t == HitType.Road) ? Color.cyan : Color.red;
            }
            else
            {
                c = hazard ? Color.red : Color.red;
            }
            if (castingRays)
                Debug.DrawRay(origin, dir * drawLen, c);
        }
    }

    private void FixedUpdate()
    {
        if (!rb || !RaycastSensorFront || !RaycastSensorLeft || !RaycastSensorRight)
            return;
        float speedofcar = rb.linearVelocity.magnitude;
        int LR, RR, FR;
        bool leftBad, rightBad;
        bool obstacleFront;
        EvaluateSensors(out LR, out RR, out FR, out leftBad, out rightBad, out obstacleFront);
        float targetMotor = carForce;
        bool downhillAssist = false;
        bool onSlope = Slope(out float signedSlope);
        if (onSlope)
        {
            if (signedSlope < 0f)
            {
                targetMotor = Mathf.Min(carForce + 600f, 2000f);
                isBreaking = false;
                if (rb.linearVelocity.magnitude < 1f)
                    rb.AddForce(transform.forward * 2500f, ForceMode.Force);
            }
            else
            {
                targetMotor = carForce * 0.6f;
                if (rb.linearVelocity.magnitude > (maxSpeed + 0.5f))
                    downhillAssist = true;
            }
        }
        bool turningSoon2 = (FR == 0);

        // This function is written slows down Existing motor and trigger stays same
        if (obstacleFront || turningSoon2)
            targetMotor = 150f;
        if (speedofcar > maxSpeed)
        {
            targetMotor = 0f;
            isBreaking = true;
        }
        else isBreaking = false;

        //Steering Control
        float desiredSteer = 0f;
        if (leftBad && !rightBad) desiredSteer = +1f;
        else if (rightBad && !leftBad) desiredSteer = -1f;
        else
            desiredSteer = Mathf.Clamp((RR - LR) / 3f, -1f, 1f) * 0.6f;

        if (turningSoon2)
        {
            if (LR > RR) desiredSteer = -1f;
            else if (RR > LR) desiredSteer = +1f;
        }

        if (obstacleFront)
        {
            lastOT = Time.time;

            float obstacleDir = 0f;
            if (RR > LR) obstacleDir = +1f;
            else if (LR > RR) obstacleDir = -1f;
            else obstacleDir = Mathf.Sign(desiredSteer);

            desiredSteer = Mathf.Clamp(desiredSteer + obstacleDir * 1f, -1f, 1f);

            if (leftBad && desiredSteer < 0f) desiredSteer = 0f;
            if (rightBad && desiredSteer > 0f) desiredSteer = 0f;
        }
        else
        {
            float since = Time.time - lastOT;
            if (since < 0.9f)
                desiredSteer = Mathf.Lerp(desiredSteer, 0f, 0.35f);
        }

        // This function is written to introduce Speed authority
        float allowedSpeed = 4f;
        if (onSlope && signedSlope < 0f) allowedSpeed *= 1.15f;
        float speedError = allowedSpeed - speedofcar;
        targetMotor += speedError * 120f;
        bool overspeed = speedofcar > (allowedSpeed + 0.3f);
        if (overspeed)
        {
            targetMotor = Mathf.Min(targetMotor, carForce * 0.25f);
            downhillAssist = true;
        }
        ////This function is written to apply PID Control applied for braking
        bool hazard = turningSoon2 || obstacleFront;
        float hazardTargetSpeed = allowedSpeed * hazardSpeedMultiplier;
        float brakeTorquePID = 0f;
        if (hazard)
        {
            float dt = Time.fixedDeltaTime;
            float e = speedofcar - hazardTargetSpeed;
            if (e > 0f)
            {
                brakeIntegral += e * dt;
                brakeIntegral = Mathf.Clamp(brakeIntegral, -integralconstant, integralconstant);
                float deriv = (e - brakingPError) / Mathf.Max(1e-5f, dt);
                brakingPError = e;
                float u = (Kpconstant * e) + (Kiconstant * brakeIntegral) + (Kdconstant * deriv);
                brakeTorquePID = Mathf.Clamp(u, 0f, downhillBrakeStrength);
            }
            else
            {
                // if not overspeeding, release brakes and slowly unwind integral
                brakeIntegral = Mathf.Lerp(brakeIntegral, 0f, 0.2f);
                brakingPError = e;
                brakeTorquePID = 0f;
            }
        }
        else
        {
            // reset PID when no hazard so it the car regains it ideal speed 
            brakeIntegral = Mathf.Lerp(brakeIntegral, 0f, 0.25f);
            brakingPError = 0f;
        }

        if (castingRays)
        {
            CastFan3Rays(RaycastSensorFront, 20f, hazard);
            CastFan3Rays(RaycastSensorLeft, 20f, hazard);
            CastFan3Rays(RaycastSensorRight, 20f, hazard);
        }

        steerc = Mathf.Lerp(steerc, desiredSteer, Time.fixedDeltaTime * 8f);
        motorCurrent = Mathf.Lerp(motorCurrent, targetMotor, Time.fixedDeltaTime * 1.5f);
        HandleSteering(steerc);
        HandleMotor(motorCurrent);

        float finalBrake = 0f;
        if (isBreaking) finalBrake = downhillBrakeStrength;
        else if (hazard) finalBrake = brakeTorquePID;
        else if (downhillAssist) finalBrake = Mathf.Clamp(200f, 0f, downhillBrakeStrength);
        ApplyingBrakes(finalBrake);
        UpdateWheels();
    }
    //This function is written to cast rays from sensors
    private void EvaluateSensors(out int LR, out int RR, out int FR, out bool leftBad, out bool rightBad, out bool obstacleFront)
    {
        LR = RR = FR = 0;
        leftBad = rightBad = false;
        obstacleFront = false;
        Ray3(RaycastSensorFront, -fronttheta, 0f, +fronttheta, 20f, out int fRoad, out bool fBad, out bool fObs, 8f);
        FR = fRoad;
        obstacleFront = fObs;
        Ray3(RaycastSensorLeft, -sidethetaspread, 0f, +sidethetaspread, 20f, out int lRoad, out bool lBadTmp, out bool lObs, 8f);
        LR = lRoad;
        leftBad = lBadTmp;
        Ray3(RaycastSensorRight, -sidethetaspread, 0f, +sidethetaspread, 20f, out int rRoad, out bool rBadTmp, out bool rObs, 8f);
        RR = rRoad;
        rightBad = rBadTmp;
    }
    ////This function is written to control rays casted from sensor
    private void Ray3(Transform sensor, float yawA, float yawB, float yawC, float roadDist, out int roadCount, out bool badEdge, out bool obstacleHit, float obstacleDist)
    {
        roadCount = 0;
        badEdge = false;
        obstacleHit = false;
        Vector3 baseDir = sensor.forward;
        CastAndClassify(sensor.position, ApplyYaw(baseDir, yawA), roadDist, obstacleDist, ref roadCount, ref badEdge, ref obstacleHit);
        CastAndClassify(sensor.position, ApplyYaw(baseDir, yawB), roadDist, obstacleDist, ref roadCount, ref badEdge, ref obstacleHit);
        CastAndClassify(sensor.position, ApplyYaw(baseDir, yawC), roadDist, obstacleDist, ref roadCount, ref badEdge, ref obstacleHit);
    }
    private Vector3 ApplyYaw(Vector3 dir, float yawDeg)
    {
        return (Quaternion.AngleAxis(yawDeg, Vector3.up) * dir).normalized;
    }
    //This function is written to classify the layers like grass, obstacle, kerb, gray road
    private void CastAndClassify(Vector3 origin, Vector3 dir, float distRoad, float distObstacle, ref int roadCount, ref bool badEdge, ref bool obstacleHit)
    {
        if (Physics.Raycast(origin, dir, out RaycastHit hit, distRoad, ~0, QueryTriggerInteraction.Ignore))
        {
            HitType t = Classify(hit.collider);
            if (t == HitType.Road) roadCount++;
            else if (t == HitType.Grass || t == HitType.Kerb)
            {
                if (hit.distance < distRoad * 0.85f) badEdge = true;
            }
            else if (t == HitType.Obstacle)
            {
                if (hit.distance <= distObstacle) obstacleHit = true;
            }
        }
        else
        {
            badEdge = true;
        }
    }
    //This function is written to classify the layers like grass, obstacle, kerb, gray road
    private HitType Classify(Collider col)
    {
        if (!col) return HitType.None;
        GameObject go = col.gameObject;
        if (ContainsAny(go.name, NameObstacle)) return HitType.Obstacle;
        if (ContainsAny(go.name, NameGrass)) return HitType.Grass;
        if (IsUnderParentNamed(go.transform, Parentroad) || ContainsAny(go.name, RoadName)) return HitType.Road;
        if (ContainsAny(go.name, NameKerb)) return HitType.Kerb;
        Renderer r = go.GetComponent<Renderer>();
        if (r && r.sharedMaterial && ContainsAny(r.sharedMaterial.name, NameKerb)) return HitType.Kerb;
        return HitType.None;
    }
    private bool IsUnderParentNamed(Transform t, string parentName)
    {
        if (string.IsNullOrWhiteSpace(parentName)) return false;
        Transform p = t;
        while (p != null)
        {
            if (string.Equals(p.name, parentName, StringComparison.OrdinalIgnoreCase))
                return true;
            p = p.parent;
        }
        return false;
    }
    private bool ContainsAny(string text, string[] needles)
    {
        if (string.IsNullOrEmpty(text) || needles == null) return false;
        for (int i = 0; i < needles.Length; i++)
        {
            if (!string.IsNullOrEmpty(needles[i]) &&
                text.IndexOf(needles[i], StringComparison.OrdinalIgnoreCase) >= 0)
                return true;
        }
        return false;
    }
    //This function is written to handle the car 
    private void HandleSteering(float direction01)
    {
        float steerAngle = 20f * Mathf.Clamp(direction01, -1f, 1f);
        frontLeftWheelCollider.steerAngle = steerAngle;
        frontRightWheelCollider.steerAngle = steerAngle;
    }
    //This function is written to handle motor's torque
    private void HandleMotor(float motor)
    {
        frontLeftWheelCollider.motorTorque = motor;
        frontRightWheelCollider.motorTorque = motor;
        rearLeftWheelCollider.motorTorque = motor;
        rearRightWheelCollider.motorTorque = motor;
    }
    //This function is written for applying PID brakes with a torque 
    private void ApplyingBrakes(float brakeTorque)
    {
        float c = Mathf.Clamp(brakeTorque, 0f, downhillBrakeStrength);
        frontLeftWheelCollider.brakeTorque = c;
        frontRightWheelCollider.brakeTorque = c;
        rearLeftWheelCollider.brakeTorque = c;
        rearRightWheelCollider.brakeTorque = c;
    }
    //This function is written to update wheel collider position
    private void UpdateWheels()
    {
        UpdateWheelP(frontLeftWheelCollider, frontLeftWheelTransform);
        UpdateWheelP(frontRightWheelCollider, frontRightWheelTransform);
        UpdateWheelP(rearLeftWheelCollider, rearLeftWheelTransform);
        UpdateWheelP(rearRightWheelCollider, rearRightWheelTransform);
    }
    private void UpdateWheelP(WheelCollider wheelCollider, Transform trans)
    {
        wheelCollider.GetWorldPose(out Vector3 pos, out Quaternion rot);
        trans.rotation = rot;
        trans.position = pos;
    }
    //This function helps the car to move up and down the slope
    private bool Slope(out float signedSlopeDeg)
    {
        signedSlopeDeg = 0f;
        if (!RaycastSensorFront) return false;
        float[] z = { -spreaddeg, 0f, +spreaddeg };
        Vector3 origin = RaycastSensorFront.position + RaycastSensorFront.forward * 1.2f + RaycastSensorFront.up * 0.1f;
        float forwardWeight = 1.8f;
        float downWeight = 0.35f;
        Vector3 baseDir = (RaycastSensorFront.forward * forwardWeight + Vector3.down * downWeight).normalized;
        Vector3 normalSum = Vector3.zero;
        int roadHits = 0;
        foreach (float a in z)
        {
            Vector3 dir = (Quaternion.AngleAxis(a, Vector3.up) * baseDir).normalized;
            if (Physics.Raycast(origin, dir, out RaycastHit hit, 25f, ~0, QueryTriggerInteraction.Ignore))
            {
                if (Classify(hit.collider) == HitType.Road)
                {
                    normalSum += hit.normal;
                    roadHits++;
                }
            }
        }
        if (roadHits == 0) return false;
        Vector3 avgNormal = (normalSum / roadHits).normalized;
        float slopeDeg = Vector3.Angle(avgNormal, Vector3.up);
        if (slopeDeg < 3f) return false;
        Vector3 downhillDir = Vector3.ProjectOnPlane(Vector3.down, avgNormal).normalized;
        float sign = Mathf.Sign(Vector3.Dot(transform.forward, downhillDir));
        signedSlopeDeg = slopeDeg * sign;
        return true;
    }
}

