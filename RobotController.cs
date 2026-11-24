using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;


public class RobotController : MonoBehaviour
{
    // wheel colliders
    [SerializeField] private WheelCollider WheelColliderFrontLeft;
    [SerializeField] private WheelCollider WheelColliderFrontRight;
    [SerializeField] private WheelCollider WheelColliderBackLeft;
    [SerializeField] private WheelCollider WheelColliderBackRight;
    // transforms for the wheels
    [SerializeField] private Transform TransformWheelFrontLeft;
    [SerializeField] private Transform TransformWheelFrontRight;
    [SerializeField] private Transform TransformWheelBackLeft;
    [SerializeField] private Transform TransformWheelBackRight;

    // transforms for the sensors
    [SerializeField] private Transform RaycastSensorFront;
    [SerializeField] private Transform RaycastSensorLeft;
    [SerializeField] private Transform RaycastSensorRight;
    [SerializeField] private Transform EulerAnglesSensor;



    [SerializeField] private GameObject TrackParent;
    [SerializeField] private GameObject ObstacleParent;




    // ----------------- EULER (YAW) SENSOR (stability helper) -----------------
    // We estimate yaw rate (how fast the car is turning) and apply damping so steering is smoother.
    private float previousYaw;
    private float yawRate;

    // yaw damping tuning (small values on purpose)
    public float yawDampingStrength = 0.015f;
    public float yawDampingWeight = 0.6f;

    // -------------------- DRIVING SETTINGS --------------------
    [Header("Driving Settings")]
    public float maxMotorTorque = 500;
    public float maxSteerAngle = 30;
    public float targetSpeed = 7; // km/h





    [Header("Sensor Settings")]
    public float frontRange = 10f;
    public float trackRayLength = 15f;








    public LayerMask trackMask;

    [Header("Obstacle Layer")]
    public LayerMask obstacleMask;

    // internal
    private float currentSpeed;
    private float motorTorque;
    private float steeringAngle;
    private Rigidbody rb;

    // avoidance state
    private bool isAvoiding = false;
    private float avoidTimeout = 1.0f;      // how long to remain in avoidance after last hit (seconds)
    private float avoidTimer = 0f;

    // tuning smoothing
    public float steerLerpSpeed = 1f;
    public float avoidSteerLerpSpeed = 1f;



    public float SideSensorWidth = 45f;



    [Header("Front Obstacle Fan")]
    public int frontRayCount = 5;          // odd number recommended (center ray exists)
    public float frontFanAngle = 20f;      // degrees left/right


    private void Start()
    {


        int roadLayer = LayerMask.NameToLayer("Road");
        SetLayerToImmediateChildren(TrackParent, roadLayer);

        int ObstacleLayer = LayerMask.NameToLayer("Obs");
        SetLayerToImmediateChildren(ObstacleParent, ObstacleLayer);


        previousYaw = transform.eulerAngles.y;
        trackMask = LayerMask.GetMask("Road");
        obstacleMask = LayerMask.GetMask("Obs");


       


        SetSensorRotations();
        rb = GetComponent<Rigidbody>();
     //   rb.mass = 10f;
        ImproveFriction(WheelColliderFrontLeft);
        ImproveFriction(WheelColliderFrontRight);
        ImproveFriction(WheelColliderBackLeft);
        ImproveFriction(WheelColliderBackRight);

        // Optional: if you haven't rotated sensors in editor, set defaults:
        if (RaycastSensorLeft != null && RaycastSensorRight != null)
        {
            // ensure front left/right have outward angle — you can override in inspector
            // Only set if the local rotation appears untouched (approx identity).
            // NOTE: comment these out if you set angles manually in inspector.
            if (Quaternion.Angle(RaycastSensorLeft.localRotation, Quaternion.identity) < 1f)
                RaycastSensorLeft.localRotation = Quaternion.Euler(0f, -25f, 0f);
            if (Quaternion.Angle(RaycastSensorRight.localRotation, Quaternion.identity) < 1f)
                RaycastSensorRight.localRotation = Quaternion.Euler(0f, 25f, 0f);
        }
    }
    void SetLayerToImmediateChildren(GameObject parent, int layer)
    {
        foreach (Transform child in parent.transform)
        {
            child.gameObject.layer = layer;
        }
    }




    void UpdateEulerYawSensor()
    {
        float currentYaw = transform.eulerAngles.y;

        // DeltaAngle handles 0–360 wrap correctly
        float deltaYaw = Mathf.DeltaAngle(previousYaw, currentYaw);

        yawRate = deltaYaw / Time.fixedDeltaTime;

        previousYaw = currentYaw;
    }
    float GetEulerYawDamping()
    {
        // Positive yawRate → turning right
        // Negative yawRate → turning left
        return Mathf.Clamp(yawRate * yawDampingStrength, -1f, 1f);
    }

    public void SetSensorRotations()
    {





        RaycastSensorFront.localRotation = Quaternion.Euler(3, 0, 0);

        RaycastSensorLeft.localRotation = Quaternion.Euler(10, -SideSensorWidth, 0);
        RaycastSensorRight.localRotation = Quaternion.Euler(10, SideSensorWidth, 0);

    }





    void FixedUpdate()
    {
        // -------------------------------------------------
        // 1. SENSOR & PHYSICS STATE UPDATE
        // -------------------------------------------------
        UpdateEulerYawSensor();
        UpdateVelocitySensor();
        SpeedControl();
        SelfRighting();
        ApplyAntiRoll();

        // -------------------------------------------------
        // 2. HIGH PRIORITY: OBSTACLE AVOIDANCE
        //    (this internally performs raycasts)
        // -------------------------------------------------
        bool avoidingNow = HandleObstacleAvoidance();

        // -------------------------------------------------
        // 3. AVOIDANCE COOLDOWN MANAGEMENT
        // -------------------------------------------------
        if (!avoidingNow && avoidTimer > 0f)
        {
            avoidTimer -= Time.fixedDeltaTime;
            if (avoidTimer <= 0f)
            {
                isAvoiding = false;
                avoidTimer = 0f;
            }
        }

        // -------------------------------------------------
        // 4. LOW PRIORITY: TRACK FOLLOW (SIDE SENSORS)
        // -------------------------------------------------
        if (!isAvoiding)
        {
            TrackFollow_UsingFrontSensors();
        }

        // -------------------------------------------------
        // 5. VERY LOW PRIORITY: EULER YAW STABILITY
        // -------------------------------------------------
        if (!isAvoiding)
        {
            float yawDamping = GetEulerYawDamping();

            steeringAngle = Mathf.Lerp(
                steeringAngle,
                steeringAngle - (yawDamping * maxSteerAngle * yawDampingWeight),
                Time.fixedDeltaTime * steerLerpSpeed
            );
        }

        // -------------------------------------------------
        // 6. APPLY TO VEHICLE
        // -------------------------------------------------
        ApplySteering();
        ApplyMotorTorque();
        UpdateWheels();
    }



    // ----------------- SPEED & SLOPE -----------------
    void UpdateVelocitySensor()
    {
        currentSpeed = Mathf.Abs(2 * Mathf.PI * WheelColliderFrontLeft.radius * WheelColliderFrontLeft.rpm * 60 / 1000f);
    }

    void SpeedControl()
    {
        float realSpeed = rb.linearVelocity.magnitude * 3.6f; // km/h

        float forwardSlope = 0f;

        if (Physics.Raycast(transform.position + Vector3.up, Vector3.down,
                             out RaycastHit hit, 3f))
        {
            Vector3 groundNormal = hit.normal;

            // 🔒 Remove sideways slope (banking)
            Vector3 flatForward =
                Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;

            // Normal projected into forward-up plane
            Vector3 slopeNormal =
                Vector3.ProjectOnPlane(groundNormal, Vector3.Cross(flatForward, Vector3.up)).normalized;

            // Signed angle around right axis (WORLD-STABLE)
            forwardSlope = Vector3.SignedAngle(
                slopeNormal,
                Vector3.up,
                Vector3.Cross(Vector3.up, flatForward)
            );
        }

        // 🔧 Tune strength here
        float slopeCompensation = forwardSlope * 10f;

        if (realSpeed < targetSpeed)
        {
            motorTorque = Mathf.Clamp(
                maxMotorTorque + slopeCompensation,
                0f,
                maxMotorTorque
            );
        }
        else
        {
            motorTorque = Mathf.Lerp(motorTorque, 0f, 2f * Time.deltaTime);
        }


    }

    // ----------------- WHEELS -----------------
    void ApplySteering()
    {
        float clamped = Mathf.Clamp(steeringAngle, -maxSteerAngle, maxSteerAngle);
        WheelColliderFrontLeft.steerAngle = clamped;
        WheelColliderFrontRight.steerAngle = clamped;
    }

    void ApplyMotorTorque()
    {
        WheelColliderFrontLeft.motorTorque = motorTorque * 0.7f;
        WheelColliderFrontRight.motorTorque = motorTorque * 0.7f;
        WheelColliderBackLeft.motorTorque = motorTorque;
        WheelColliderBackRight.motorTorque = motorTorque;
    }

    void UpdateWheels()
    {
        UpdateWheelVisual(WheelColliderFrontLeft, TransformWheelFrontLeft);
        UpdateWheelVisual(WheelColliderFrontRight, TransformWheelFrontRight);
        UpdateWheelVisual(WheelColliderBackLeft, TransformWheelBackLeft);
        UpdateWheelVisual(WheelColliderBackRight, TransformWheelBackRight);
    }

    void UpdateWheelVisual(WheelCollider col, Transform mesh)
    {
        col.GetWorldPose(out Vector3 p, out Quaternion r);
        mesh.position = p;
        mesh.rotation = r;
    }

    void ImproveFriction(WheelCollider wheel)
    {
        WheelFrictionCurve f = wheel.forwardFriction;
        f.stiffness = 1.5f;
        wheel.forwardFriction = f;
        WheelFrictionCurve s = wheel.sidewaysFriction;
        s.stiffness = 2.0f;
        wheel.sidewaysFriction = s;
    }



    // ----------------- OBSTACLE AVOIDANCE (FRONT FAN RAYS) -----------------
    bool HandleObstacleAvoidance()
    {
        if (RaycastSensorFront == null)
            return false;

        float steerAccumulator = 0f;
        float weightAccumulator = 0f;
        float closestHit01 = 0f;

        Vector3 origin = RaycastSensorFront.position;
        Vector3 forward = RaycastSensorFront.forward;

        for (int i = 0; i < frontRayCount; i++)
        {
            // t in [-1 .. +1]
            float t = (frontRayCount == 1)
                ? 0f
                : (i / (float)(frontRayCount - 1)) * 2f - 1f;

            float angle = t * frontFanAngle;

            Vector3 dir = Quaternion.AngleAxis(angle, Vector3.up) * forward;

            if (Physics.Raycast(origin, dir, out RaycastHit hit, frontRange, obstacleMask))
            {
                float strength = Mathf.Clamp01((frontRange - hit.distance) / frontRange);

                // Rays on left push right, rays on right push left
                float steerDir = -Mathf.Sign(angle);

                steerAccumulator += steerDir * strength;
                weightAccumulator += strength;

                closestHit01 = Mathf.Max(closestHit01, strength);

                Debug.DrawRay(origin, dir * hit.distance, Color.red);
            }
            else
            {
                Debug.DrawRay(origin, dir * frontRange, Color.yellow);
            }
        }

        // ----------------- NO OBSTACLE -----------------
        if (weightAccumulator <= 0.001f)
            return false;

        // ----------------- STEERING DECISION -----------------
        float steerCommand = (steerAccumulator / weightAccumulator) * maxSteerAngle;

        steeringAngle = Mathf.Lerp(
            steeringAngle,
            steerCommand,
            Time.deltaTime * avoidSteerLerpSpeed
        );

        // ----------------- SPEED REDUCTION -----------------
        motorTorque *= Mathf.Lerp(1f, 0.25f, closestHit01);

        // ----------------- AVOID STATE -----------------
        bool avoided = Mathf.Abs(steerCommand) > 0.01f;

        if (avoided)
        {
            isAvoiding = true;
            avoidTimer = avoidTimeout;
        }

        return avoided;
    }



    bool TrackDetected(Transform sensor, out float distance)
    {
        distance = trackRayLength;

        if (sensor == null)
            return false;

        if (Physics.Raycast(sensor.position, sensor.forward, out RaycastHit hit, trackRayLength, trackMask))
        {
            distance = hit.distance;
            return true;
        }

        return false;
    }



    // ----------------- TRACK FOLLOW (same sensors, track-aware) -----------------
    void TrackFollow_UsingFrontSensors()
    {
        if (isAvoiding) return;

        bool leftOnTrack = TrackDetected(RaycastSensorLeft, out float leftDist);
        bool rightOnTrack = TrackDetected(RaycastSensorRight, out float rightDist);

        // Debug
        Debug.DrawRay(RaycastSensorLeft.position,
            RaycastSensorLeft.forward * trackRayLength,
            leftOnTrack ? Color.green : Color.red);

        Debug.DrawRay(RaycastSensorRight.position,
            RaycastSensorRight.forward * trackRayLength,
            rightOnTrack ? Color.green : Color.red);

        float steerForce = 0f;

        // One side lost track → steer back
        if (leftOnTrack && !rightOnTrack)
            steerForce = -1f;
        else if (rightOnTrack && !leftOnTrack)
            steerForce = 1f;

        // Both on track → balance distances (center seeking)
        else if (leftOnTrack && rightOnTrack)
        {
            float diff = (rightDist - leftDist) / trackRayLength;

            // dead zone to prevent jitter
            if (Mathf.Abs(diff) < 0.05f)
                diff = 0f;

            steerForce = diff;
        }

        steeringAngle = Mathf.Lerp(
            steeringAngle,
            steerForce * maxSteerAngle * 0.6f,
            Time.deltaTime * steerLerpSpeed
        );
        if (!leftOnTrack && !rightOnTrack)
        {
            // emergency: slow down and keep last steering
            motorTorque *= 0.5f;
            return;
        }

    }


    float GetSideSlope()
    {
        if (Physics.Raycast(transform.position + Vector3.up, Vector3.down, out RaycastHit hit, 3f))
        {
            Vector3 groundNormal = hit.normal;

            // Project normal onto plane perpendicular to car's forward direction
            // Removes forward slope
            Vector3 sideOnlyNormal = Vector3.ProjectOnPlane(groundNormal, transform.forward).normalized;

            // Signed angle relative to global up, around the forward axis
            float sideSlope = Vector3.SignedAngle(sideOnlyNormal, Vector3.up, transform.forward);

            // Positive = leaning right, Negative = leaning left
            return sideSlope;
        }

        return 0f;
    }
    void ApplyAntiRoll()
    {
        ApplyWheelAntiRoll(WheelColliderFrontLeft, WheelColliderFrontRight);
        ApplyWheelAntiRoll(WheelColliderBackLeft, WheelColliderBackRight);
    }

    void ApplyWheelAntiRoll(WheelCollider wheelL, WheelCollider wheelR)
    {
        WheelHit hitL, hitR;
        bool groundedL = wheelL.GetGroundHit(out hitL);
        bool groundedR = wheelR.GetGroundHit(out hitR);

        float suspensionDistL = wheelL.suspensionDistance;
        float suspensionDistR = wheelR.suspensionDistance;

        // 🛑 PREVENT DIVISION BY ZERO
        if (suspensionDistL <= 0.001f || suspensionDistR <= 0.001f)
            return;

        float travelL = 1f;
        float travelR = 1f;

        // -------------------------
        // SAFE TRAVEL CALCULATION
        // -------------------------
        if (groundedL)
        {
            float point = -wheelL.transform.InverseTransformPoint(hitL.point).y - wheelL.radius;
            travelL = Mathf.Clamp01(point / suspensionDistL);
        }

        if (groundedR)
        {
            float point = -wheelR.transform.InverseTransformPoint(hitR.point).y - wheelR.radius;
            travelR = Mathf.Clamp01(point / suspensionDistR);
        }

        // -------------------------
        // ANTI-ROLL FORCE
        // -------------------------
        float antiRoll = 6000f;
        float force = (travelL - travelR) * antiRoll;

        // 🛑 PREVENT INFINITY AND NaN
        if (float.IsNaN(force) || float.IsInfinity(force))
            return;

        // APPLY FORCES SAFELY
        if (groundedL)
            rb.AddForceAtPosition(wheelL.transform.up * -force, wheelL.transform.position);

        if (groundedR)
            rb.AddForceAtPosition(wheelR.transform.up * force, wheelR.transform.position);
    }

    void SelfRighting()
    {
        float roll = GetSideSlope();

        if (Mathf.Abs(roll) > 35f) // about to flip
        {
            Quaternion target = Quaternion.Euler(0f, transform.eulerAngles.y, 0f);
            transform.rotation = Quaternion.Slerp(transform.rotation, target, Time.deltaTime * 2f);
        }
    }



}
