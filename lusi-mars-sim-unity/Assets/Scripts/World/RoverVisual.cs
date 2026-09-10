using UnityEngine;

public sealed class RoverVisual : MonoBehaviour
{
    public float CurrentSpeed { get; private set; }
    public Vector2 LastCommand { get; private set; }
    public bool MotionAccepted { get; private set; }
    public string SurfaceContactName { get; private set; } = "Terrain";

    private Material bodyMaterial;
    private float steering;
    private Vector2 commandedDrive;

    public void Build(Shader runtimeSafeShader = null)
    {
        bodyMaterial = CreateMaterial(runtimeSafeShader, new Color(0.64f, 0.68f, 0.66f, 1f));
        CreatePart("Rover chassis", PrimitiveType.Cube, new Vector3(0f, 0.78f, 0f), new Vector3(2.7f, 0.48f, 1.9f), bodyMaterial);
        CreatePart("Rover equipment deck", PrimitiveType.Cube, new Vector3(0.05f, 1.12f, 0f), new Vector3(1.7f, 0.16f, 1.35f), bodyMaterial);
        CreatePart("Rover mast", PrimitiveType.Cylinder, new Vector3(0.76f, 1.65f, 0f), new Vector3(0.15f, 0.7f, 0.15f), bodyMaterial);
        CreatePart("FPV camera housing", PrimitiveType.Cube, new Vector3(0.8f, 2.12f, 0f), new Vector3(0.45f, 0.26f, 0.6f), bodyMaterial);

        var wheelMaterial = CreateMaterial(runtimeSafeShader, new Color(0.055f, 0.048f, 0.043f, 1f));
        var wheelPositions = new[]
        {
            new Vector3(-0.85f, 0.43f, -1.08f), new Vector3(0f, 0.43f, -1.12f), new Vector3(0.85f, 0.43f, -1.08f),
            new Vector3(-0.85f, 0.43f, 1.08f), new Vector3(0f, 0.43f, 1.12f), new Vector3(0.85f, 0.43f, 1.08f)
        };
        for (var i = 0; i < wheelPositions.Length; i++)
        {
            CreatePart("Rover wheel " + i, PrimitiveType.Cylinder, wheelPositions[i], new Vector3(0.52f, 0.18f, 0.52f), wheelMaterial).transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
        }

        var mastLight = new GameObject("Rover simulated camera indicator");
        mastLight.transform.SetParent(transform);
        mastLight.transform.localPosition = new Vector3(1.03f, 2.18f, 0f);
        var light = mastLight.AddComponent<Light>();
        light.type = LightType.Point;
        light.color = new Color(0.25f, 0.85f, 1f);
        light.intensity = 1.6f;
        light.range = 3.5f;
    }

    public void SetCommand(Vector2 drive, bool deadman)
    {
        LastCommand = drive;
        MotionAccepted = deadman;
        commandedDrive = deadman ? Vector2.ClampMagnitude(drive, 1f) : Vector2.zero;
    }

    public void ResetCommand()
    {
        SetCommand(Vector2.zero, false);
        CurrentSpeed = 0f;
    }

    public void SnapToSurface()
    {
        if (TryFindSurface(transform.position, out var hit))
        {
            transform.position = new Vector3(transform.position.x, hit.point.y + 0.02f, transform.position.z);
            SurfaceContactName = hit.collider.gameObject.name;
        }
    }

    private void Update()
    {
        var dt = Time.deltaTime;
        var throttle = commandedDrive.y;
        steering = Mathf.MoveTowards(steering, commandedDrive.x, dt * 4f);
        var targetSpeed = throttle * 3.4f;
        CurrentSpeed = Mathf.MoveTowards(CurrentSpeed, targetSpeed, dt * 5.5f);
        transform.Rotate(Vector3.up, steering * CurrentSpeed * 7.5f * dt, Space.World);
        var proposedPosition = transform.position + transform.forward * CurrentSpeed * dt;
        if (TryFindSurface(proposedPosition, out var hit))
        {
            proposedPosition.y = hit.point.y + 0.02f;
            SurfaceContactName = hit.collider.gameObject.name;

            var surfaceForward = Vector3.ProjectOnPlane(transform.forward, hit.normal).normalized;
            if (surfaceForward.sqrMagnitude > 0.01f)
            {
                var targetRotation = Quaternion.LookRotation(surfaceForward, hit.normal);
                transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Mathf.Clamp01(dt * 10f));
            }
        }
        else
        {
            proposedPosition.y = SampleGroundHeight(proposedPosition) + 0.02f;
            SurfaceContactName = "Terrain fallback";
        }

        transform.position = proposedPosition;
    }

    private static bool TryFindSurface(Vector3 position, out RaycastHit hit)
    {
        var origin = position + Vector3.up * 12f;
        return Physics.Raycast(origin, Vector3.down, out hit, 30f, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore);
    }

    private static float SampleGroundHeight(Vector3 position)
    {
        return 0.1f + Mathf.PerlinNoise(position.x * 0.025f + 0.7f, position.z * 0.025f + 1.4f) * 0.18f;
    }

    private GameObject CreatePart(string partName, PrimitiveType type, Vector3 localPosition, Vector3 localScale, Material material)
    {
        var part = GameObject.CreatePrimitive(type);
        part.name = partName;
        part.transform.SetParent(transform);
        part.transform.localPosition = localPosition;
        part.transform.localScale = localScale;
        part.GetComponent<Renderer>().sharedMaterial = material;
        Destroy(part.GetComponent<Collider>());
        return part;
    }

    private static Material CreateMaterial(Shader runtimeSafeShader, Color color)
    {
        var shader = runtimeSafeShader ?? Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
        if (shader == null)
        {
            Debug.LogError("LUSI rover could not find a runtime-safe shader.");
            return null;
        }

        var material = new Material(shader);
        if (shader.name == "LUSI/MarsRegolith")
        {
            material.SetColor("_BaseColor", color);
        }
        else
        {
            material.color = color;
        }

        return material;
    }
}
