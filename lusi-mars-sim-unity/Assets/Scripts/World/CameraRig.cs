using UnityEngine;

public sealed class CameraRig : MonoBehaviour
{
    public Camera ActiveCamera { get; private set; }
    public int CameraIndex { get; private set; }
    public bool MinimapDetailed { get; private set; }

    private Transform rover;
    private Vector2 lookInput;
    private float yaw;
    private float pitch = 13f;

    public void Bind(Transform roverTransform)
    {
        rover = roverTransform;
        var cameraObject = new GameObject("FPV Camera");
        cameraObject.transform.SetParent(transform, false);
        ActiveCamera = cameraObject.AddComponent<Camera>();
        ActiveCamera.name = "FPV Camera";
        ActiveCamera.fieldOfView = 68f;
        ActiveCamera.nearClipPlane = 0.05f;
        ActiveCamera.farClipPlane = 1200f;
        ActiveCamera.clearFlags = CameraClearFlags.Skybox;
        ActiveCamera.backgroundColor = new Color(0.34f, 0.24f, 0.22f, 1f);
        ActiveCamera.allowHDR = false;
        ActiveCamera.allowMSAA = false;
        // Mount the camera just above and in front of the housing. The previous
        // position was inside the housing volume, so the feed clipped through it.
        ActiveCamera.transform.localPosition = new Vector3(0.80f, 2.30f, 0.34f);
        ActiveCamera.transform.localRotation = Quaternion.Euler(7f, 0f, 0f);
        CameraIndex = 0;
        ApplyCameraMode();
        CreateStarfield();
    }

    private void CreateStarfield()
    {
        var starfield = new GameObject("Mars stars (SIM)");
        starfield.transform.SetParent(transform, false);
        var particles = starfield.AddComponent<ParticleSystem>();
        var main = particles.main;
        main.loop = false;
        main.playOnAwake = false;
        main.duration = 99999f;
        main.maxParticles = 150;
        main.startLifetime = 99999f;
        main.startSpeed = 0f;
        main.startSize = 1.15f;
        main.simulationSpace = ParticleSystemSimulationSpace.Local;
        main.startColor = Color.white;
        var emission = particles.emission;
        emission.enabled = false;
        var particleRenderer = starfield.GetComponent<ParticleSystemRenderer>();
        particleRenderer.renderMode = ParticleSystemRenderMode.Billboard;
        // Use the URP Lit shader already used by the terrain and rocks so the
        // standalone player cannot strip the star material.
        var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
        var material = new Material(shader) { name = "SIM Starfield Material" };
        material.SetColor("_BaseColor", Color.white);
        material.SetColor("_Color", Color.white);
        material.SetFloat("_Metallic", 0f);
        material.SetFloat("_Smoothness", 0f);
        material.EnableKeyword("_EMISSION");
        material.SetColor("_EmissionColor", Color.white * 2.5f);
        particleRenderer.sharedMaterial = material;

        var starParticles = new ParticleSystem.Particle[150];
        var random = new System.Random(8901);
        for (var i = 0; i < starParticles.Length; i++)
        {
            var azimuth = (float)random.NextDouble() * Mathf.PI * 2f;
            var elevation = 6f + (float)random.NextDouble() * 80f;
            var elevationRadians = elevation * Mathf.Deg2Rad;
            var horizontal = Mathf.Cos(elevationRadians);
            var direction = new Vector3(Mathf.Sin(azimuth) * horizontal, Mathf.Sin(elevationRadians), Mathf.Cos(azimuth) * horizontal);
            starParticles[i].position = direction * 600f;
            starParticles[i].startColor = Color.Lerp(new Color(0.62f, 0.70f, 0.78f, 1f), Color.white, (float)random.NextDouble());
            starParticles[i].startSize = 0.75f + (float)random.NextDouble() * 1.35f;
            starParticles[i].startLifetime = 99999f;
            starParticles[i].remainingLifetime = 99999f;
        }

        particles.SetParticles(starParticles, starParticles.Length);
        particles.Play();
    }

    public void SetLook(Vector2 input)
    {
        lookInput = Vector2.ClampMagnitude(input, 1f);
    }

    public void CycleCamera()
    {
        CameraIndex = (CameraIndex + 1) % 4;
        ApplyCameraMode();
    }

    public void ToggleMinimapDetail()
    {
        MinimapDetailed = !MinimapDetailed;
    }

    private void LateUpdate()
    {
        if (rover == null || ActiveCamera == null)
        {
            return;
        }

        var cameraTransform = ActiveCamera.transform;
        transform.rotation = Quaternion.Slerp(transform.rotation, Quaternion.Euler(0f, rover.eulerAngles.y, 0f), Time.deltaTime * 6f);
        if (CameraIndex == 0)
        {
            yaw += lookInput.x * Time.deltaTime * 55f;
            pitch = Mathf.Clamp(pitch - lookInput.y * Time.deltaTime * 40f, -16f, 28f);
            cameraTransform.localRotation = Quaternion.Euler(pitch, yaw, 0f);
        }

        transform.position = Vector3.Lerp(transform.position, rover.position, Time.deltaTime * 8f);
        lookInput = Vector2.zero;
    }

    private void ApplyCameraMode()
    {
        if (ActiveCamera == null || rover == null)
        {
            return;
        }

        switch (CameraIndex)
        {
            case 0:
                ActiveCamera.transform.localPosition = new Vector3(0.80f, 2.30f, 0.34f);
                ActiveCamera.fieldOfView = 68f;
                break;
            case 1:
                ActiveCamera.transform.localPosition = new Vector3(-0.15f, 1.35f, -3.6f);
                ActiveCamera.transform.localRotation = Quaternion.Euler(10f, 0f, 0f);
                ActiveCamera.fieldOfView = 72f;
                break;
            case 2:
                ActiveCamera.transform.localPosition = new Vector3(0f, 3.8f, 0.1f);
                ActiveCamera.transform.localRotation = Quaternion.Euler(78f, 180f, 0f);
                ActiveCamera.fieldOfView = 62f;
                break;
            default:
                ActiveCamera.transform.localPosition = new Vector3(0.9f, 2.4f, 0.2f);
                ActiveCamera.transform.localRotation = Quaternion.Euler(8f, 225f, 0f);
                ActiveCamera.fieldOfView = 64f;
                break;
        }
    }
}
