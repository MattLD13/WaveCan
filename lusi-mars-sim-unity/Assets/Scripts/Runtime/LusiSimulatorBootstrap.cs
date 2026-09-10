using UnityEngine;

[DisallowMultipleComponent]
public sealed class LusiSimulatorBootstrap : MonoBehaviour
{
    [SerializeField] private Shader runtimeRegolithShader;

    public static LusiSimulatorBootstrap Instance { get; private set; }
    public string ActiveProfile => SimulationProfile.ActiveProfileName;
    public MarsWorldBuilder World { get; private set; }
    public RoverVisual Rover { get; private set; }
    public CameraRig CameraRig { get; private set; }
    public MissionDirector Mission { get; private set; }
    public SteamDeckInput Input { get; private set; }
    public CockpitUI Cockpit { get; private set; }

    private bool initialized;

    private void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
            return;
        }

        Instance = this;
    }

    private void Start()
    {
        if (initialized || Instance != this)
        {
            return;
        }

        initialized = true;
        SimulationProfile.Apply(SimulatorQuality.DeckPerformance);
        if (!Application.isEditor)
        {
            SimulationProfile.ApplyReferenceResolution();
        }

        var runtimeRoot = new GameObject("LUSI Simulation Runtime");
        runtimeRoot.transform.SetParent(transform, false);

        var worldObject = new GameObject("Mars World");
        worldObject.transform.SetParent(runtimeRoot.transform, false);
        World = worldObject.AddComponent<MarsWorldBuilder>();
        World.RuntimeRegolithShader = runtimeRegolithShader;
        World.Build();

        var roverObject = new GameObject("LUSI Rover / SIM ONLY");
        roverObject.transform.SetParent(runtimeRoot.transform, false);
        roverObject.transform.position = World.BaseMarker.position + Vector3.up * 0.12f;
        Rover = roverObject.AddComponent<RoverVisual>();
        Rover.Build(World.RuntimeRegolithShader);
        Rover.SnapToSurface();

        var rigObject = new GameObject("FPV Camera Rig");
        rigObject.transform.SetParent(runtimeRoot.transform, false);
        rigObject.transform.position = Rover.transform.position;
        CameraRig = rigObject.AddComponent<CameraRig>();
        CameraRig.Bind(Rover.transform);

        var missionObject = new GameObject("Safe Mission State");
        missionObject.transform.SetParent(runtimeRoot.transform, false);
        Mission = missionObject.AddComponent<MissionDirector>();
        Mission.Bind(World, Rover);

        var inputObject = new GameObject("Steam Deck Input");
        inputObject.transform.SetParent(runtimeRoot.transform, false);
        Input = inputObject.AddComponent<SteamDeckInput>();
        Input.Bind(Rover, CameraRig, Mission);

        var uiObject = new GameObject("Cockpit UI");
        uiObject.transform.SetParent(runtimeRoot.transform, false);
        Cockpit = uiObject.AddComponent<CockpitUI>();
        Cockpit.Build(Mission, CameraRig, Rover);
    }

    private void OnDestroy()
    {
        if (Instance == this)
        {
            Instance = null;
        }
    }
}
