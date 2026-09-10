using UnityEngine;
using UnityEngine.UI;

public sealed class CockpitUI : MonoBehaviour
{
    public Canvas Canvas { get; private set; }

    private MissionDirector mission;
    private CameraRig cameraRig;
    private RoverVisual rover;
    private Text profileText;
    private Text speedText;
    private Text missionText;
    private Text stepText;
    private Text safetyText;
    private Text cameraText;
    private Text mapText;
    private Text resultText;
    private Image safetyLamp;
    private Image emergencyFill;

    public void Build(MissionDirector director, CameraRig rig, RoverVisual roverVisual)
    {
        mission = director;
        cameraRig = rig;
        rover = roverVisual;

        var canvasObject = new GameObject("LUSI Cockpit Canvas");
        canvasObject.transform.SetParent(transform);
        Canvas = canvasObject.AddComponent<Canvas>();
        Canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        Canvas.sortingOrder = 20;
        var scaler = canvasObject.AddComponent<CanvasScaler>();
        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(SimulationProfile.ReferenceWidth, SimulationProfile.ReferenceHeight);
        scaler.screenMatchMode = CanvasScaler.ScreenMatchMode.Expand;
        canvasObject.AddComponent<GraphicRaycaster>();

        CreateImage("SIM ribbon", canvasObject.transform, new Color(0.48f, 0.05f, 0.78f, 1f), new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0f, -5f), Vector2.zero);

        var leftRail = CreateImage("Left telemetry rail", canvasObject.transform, new Color(0.025f, 0.045f, 0.055f, 0.86f), new Vector2(0f, 0.1f), new Vector2(0.185f, 0.99f), Vector2.zero, Vector2.zero);
        var rightRail = CreateImage("Right mission rail", canvasObject.transform, new Color(0.025f, 0.045f, 0.055f, 0.9f), new Vector2(0.805f, 0.1f), new Vector2(1f, 0.99f), Vector2.zero, Vector2.zero);
        var bottom = CreateImage("Bottom status strip", canvasObject.transform, new Color(0.018f, 0.028f, 0.034f, 0.96f), new Vector2(0f, 0f), new Vector2(1f, 0.1f), Vector2.zero, Vector2.zero);

        profileText = CreateLabel(leftRail.transform, "Profile", "DECK PERFORMANCE", 23, new Vector2(0f, 0.91f), new Vector2(1f, 0.98f), Color.white, TextAnchor.MiddleCenter);
        CreateLabel(leftRail.transform, "ProfileSub", "1280 × 800  /  30 FPS", 14, new Vector2(0.06f, 0.86f), new Vector2(0.94f, 0.91f), new Color(0.52f, 0.72f, 0.75f), TextAnchor.MiddleLeft);
        CreateLabel(leftRail.transform, "TelemetryTitle", "ROVER TELEMETRY", 13, new Vector2(0.06f, 0.75f), new Vector2(0.94f, 0.81f), new Color(0.45f, 0.70f, 0.72f), TextAnchor.MiddleLeft);
        CreateLabel(leftRail.transform, "SpeedCaption", "GROUND SPEED", 12, new Vector2(0.06f, 0.66f), new Vector2(0.94f, 0.71f), new Color(0.55f, 0.59f, 0.60f), TextAnchor.MiddleLeft);
        speedText = CreateLabel(leftRail.transform, "Speed", "0.00 M/S", 16, new Vector2(0.06f, 0.61f), new Vector2(0.94f, 0.67f), Color.white, TextAnchor.MiddleLeft);
        missionText = CreateLabel(leftRail.transform, "Mission", "FREE DRIVE / BASE", 15, new Vector2(0.06f, 0.53f), new Vector2(0.94f, 0.60f), Color.white, TextAnchor.MiddleLeft);
        CreateLabel(leftRail.transform, "Feed", "FPV CAMERA\nCENTER FEED", 16, new Vector2(0.06f, 0.42f), new Vector2(0.94f, 0.54f), new Color(0.78f, 0.84f, 0.82f), TextAnchor.MiddleLeft);
        safetyText = CreateLabel(leftRail.transform, "Safety", "DEADMAN: RELEASED", 13, new Vector2(0.06f, 0.18f), new Vector2(0.94f, 0.26f), new Color(1f, 0.71f, 0.33f), TextAnchor.MiddleLeft);
        safetyLamp = CreateImage("Safety lamp", leftRail.transform, new Color(1f, 0.60f, 0.18f, 1f), new Vector2(0.06f, 0.13f), new Vector2(0.11f, 0.17f), Vector2.zero, Vector2.zero);

        CreateLabel(rightRail.transform, "MissionTitle", "MISSION PHASE", 13, new Vector2(0.08f, 0.91f), new Vector2(0.92f, 0.97f), new Color(0.45f, 0.70f, 0.72f), TextAnchor.MiddleLeft);
        stepText = CreateLabel(rightRail.transform, "Step", "PARK AT GEOLOGY OUTCROP", 20, new Vector2(0.08f, 0.79f), new Vector2(0.92f, 0.9f), Color.white, TextAnchor.UpperLeft);
        resultText = CreateLabel(rightRail.transform, "Result", "SIMULATION READY", 13, new Vector2(0.08f, 0.69f), new Vector2(0.92f, 0.77f), new Color(0.50f, 0.80f, 0.73f), TextAnchor.UpperLeft);
        CreateLabel(rightRail.transform, "MapTitle", "MINIMAP / ROUTE", 13, new Vector2(0.08f, 0.56f), new Vector2(0.92f, 0.62f), new Color(0.45f, 0.70f, 0.72f), TextAnchor.MiddleLeft);
        mapText = CreateLabel(rightRail.transform, "Map", "BASE  ━━━  GEO  ━━━  MARKER\n                         ┗━━ FINAL", 12, new Vector2(0.08f, 0.43f), new Vector2(0.92f, 0.55f), new Color(0.70f, 0.78f, 0.74f), TextAnchor.UpperLeft);
        CreateLabel(rightRail.transform, "ScienceNotice", "SCIENCE OUTPUTS ARE\nSIMULATED / NO LIFE CLAIM", 12, new Vector2(0.08f, 0.24f), new Vector2(0.92f, 0.35f), new Color(0.95f, 0.63f, 0.37f), TextAnchor.UpperLeft);
        cameraText = CreateLabel(rightRail.transform, "Camera", "CAM 1 / FRONT FPV", 13, new Vector2(0.08f, 0.12f), new Vector2(0.92f, 0.19f), new Color(0.70f, 0.78f, 0.74f), TextAnchor.MiddleLeft);

        emergencyFill = CreateImage("Emergency progress", bottom.transform, new Color(0.80f, 0.18f, 0.22f, 1f), new Vector2(0.72f, 0.22f), new Vector2(0.72f, 0.34f), Vector2.zero, Vector2.zero);
        CreateLabel(bottom.transform, "Controls", "R2 / SPACE  DEADMAN       A / E  ACTION       X / C  CAMERA       Y / M  MINIMAP       HOLD B / ESC  E-STOP", 13, new Vector2(0.03f, 0.53f), new Vector2(0.97f, 0.9f), new Color(0.74f, 0.79f, 0.77f), TextAnchor.MiddleLeft);
        CreateLabel(bottom.transform, "BottomHint", "LOCAL SIMULATION  •  TERRAIN RESPONDS TO MOTION  •  NO EXTERNAL OUTPUT", 12, new Vector2(0.03f, 0.08f), new Vector2(0.68f, 0.42f), new Color(0.45f, 0.65f, 0.64f), TextAnchor.MiddleLeft);
        UpdateMapText();
    }

    private void Update()
    {
        if (mission == null || mission.State == null || rover == null)
        {
            return;
        }

        var state = mission.State;
        profileText.text = SimulationProfile.ActiveProfileName.ToUpperInvariant();
        missionText.text = "FREE DRIVE / " + state.Checkpoint.ToString().ToUpperInvariant().Replace("GEOLOGYOUTCROP", "GEOLOGY OUTCROP").Replace("FIELDMARKER", "FIELD MARKER");
        speedText.text = rover.CurrentSpeed.ToString("0.00") + " M/S";
        stepText.text = CurrentStepText(state);
        resultText.text = mission.LastResult;
        safetyText.text = state.EmergencyStopped ? "E-STOP: LATCHED" : state.DeadmanEngaged ? "DEADMAN: HELD" : "DEADMAN: RELEASED";
        safetyLamp.color = state.EmergencyStopped ? new Color(0.95f, 0.10f, 0.14f) : state.DeadmanEngaged ? new Color(0.22f, 0.92f, 0.48f) : new Color(1f, 0.60f, 0.18f);
        cameraText.text = "CAM " + (cameraRig.CameraIndex + 1) + " / " + CameraName(cameraRig.CameraIndex);
        emergencyFill.rectTransform.anchorMax = new Vector2(0.72f + Mathf.Clamp01(state.EmergencyStopProgress) * 0.23f, 0.34f);
        emergencyFill.color = state.EmergencyStopped ? new Color(1f, 0.12f, 0.16f) : new Color(0.80f, 0.18f, 0.22f, 1f);
        UpdateMapText();
    }

    private void UpdateMapText()
    {
        if (mapText == null || cameraRig == null)
        {
            return;
        }

        mapText.text = cameraRig.MinimapDetailed
            ? "BASE  ━━━  GEO  ━━━  MARKER\n  rover heading / terrain detail\n                         ┗━━ FINAL"
            : "BASE  ━━━  GEO  ━━━  MARKER\n                         ┗━━ FINAL";
    }

    private string CurrentStepText(SafeMissionState state)
    {
        switch (state.Checkpoint)
        {
            case MissionCheckpoint.Base: return "DRIVE TO GEOLOGY OUTCROP";
            case MissionCheckpoint.GeologyOutcrop: return state.ScienceComplete ? "DRIVE TO FIELD MARKER" : ScienceSequence.LabelFor(state.ExpectedScienceStep);
            case MissionCheckpoint.FieldMarker: return state.ArmComplete ? "DRIVE TO FINAL" : ArmSequence.LabelFor(state.ExpectedArmStep);
            case MissionCheckpoint.Final: return "CONFIRM COMPLETION";
            case MissionCheckpoint.Completed: return "MISSION COMPLETE / SIM ONLY";
            default: return "SIMULATION";
        }
    }

    private static string CameraName(int index)
    {
        switch (index)
        {
            case 0: return "FRONT FPV";
            case 1: return "REAR";
            case 2: return "OVERHEAD";
            default: return "ARM VIEW";
        }
    }

    private static Image CreateImage(string name, Transform parent, Color color, Vector2 anchorMin, Vector2 anchorMax, Vector2 offsetMin, Vector2 offsetMax)
    {
        var imageObject = new GameObject(name);
        imageObject.transform.SetParent(parent, false);
        var image = imageObject.AddComponent<Image>();
        image.color = color;
        var rect = image.rectTransform;
        rect.anchorMin = anchorMin;
        rect.anchorMax = anchorMax;
        rect.offsetMin = offsetMin;
        rect.offsetMax = offsetMax;
        return image;
    }

    private static Text CreateLabel(Transform parent, string name, string value, int fontSize, Vector2 anchorMin, Vector2 anchorMax, Color color, TextAnchor alignment)
    {
        var labelObject = new GameObject(name);
        labelObject.transform.SetParent(parent, false);
        var label = labelObject.AddComponent<Text>();
        label.text = value;
        label.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        label.fontSize = fontSize;
        label.color = color;
        label.alignment = alignment;
        label.horizontalOverflow = HorizontalWrapMode.Wrap;
        label.verticalOverflow = VerticalWrapMode.Overflow;
        label.raycastTarget = false;
        var rect = label.rectTransform;
        rect.anchorMin = anchorMin;
        rect.anchorMax = anchorMax;
        rect.offsetMin = new Vector2(8f, 0f);
        rect.offsetMax = new Vector2(-8f, 0f);
        return label;
    }
}
