using UnityEngine;

public enum SimulatorQuality
{
    DeckPerformance,
    DeckQuality
}

public static class SimulationProfile
{
    public const int ReferenceWidth = 1280;
    public const int ReferenceHeight = 800;
    public const int TargetFrameRate = 30;
    public static SimulatorQuality ActiveQuality { get; private set; } = SimulatorQuality.DeckPerformance;

    public static string ActiveProfileName => ActiveQuality == SimulatorQuality.DeckPerformance
        ? "Deck Performance"
        : "Deck Quality";

    public static void Apply(SimulatorQuality quality)
    {
        ActiveQuality = quality;
        Application.targetFrameRate = TargetFrameRate;
        QualitySettings.vSyncCount = 0;
        QualitySettings.shadowDistance = quality == SimulatorQuality.DeckPerformance ? 70f : 105f;
        QualitySettings.shadowCascades = quality == SimulatorQuality.DeckPerformance ? 2 : 4;
        QualitySettings.pixelLightCount = quality == SimulatorQuality.DeckPerformance ? 1 : 2;
        QualitySettings.lodBias = quality == SimulatorQuality.DeckPerformance ? 1.1f : 1.55f;
        QualitySettings.anisotropicFiltering = AnisotropicFiltering.Enable;
        QualitySettings.realtimeReflectionProbes = false;
        QualitySettings.billboardsFaceCameraPosition = true;
    }

    public static void ApplyReferenceResolution()
    {
        if (Screen.width != ReferenceWidth || Screen.height != ReferenceHeight)
        {
            Screen.SetResolution(ReferenceWidth, ReferenceHeight, FullScreenMode.Windowed);
        }
    }
}
