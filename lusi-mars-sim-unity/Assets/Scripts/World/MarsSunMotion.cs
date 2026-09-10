using UnityEngine;

public sealed class MarsSunMotion : MonoBehaviour
{
    private Light sunLight;
    private Transform sunDisc;
    private float azimuth;
    private const float OrbitRadius = 520f;
    private const float DayLengthSeconds = 240f;

    public void Bind(Light lightSource, Transform disc, Vector3 initialDirection)
    {
        sunLight = lightSource;
        sunDisc = disc;
        azimuth = Mathf.Atan2(initialDirection.x, initialDirection.z);
        UpdateSun(0f);
    }

    private void Update()
    {
        UpdateSun(Time.deltaTime / DayLengthSeconds);
    }

    private void UpdateSun(float phaseDelta)
    {
        if (sunLight == null || sunDisc == null)
        {
            return;
        }

        azimuth = Mathf.Repeat(azimuth + phaseDelta * Mathf.PI * 2f, Mathf.PI * 2f);
        var phase = Mathf.Repeat(azimuth / (Mathf.PI * 2f), 1f);
        var elevation = 15f + 20f * (0.5f - 0.5f * Mathf.Cos(phase * Mathf.PI * 2f));
        var elevationRadians = elevation * Mathf.Deg2Rad;
        var horizontal = Mathf.Cos(elevationRadians);
        var direction = new Vector3(
            Mathf.Sin(azimuth) * horizontal,
            Mathf.Sin(elevationRadians),
            Mathf.Cos(azimuth) * horizontal).normalized;

        sunLight.transform.rotation = Quaternion.LookRotation(-direction, Vector3.up);
        sunDisc.position = direction * OrbitRadius;
    }
}
