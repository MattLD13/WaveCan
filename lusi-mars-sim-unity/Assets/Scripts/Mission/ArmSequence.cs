using UnityEngine;

public sealed class ArmSequence : MonoBehaviour
{
    public SafeMissionState State { get; private set; }

    public string CurrentLabel
    {
        get
        {
            if (State == null || State.ArmComplete)
            {
                return "BEACON READY";
            }

            return LabelFor(State.ExpectedArmStep);
        }
    }

    public void Bind(SafeMissionState state)
    {
        State = state;
    }

    public bool RunCurrentStep()
    {
        return State != null && State.TryCompleteArm(State.ExpectedArmStep);
    }

    public static string LabelFor(ArmStep step)
    {
        switch (step)
        {
            case ArmStep.PlaceSampleMarker: return "PLACE SAMPLE MARKER";
            case ArmStep.RepairBeacon: return "REPAIR BEACON";
            case ArmStep.ActivateBeacon: return "ACTIVATE BEACON";
            default: return "UNKNOWN ARM STEP";
        }
    }
}
