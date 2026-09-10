using UnityEngine;

public sealed class ScienceSequence : MonoBehaviour
{
    public SafeMissionState State { get; private set; }

    public string CurrentLabel
    {
        get
        {
            if (State == null || State.ScienceComplete)
            {
                return "SCIENCE COMPLETE";
            }

            return LabelFor(State.ExpectedScienceStep);
        }
    }

    public void Bind(SafeMissionState state)
    {
        State = state;
    }

    public bool RunCurrentStep()
    {
        return State != null && State.TryCompleteScience(State.ExpectedScienceStep);
    }

    public static string LabelFor(ScienceStep step)
    {
        switch (step)
        {
            case ScienceStep.GeoCamera: return "GEO CAMERA";
            case ScienceStep.RockDatabase: return "ROCK DATABASE";
            case ScienceStep.DeepSampleOver10cm: return "DEEP SAMPLE > 10 CM";
            case ScienceStep.ShallowMaterialDiscard: return "SHALLOW MATERIAL DISCARD";
            case ScienceStep.LoadCellOver5g: return "LOAD CELL > 5 G";
            case ScienceStep.MoistureAndTemperature: return "MOISTURE + TEMPERATURE";
            case ScienceStep.HydrogenPeroxide: return "H2O2 REAGENT";
            case ScienceStep.CobaltBicarbonate: return "COBALT BICARBONATE";
            case ScienceStep.BlankCuvette: return "BLANK CUVETTE";
            case ScienceStep.ControlSample: return "CONTROL SAMPLE";
            case ScienceStep.Reading440nm: return "SIMULATED 440 NM READING";
            default: return "UNKNOWN STEP";
        }
    }
}
