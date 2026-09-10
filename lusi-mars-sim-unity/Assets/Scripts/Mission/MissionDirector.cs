using UnityEngine;

public sealed class MissionDirector : MonoBehaviour
{
    public SafeMissionState State { get; private set; }
    public ScienceSequence Science { get; private set; }
    public ArmSequence Arm { get; private set; }
    public string LastResult { get; private set; } = "SIMULATION READY";

    private MarsWorldBuilder world;
    private RoverVisual rover;

    public void Bind(MarsWorldBuilder worldBuilder, RoverVisual roverVisual)
    {
        world = worldBuilder;
        rover = roverVisual;
        State = new SafeMissionState();

        Science = gameObject.AddComponent<ScienceSequence>();
        Science.Bind(State);
        Arm = gameObject.AddComponent<ArmSequence>();
        Arm.Bind(State);
    }

    public bool TryInteraction()
    {
        if (State == null || State.EmergencyStopped)
        {
            return false;
        }

        if (State.Checkpoint == MissionCheckpoint.GeologyOutcrop)
        {
            var complete = Science.RunCurrentStep();
            if (complete)
            {
                LastResult = Science.CurrentLabel;
            }
            return complete;
        }

        if (State.Checkpoint == MissionCheckpoint.FieldMarker)
        {
            var complete = Arm.RunCurrentStep();
            if (complete)
            {
                LastResult = Arm.CurrentLabel;
            }
            return complete;
        }

        if (State.Checkpoint == MissionCheckpoint.Final)
        {
            var complete = State.TryCompleteFinal();
            if (complete)
            {
                LastResult = "MISSION COMPLETE / SIM ONLY";
            }
            return complete;
        }

        return false;
    }

    public void ResetMission()
    {
        State.Reset();
        LastResult = "SIMULATION RESET";
        if (rover != null && world != null && world.BaseMarker != null)
        {
            rover.transform.position = world.BaseMarker.position + Vector3.up * 0.12f;
            rover.transform.rotation = Quaternion.identity;
            rover.ResetCommand();
        }
    }

    private void Update()
    {
        if (State == null || rover == null || world == null || State.EmergencyStopped || rover.CurrentSpeed > 0.18f)
        {
            return;
        }

        switch (State.Checkpoint)
        {
            case MissionCheckpoint.Base:
                TryPark(world.GeologyOutcropMarker, MissionCheckpoint.GeologyOutcrop, "GEOLOGY OUTCROP / PARKED");
                break;
            case MissionCheckpoint.GeologyOutcrop:
                TryPark(world.FieldMarker, MissionCheckpoint.FieldMarker, "FIELD MARKER / PARKED");
                break;
            case MissionCheckpoint.FieldMarker:
                TryPark(world.FinalMarker, MissionCheckpoint.Final, "FINAL APPROACH / PARKED");
                break;
            case MissionCheckpoint.Final:
                if (State.TryCompleteFinal())
                {
                    LastResult = "MISSION COMPLETE / SIM ONLY";
                }
                break;
        }
    }

    private void TryPark(Transform marker, MissionCheckpoint checkpoint, string result)
    {
        var delta = rover.transform.position - marker.position;
        delta.y = 0f;
        if (delta.sqrMagnitude <= 30.25f && State.TryParkAt(checkpoint))
        {
            LastResult = result;
        }
    }
}
