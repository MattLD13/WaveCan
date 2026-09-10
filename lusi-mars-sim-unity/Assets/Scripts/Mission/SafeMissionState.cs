using System;
using System.Collections.Generic;

public enum MissionCheckpoint
{
    Base,
    GeologyOutcrop,
    FieldMarker,
    Final,
    Completed
}

public enum ScienceStep
{
    GeoCamera,
    RockDatabase,
    DeepSampleOver10cm,
    ShallowMaterialDiscard,
    LoadCellOver5g,
    MoistureAndTemperature,
    HydrogenPeroxide,
    CobaltBicarbonate,
    BlankCuvette,
    ControlSample,
    Reading440nm
}

public enum ArmStep
{
    PlaceSampleMarker,
    RepairBeacon,
    ActivateBeacon
}

public sealed class SafeMissionState
{
    public const float EmergencyStopHoldSeconds = 1.2f;

    private static readonly ScienceStep[] OrderedScienceSteps =
    {
        ScienceStep.GeoCamera,
        ScienceStep.RockDatabase,
        ScienceStep.DeepSampleOver10cm,
        ScienceStep.ShallowMaterialDiscard,
        ScienceStep.LoadCellOver5g,
        ScienceStep.MoistureAndTemperature,
        ScienceStep.HydrogenPeroxide,
        ScienceStep.CobaltBicarbonate,
        ScienceStep.BlankCuvette,
        ScienceStep.ControlSample,
        ScienceStep.Reading440nm
    };

    private static readonly ArmStep[] OrderedArmSteps =
    {
        ArmStep.PlaceSampleMarker,
        ArmStep.RepairBeacon,
        ArmStep.ActivateBeacon
    };

    private float emergencyStopHold;

    public MissionCheckpoint Checkpoint { get; private set; } = MissionCheckpoint.Base;
    public bool DeadmanEngaged { get; private set; }
    public bool EmergencyStopped { get; private set; }
    public float EmergencyStopProgress => emergencyStopHold / EmergencyStopHoldSeconds;
    public int ScienceIndex { get; private set; }
    public int ArmIndex { get; private set; }
    public bool ScienceComplete => ScienceIndex >= OrderedScienceSteps.Length;
    public bool ArmComplete => ArmIndex >= OrderedArmSteps.Length;
    public bool MissionComplete => Checkpoint == MissionCheckpoint.Completed;
    public ScienceStep ExpectedScienceStep => OrderedScienceSteps[Math.Min(ScienceIndex, OrderedScienceSteps.Length - 1)];
    public ArmStep ExpectedArmStep => OrderedArmSteps[Math.Min(ArmIndex, OrderedArmSteps.Length - 1)];
    public IReadOnlyList<ScienceStep> ScienceOrder => OrderedScienceSteps;
    public IReadOnlyList<ArmStep> ArmOrder => OrderedArmSteps;

    public void SetDeadman(bool engaged)
    {
        DeadmanEngaged = engaged && !EmergencyStopped;
    }

    public bool TryDrive(float throttle, float steering)
    {
        return DeadmanEngaged && !EmergencyStopped &&
               Math.Abs(throttle) <= 1f && Math.Abs(steering) <= 1f;
    }

    public bool HoldEmergencyStop(float deltaSeconds)
    {
        if (EmergencyStopped)
        {
            return true;
        }

        emergencyStopHold = Math.Min(EmergencyStopHoldSeconds, emergencyStopHold + Math.Max(0f, deltaSeconds));
        if (emergencyStopHold >= EmergencyStopHoldSeconds)
        {
            EmergencyStopped = true;
            DeadmanEngaged = false;
        }

        return EmergencyStopped;
    }

    public void ReleaseEmergencyStopHold()
    {
        if (!EmergencyStopped)
        {
            emergencyStopHold = 0f;
        }
    }

    public bool TryParkAt(MissionCheckpoint checkpoint)
    {
        if (checkpoint == MissionCheckpoint.GeologyOutcrop && Checkpoint == MissionCheckpoint.Base)
        {
            Checkpoint = checkpoint;
            return true;
        }

        if (checkpoint == MissionCheckpoint.FieldMarker && Checkpoint == MissionCheckpoint.GeologyOutcrop && ScienceComplete)
        {
            Checkpoint = checkpoint;
            return true;
        }

        if (checkpoint == MissionCheckpoint.Final && Checkpoint == MissionCheckpoint.FieldMarker && ArmComplete)
        {
            Checkpoint = checkpoint;
            return true;
        }

        return false;
    }

    public bool TryCompleteFinal()
    {
        if (Checkpoint != MissionCheckpoint.Final)
        {
            return false;
        }

        Checkpoint = MissionCheckpoint.Completed;
        return true;
    }

    public bool TryCompleteScience(ScienceStep step)
    {
        if (Checkpoint != MissionCheckpoint.GeologyOutcrop || ScienceComplete || step != ExpectedScienceStep)
        {
            return false;
        }

        ScienceIndex++;
        return true;
    }

    public bool TryCompleteArm(ArmStep step)
    {
        if (Checkpoint != MissionCheckpoint.FieldMarker || ArmComplete || step != ExpectedArmStep)
        {
            return false;
        }

        ArmIndex++;
        return true;
    }

    public void Reset()
    {
        Checkpoint = MissionCheckpoint.Base;
        DeadmanEngaged = false;
        EmergencyStopped = false;
        ScienceIndex = 0;
        ArmIndex = 0;
        emergencyStopHold = 0f;
    }
}
