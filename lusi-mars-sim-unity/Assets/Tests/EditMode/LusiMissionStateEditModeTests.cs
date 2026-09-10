using NUnit.Framework;

public sealed class LusiMissionStateEditModeTests
{
    [Test]
    public void MotionRequiresDeadman()
    {
        var state = new SafeMissionState();
        Assert.IsFalse(state.TryDrive(0.5f, 0f));
        state.SetDeadman(true);
        Assert.IsTrue(state.TryDrive(0.5f, 0f));
    }

    [Test]
    public void EmergencyStopLatchesOnlyAfterHoldTime()
    {
        var state = new SafeMissionState();
        state.SetDeadman(true);
        Assert.IsFalse(state.HoldEmergencyStop(1.19f));
        Assert.IsFalse(state.EmergencyStopped);
        Assert.IsTrue(state.HoldEmergencyStop(0.01f));
        Assert.IsFalse(state.TryDrive(0.5f, 0f));
    }

    [Test]
    public void ScienceOrderRejectsSkippingAndCompletesInOrder()
    {
        var state = new SafeMissionState();
        Assert.IsTrue(state.TryParkAt(MissionCheckpoint.GeologyOutcrop));
        Assert.IsFalse(state.TryCompleteScience(ScienceStep.RockDatabase));

        foreach (var step in state.ScienceOrder)
        {
            Assert.IsTrue(state.TryCompleteScience(step), step.ToString());
        }

        Assert.IsTrue(state.ScienceComplete);
        Assert.IsFalse(state.TryCompleteScience(ScienceStep.Reading440nm));
    }

    [Test]
    public void ArmOrderAndCheckpointCompletionAreGated()
    {
        var state = new SafeMissionState();
        Assert.IsFalse(state.TryParkAt(MissionCheckpoint.FieldMarker));
        Assert.IsTrue(state.TryParkAt(MissionCheckpoint.GeologyOutcrop));
        foreach (var step in state.ScienceOrder) Assert.IsTrue(state.TryCompleteScience(step));
        Assert.IsTrue(state.TryParkAt(MissionCheckpoint.FieldMarker));
        Assert.IsFalse(state.TryCompleteArm(ArmStep.RepairBeacon));
        foreach (var step in state.ArmOrder) Assert.IsTrue(state.TryCompleteArm(step));
        Assert.IsTrue(state.TryParkAt(MissionCheckpoint.Final));
        Assert.IsTrue(state.TryCompleteFinal());
        Assert.IsTrue(state.MissionComplete);
    }
}
