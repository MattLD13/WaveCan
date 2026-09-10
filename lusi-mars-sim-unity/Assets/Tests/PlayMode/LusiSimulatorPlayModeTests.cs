using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

public sealed class LusiSimulatorPlayModeTests
{
    [UnityTest]
    public IEnumerator StartupBuildsRuntimeAndDeckProfile()
    {
        var bootstrapObject = new GameObject("PlayMode Bootstrap Test");
        var bootstrap = bootstrapObject.AddComponent<LusiSimulatorBootstrap>();
        yield return null;

        Assert.AreEqual("Deck Performance", bootstrap.ActiveProfile);
        Assert.IsNotNull(bootstrap.World);
        Assert.IsNotNull(bootstrap.Rover);
        Assert.IsNotNull(bootstrap.CameraRig.ActiveCamera);
        Assert.IsNotNull(bootstrap.Cockpit.Canvas);
        Assert.AreEqual(30, Application.targetFrameRate);

        Object.Destroy(bootstrapObject);
        yield return null;
    }

    [UnityTest]
    public IEnumerator DeadmanGatesRoverMotion()
    {
        var state = new SafeMissionState();
        Assert.IsFalse(state.TryDrive(1f, 0f));
        state.SetDeadman(true);
        Assert.IsTrue(state.TryDrive(1f, 0f));
        yield return null;
    }

    [UnityTest]
    public IEnumerator RoverMovesAcrossTerrainWhenCommanded()
    {
        var bootstrapObject = new GameObject("Drive Motion Bootstrap Test");
        var bootstrap = bootstrapObject.AddComponent<LusiSimulatorBootstrap>();
        yield return null;

        bootstrap.Input.enabled = false;
        var start = bootstrap.Rover.transform.position;
        bootstrap.Rover.SetCommand(new Vector2(0f, 1f), true);
        yield return new WaitForSeconds(0.75f);

        var distance = bootstrap.Rover.transform.position.z - start.z;
        var speed = bootstrap.Rover.CurrentSpeed;
        bootstrap.Rover.ResetCommand();
        Object.Destroy(bootstrapObject);
        yield return null;

        Assert.Greater(distance, 0.25f);
        Assert.Greater(speed, 0f);
    }

    [UnityTest]
    public IEnumerator RoverCanUseRockAsClimbableSurface()
    {
        var bootstrapObject = new GameObject("Rock Climb Bootstrap Test");
        var bootstrap = bootstrapObject.AddComponent<LusiSimulatorBootstrap>();
        yield return null;

        var rockColliders = Object.FindObjectsOfType<MeshCollider>();
        MeshCollider largestRock = null;
        foreach (var collider in rockColliders)
        {
            if (collider.gameObject.name == "Regolith Rock" && (largestRock == null || collider.bounds.size.y > largestRock.bounds.size.y))
            {
                largestRock = collider;
            }
        }

        Assert.IsNotNull(largestRock);
        bootstrap.Rover.transform.position = largestRock.bounds.center + Vector3.up * 4f;
        bootstrap.Rover.SnapToSurface();
        Assert.AreEqual("Regolith Rock", bootstrap.Rover.SurfaceContactName);

        Object.Destroy(bootstrapObject);
        yield return null;
    }

    [UnityTest]
    public IEnumerator EmergencyStopHoldTimeIsStableInPlayMode()
    {
        var state = new SafeMissionState();
        Assert.IsFalse(state.HoldEmergencyStop(1.19f));
        yield return null;
        Assert.IsTrue(state.HoldEmergencyStop(0.01f));
        Assert.IsTrue(state.EmergencyStopped);
    }

    [UnityTest]
    public IEnumerator ScienceArmAndCheckpointOrderCompletesMission()
    {
        var state = new SafeMissionState();
        Assert.IsTrue(state.TryParkAt(MissionCheckpoint.GeologyOutcrop));
        foreach (var step in state.ScienceOrder) Assert.IsTrue(state.TryCompleteScience(step));
        Assert.IsTrue(state.TryParkAt(MissionCheckpoint.FieldMarker));
        foreach (var step in state.ArmOrder) Assert.IsTrue(state.TryCompleteArm(step));
        Assert.IsTrue(state.TryParkAt(MissionCheckpoint.Final));
        Assert.IsTrue(state.TryCompleteFinal());
        yield return null;
        Assert.IsTrue(state.MissionComplete);
    }
}
