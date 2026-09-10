using UnityEngine;
using UnityEngine.InputSystem;

public sealed class SteamDeckInput : MonoBehaviour
{
    public SafeMissionState State { get; private set; }

    private RoverVisual rover;
    private CameraRig cameraRig;
    private MissionDirector mission;
    private bool wasActionPressed;
    private bool wasCyclePressed;
    private bool wasMinimapPressed;

    public void Bind(RoverVisual roverVisual, CameraRig rig, MissionDirector director)
    {
        rover = roverVisual;
        cameraRig = rig;
        mission = director;
        State = director.State;
    }

    private void Update()
    {
        if (State == null || rover == null)
        {
            return;
        }

        var gamepad = Gamepad.current;
        var keyboard = Keyboard.current;
        var drive = keyboard == null ? Vector2.zero : new Vector2(
            (keyboard.dKey.isPressed ? 1f : 0f) - (keyboard.aKey.isPressed ? 1f : 0f),
            (keyboard.wKey.isPressed ? 1f : 0f) - (keyboard.sKey.isPressed ? 1f : 0f));
        var look = keyboard == null ? Vector2.zero : new Vector2(
            (keyboard.rightArrowKey.isPressed ? 1f : 0f) - (keyboard.leftArrowKey.isPressed ? 1f : 0f),
            (keyboard.upArrowKey.isPressed ? 1f : 0f) - (keyboard.downArrowKey.isPressed ? 1f : 0f));

        var deadman = keyboard != null && keyboard.spaceKey.isPressed;
        var actionPressed = keyboard != null && keyboard.eKey.wasPressedThisFrame;
        var cyclePressed = keyboard != null && keyboard.cKey.wasPressedThisFrame;
        var minimapPressed = keyboard != null && keyboard.mKey.wasPressedThisFrame;
        var emergencyHeld = keyboard != null && keyboard.escapeKey.isPressed;
        var resetPressed = keyboard != null && keyboard.rKey.wasPressedThisFrame;

        if (gamepad != null)
        {
            var stickDrive = gamepad.leftStick.ReadValue();
            var stickLook = gamepad.rightStick.ReadValue();
            if (stickDrive.sqrMagnitude > 0.01f) drive = stickDrive;
            if (stickLook.sqrMagnitude > 0.01f) look = stickLook;
            deadman |= gamepad.rightTrigger.ReadValue() > 0.45f;
            actionPressed |= gamepad.buttonSouth.wasPressedThisFrame;
            cyclePressed |= gamepad.buttonWest.wasPressedThisFrame;
            minimapPressed |= gamepad.buttonNorth.wasPressedThisFrame;
            emergencyHeld |= gamepad.buttonEast.isPressed;
            resetPressed |= gamepad.startButton.wasPressedThisFrame && gamepad.selectButton.isPressed;
        }

        State.SetDeadman(deadman);
        if (emergencyHeld)
        {
            State.HoldEmergencyStop(Time.deltaTime);
        }
        else
        {
            State.ReleaseEmergencyStopHold();
        }

        if (State.TryDrive(drive.y, drive.x))
        {
            rover.SetCommand(drive, true);
        }
        else
        {
            rover.SetCommand(Vector2.zero, false);
        }

        cameraRig.SetLook(look);
        if (actionPressed && !wasActionPressed) mission.TryInteraction();
        if (cyclePressed && !wasCyclePressed) cameraRig.CycleCamera();
        if (minimapPressed && !wasMinimapPressed) cameraRig.ToggleMinimapDetail();
        if (resetPressed) mission.ResetMission();
        wasActionPressed = actionPressed;
        wasCyclePressed = cyclePressed;
        wasMinimapPressed = minimapPressed;
    }
}
