using UnityEngine;
using UnityEngine.UI;

namespace Lusi.MarsSim
{
    /// <summary>
    /// Builds the LUSI Steam Deck driving cockpit at the fixed 1280 by 800
    /// reference size. The lower camera area is intentionally left untouched.
    /// </summary>
    public sealed class UIController : MonoBehaviour
    {
        private static readonly Color Band = new Color(0.043f, 0.047f, 0.058f, 0.99f);
        private static readonly Color Cell = new Color(0.060f, 0.065f, 0.078f, 1f);
        private static readonly Color Separator = new Color(0.20f, 0.21f, 0.24f, 0.92f);
        private static readonly Color TextPrimary = new Color(0.91f, 0.92f, 0.94f, 1f);
        private static readonly Color TextMuted = new Color(0.53f, 0.56f, 0.62f, 1f);
        private static readonly Color SimPurple = new Color(0.58f, 0.29f, 0.96f, 1f);
        private static readonly Color RoverGreen = new Color(0.25f, 0.86f, 0.56f, 1f);
        private static readonly Color ArmBlue = new Color(0.27f, 0.60f, 1f, 1f);
        private static readonly Color ScienceOrange = new Color(0.98f, 0.49f, 0.16f, 1f);
        private static readonly Color CautionYellow = new Color(1f, 0.82f, 0.05f, 1f);
        private static readonly Color EmergencyRed = new Color(0.91f, 0.10f, 0.12f, 1f);

        private MissionStateMachine _mission;
        private RoverController _rover;
        private SteamDeckInput _input;
        private MarsTerrainBuilder _terrain;

        private Canvas _canvas;
        private Image _moduleRibbon;
        private Image _statusVisor;
        private Text _speedText;
        private Text _headingText;
        private Text _distanceText;
        private Text _modeText;
        private Text _roverBatteryText;
        private Text _deckBatteryText;
        private Image _roverBatteryFill;
        private Image _deckBatteryFill;
        private Text _motorText;
        private Text _canText;
        private Text _commandText;
        private Text _estopText;
        private Image _estopAccent;
        private Button _commandButton;
        private bool _commandActive = true;
        private float _blinkTimer;

        public enum ModuleKind
        {
            Rover,
            Arm,
            Science,
            Simulation
        }

        public enum LinkCondition
        {
            Normal,
            Bluetooth,
            TemporaryLoss,
            Disconnected
        }

        private ModuleKind _moduleKind = ModuleKind.Simulation;
        private LinkCondition _linkCondition = LinkCondition.Normal;

        public void Initialize(
            MissionStateMachine mission,
            ScienceModuleSimulation science,
            ArmBeaconSequence arm,
            RoverController rover,
            SteamDeckInput input,
            CameraRigController cameraRig,
            QualityProfile quality,
            MarsTerrainBuilder terrain)
        {
            _mission = mission;
            _rover = rover;
            _input = input;
            _terrain = terrain;
            BuildCockpit();
        }

        public void BindInput(SteamDeckInput input)
        {
            _input = input;
        }

        /// <summary>Lets a future connected module select its ribbon treatment.</summary>
        public void SetStatusModule(ModuleKind module, LinkCondition condition)
        {
            _moduleKind = module;
            _linkCondition = condition;
            RefreshRibbon();
        }

        private void Update()
        {
            if (_canvas == null)
            {
                return;
            }

            UpdateTelemetry();
            UpdateCommandState();
            UpdateRibbonAnimation();
        }

        private void UpdateTelemetry()
        {
            if (_rover != null)
            {
                _speedText.text = Mathf.Abs(_rover.SimulatedSpeed).ToString("0.0") + " m/s";
                _headingText.text = NormalizeHeading(_rover.transform.eulerAngles.y).ToString("000") + "°";
                Vector3 position = _rover.transform.position;
                float distance = _terrain == null ? position.magnitude : position.magnitude;
                _distanceText.text = distance.ToString("0.0") + " m";
            }

            _modeText.text = "SIM";
            _roverBatteryText.text = "94%";
            _deckBatteryText.text = "100%";
            _roverBatteryFill.fillAmount = 0.94f;
            _deckBatteryFill.fillAmount = 1f;

            bool moving = _rover != null && Mathf.Abs(_rover.SimulatedSpeed) > 0.04f;
            string motorDirection = moving ? "FWD" : "IDLE";
            string motorPower = moving ? Mathf.RoundToInt(Mathf.Abs(_rover.SimulatedSpeed) * 18f).ToString() : "00";
            _motorText.text = "M1  " + motorPower + "%  " + motorDirection + "\n"
                + "M2  " + motorPower + "%  " + motorDirection + "\n"
                + "M3  " + motorPower + "%  " + motorDirection + "\n"
                + "M4  " + motorPower + "%  " + motorDirection;
            _canText.text = "CAN PORT   OK\nCAN LINK     " + (_input == null || _input.GamepadDetected ? "OK" : "SIM");
        }

        private void UpdateCommandState()
        {
            bool emergencyStopped = _mission != null && _mission.IsEmergencyStopped;
            if (emergencyStopped)
            {
                _commandActive = false;
            }

            _commandText.text = _commandActive ? "ACTIVATE" : "DEACTIVATE";
            _commandText.color = _commandActive ? RoverGreen : EmergencyRed;
            _estopText.text = emergencyStopped ? "E STOP ACTIVE" : "E STOP";
            _estopText.color = emergencyStopped ? EmergencyRed : CautionYellow;
            _estopAccent.color = emergencyStopped ? EmergencyRed : new Color(EmergencyRed.r, EmergencyRed.g, EmergencyRed.b, 0.62f);
        }

        private void UpdateRibbonAnimation()
        {
            if (_linkCondition != LinkCondition.Disconnected || _moduleRibbon == null)
            {
                return;
            }

            _blinkTimer += Time.unscaledDeltaTime;
            _moduleRibbon.enabled = Mathf.Repeat(_blinkTimer, 1f) < 0.5f;
        }

        private void BuildCockpit()
        {
            GameObject canvasObject = new GameObject("LUSI Cockpit UI · Top Band [SIM]");
            canvasObject.transform.SetParent(transform, false);
            _canvas = canvasObject.AddComponent<Canvas>();
            _canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            _canvas.sortingOrder = 100;

            CanvasScaler scaler = canvasObject.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1280f, 800f);
            scaler.screenMatchMode = CanvasScaler.ScreenMatchMode.MatchWidthOrHeight;
            scaler.matchWidthOrHeight = 0.5f;
            canvasObject.AddComponent<GraphicRaycaster>();

            RectTransform root = _canvas.GetComponent<RectTransform>();
            RectTransform band = CreatePanel("LUSI Top Status Band [SIM]", root,
                new Vector2(0f, 0.91f), new Vector2(1f, 0.99375f), Band);
            BuildBrand(band);
            BuildTelemetry(band);
            BuildBatteries(band);
            BuildMotorStatus(band);
            BuildEmergencyStop(band);
            AddSeparators(band);

            BuildLeftRail(root);
            BuildRightRail(root);

            RectTransform cameraClearance = CreatePanel("FPV Camera Clear Area", root,
                new Vector2(0f, 0f), new Vector2(1f, 0.90f), new Color(0f, 0f, 0f, 0f));
            cameraClearance.SetAsFirstSibling();

            _moduleRibbon = CreatePanel("Module Status Ribbon · 5 px [SIM]", root,
                new Vector2(0f, 0.90375f), new Vector2(1f, 0.91f), SimPurple).GetComponent<Image>();
            RefreshRibbon();
        }

        private void BuildLeftRail(RectTransform root)
        {
            RectTransform rail = CreateRail("Left Operations Rail", root, 0.012f, 0.162f);
            BuildRailCard(rail, "CAMERAS", "LIVE", 0.965f, 0.78f);
            CreateRailButton(rail, "FRONT", 0.035f, 0.69f, 0.48f, 0.755f, true);
            CreateRailButton(rail, "REAR", 0.52f, 0.69f, 0.965f, 0.755f, false);
            CreateRailButton(rail, "ARM", 0.035f, 0.60f, 0.48f, 0.665f, false);
            CreateRailButton(rail, "OVERHEAD", 0.52f, 0.60f, 0.965f, 0.665f, false);
            CreateRailButton(rail, "GEO HD", 0.035f, 0.515f, 0.965f, 0.575f, false);
            CreateText(rail, "Active Feed", "ACTIVE FEED                                  FRONT", 7, TextMuted,
                new Vector2(10f, 0.475f * 800f), new Vector2(-10f, 0.505f * 800f), TextAnchor.MiddleLeft);

            BuildRailCard(rail, "MACROS", "LOCAL", 0.455f, 0.285f);
            CreateRailButton(rail, "GEO SCAN", 0.035f, 0.385f, 0.48f, 0.435f, false);
            CreateRailButton(rail, "DEEP SAMPLE", 0.52f, 0.385f, 0.965f, 0.435f, false);
            CreateRailButton(rail, "SOIL READ", 0.035f, 0.315f, 0.48f, 0.365f, false);
            CreateRailButton(rail, "ARM MACRO", 0.52f, 0.315f, 0.965f, 0.365f, false);
            CreateText(rail, "Macro State", "MACRO STATE                                      ARM 1/4", 7, TextMuted,
                new Vector2(10f, 0.275f * 800f), new Vector2(-10f, 0.30f * 800f), TextAnchor.MiddleLeft);

            BuildRailCard(rail, "INPUT DETAILS", "READY", 0.255f, 0.035f);
            CreateText(rail, "Stick Guidance", "L STICK                                      R STICK\nDRIVE / STEER                         CAMERA LOOK\n\nR2 / SPACE                                HOLD DEADMAN\nB / ESC                                      STOP OUTPUT\nX                                                CYCLE CAMERA", 7, TextPrimary,
                new Vector2(10f, 0.075f * 800f), new Vector2(-10f, 0.235f * 800f), TextAnchor.UpperLeft);
            CreateText(rail, "Deadman", "●  DEADMAN                                      HOLD TO ENABLE", 7, RoverGreen,
                new Vector2(10f, 0.045f * 800f), new Vector2(-10f, 0.075f * 800f), TextAnchor.MiddleLeft);
        }

        private void BuildRightRail(RectTransform root)
        {
            RectTransform rail = CreateRail("Right Operations Rail", root, 0.842f, 0.988f);
            BuildRailCard(rail, "ROVER MAP", "LIVE", 0.965f, 0.79f);
            CreateImage("Map Route", rail, new Vector2(0.08f, 0.805f), new Vector2(0.92f, 0.935f), new Color(0.08f, 0.09f, 0.11f, 1f));
            CreateText(rail, "Map Route", "BASE                OUTCROP\n              ●──────●\n                         MARKER", 6, TextMuted,
                new Vector2(10f, 0.80f * 800f), new Vector2(-10f, 0.925f * 800f), TextAnchor.MiddleLeft);
            CreateText(rail, "Map Footer", "PATH 12.4 M                                      MARKER", 6, TextMuted,
                new Vector2(10f, 0.765f * 800f), new Vector2(-10f, 0.79f * 800f), TextAnchor.MiddleLeft);

            BuildRailCard(rail, "MISSION 01", "ACTIVE", 0.755f, 0.605f);
            CreateText(rail, "Mission Step", "PLACE MARKER + REPAIR BEACON\nHold position while the arm macro\nplaces and verifies the beacon.", 7, TextPrimary,
                new Vector2(10f, 0.63f * 800f), new Vector2(-10f, 0.735f * 800f), TextAnchor.UpperLeft);
            CreateText(rail, "Mission Progress", "● APPROACH OUTCROP     DONE\n● PLACE SAMPLE MARKER   RUNNING\n○ REPAIR FIELD BEACON   WAIT\n○ VERIFY BEACON PULSE    WAIT", 6, TextMuted,
                new Vector2(10f, 0.475f * 800f), new Vector2(-10f, 0.59f * 800f), TextAnchor.UpperLeft);

            BuildRailCard(rail, "BEACON", "PARKED", 0.57f, 0.19f);
            CreateText(rail, "Beacon Status", "FIELD MARKER PARKED                 ARM / SIM\n\n● ALIGN ROVER                         DONE\n● PLACE SAMPLE MARKER            RUNNING\n○ REPAIR FIELD BEACON              WAIT\n○ VERIFY BEACON PULSE              WAIT", 6, TextMuted,
                new Vector2(10f, 0.235f * 800f), new Vector2(-10f, 0.545f * 800f), TextAnchor.UpperLeft);
            CreateRailButton(rail, "ARM MACRO 1/4", 0.08f, 0.205f, 0.92f, 0.255f, false);

            BuildRailCard(rail, "SAFETY / LINK", "SIM", 0.16f, 0.035f);
            CreateText(rail, "Link Status", "● SIM LINK                         CONNECTED\n● WATCHDOG                              100 MS\n\n                                  STOP OUTPUT", 6, TextMuted,
                new Vector2(10f, 0.055f * 800f), new Vector2(-10f, 0.145f * 800f), TextAnchor.UpperLeft);
        }

        private static RectTransform CreateRail(string name, RectTransform root, float left, float right)
        {
            RectTransform rail = CreatePanel(name, root, new Vector2(left, 0.045f), new Vector2(right, 0.885f),
                new Color(0.035f, 0.043f, 0.052f, 0.96f));
            Color border = new Color(0.17f, 0.19f, 0.23f, 0.9f);
            CreateImage(name + " Border Top", rail, new Vector2(0f, 0.997f), new Vector2(1f, 1f), border);
            CreateImage(name + " Border Bottom", rail, new Vector2(0f, 0f), new Vector2(1f, 0.003f), border);
            CreateImage(name + " Border Left", rail, new Vector2(0f, 0f), new Vector2(0.004f, 1f), border);
            CreateImage(name + " Border Right", rail, new Vector2(0.996f, 0f), new Vector2(1f, 1f), border);
            return rail;
        }

        private static void BuildRailCard(RectTransform rail, string title, string state, float top, float bottom)
        {
            CreateImage(title + " Card", rail, new Vector2(0.025f, bottom), new Vector2(0.975f, top), Cell);
            CreateText(rail, title + " Title", title, 7, TextMuted,
                new Vector2(10f, (top - 0.035f) * 800f), new Vector2(-70f, (top - 0.075f) * 800f), TextAnchor.MiddleLeft);
            CreateText(rail, title + " State", state, 7, RoverGreen,
                new Vector2(10f, (top - 0.035f) * 800f), new Vector2(-10f, (top - 0.075f) * 800f), TextAnchor.MiddleRight);
        }

        private static void CreateRailButton(RectTransform rail, string label, float left, float bottom, float right, float top, bool selected)
        {
            Image image = CreateImage(label + " Rail Button", rail, new Vector2(left, bottom), new Vector2(right, top),
                selected ? new Color(0.16f, 0.10f, 0.25f, 1f) : new Color(0.05f, 0.06f, 0.075f, 1f));
            image.color = selected ? new Color(0.16f, 0.10f, 0.25f, 1f) : new Color(0.05f, 0.06f, 0.075f, 1f);
            CreateText(rail, label + " Rail Label", label, 7, selected ? TextPrimary : TextMuted,
                new Vector2(left * 1280f, bottom * 800f), new Vector2(-(1f - right) * 1280f, -(1f - top) * 800f), TextAnchor.MiddleCenter);
        }

        private void BuildBrand(RectTransform band)
        {
            RectTransform brand = CreateSection("Brand · LUSI", band, 0.015f, 0.19f);
            Texture2D logo = Resources.Load<Texture2D>("Textures/lusi-wordmark");
            if (logo != null)
            {
                GameObject logoObject = new GameObject("LUSI Wordmark · Fit");
                logoObject.transform.SetParent(brand, false);
                RectTransform rect = logoObject.AddComponent<RectTransform>();
                rect.anchorMin = new Vector2(0f, 0.13f);
                rect.anchorMax = new Vector2(1f, 0.87f);
                rect.offsetMin = new Vector2(10f, 0f);
                rect.offsetMax = new Vector2(-10f, 0f);
                Image image = logoObject.AddComponent<Image>();
                image.sprite = Sprite.Create(logo, new Rect(0f, 0f, logo.width, logo.height),
                    new Vector2(0f, 0.5f), 100f);
                image.preserveAspect = true;
                image.raycastTarget = false;
            }
            else
            {
                CreateText(brand, "LUSI Fallback Wordmark", "LUSI", 25, TextPrimary,
                    Vector2.zero, Vector2.zero, TextAnchor.MiddleCenter);
            }
            CreateText(brand, "Mission Label", "MARS ANALOG 01", 8, TextMuted,
                new Vector2(10f, 2f), new Vector2(-10f, 12f), TextAnchor.LowerLeft);
        }

        private void BuildTelemetry(RectTransform band)
        {
            RectTransform telemetry = CreateSection("Driving Data", band, 0.205f, 0.375f);
            _speedText = CreateMetric(telemetry, "SPEED", "0.0 m/s", 0.00f, 0.25f);
            _headingText = CreateMetric(telemetry, "HDG", "000°", 0.25f, 0.50f);
            _distanceText = CreateMetric(telemetry, "DIST", "0.0 m", 0.50f, 0.75f);
            _modeText = CreateMetric(telemetry, "MODE", "SIM", 0.75f, 1.00f);
        }

        private void BuildBatteries(RectTransform band)
        {
            RectTransform batteries = CreateSection("Power", band, 0.39f, 0.565f);
            BuildBattery(batteries, "ROVER", "94%", 0f, out _roverBatteryText, out _roverBatteryFill);
            BuildBattery(batteries, "DECK", "100%", 0.52f, out _deckBatteryText, out _deckBatteryFill);
        }

        private void BuildMotorStatus(RectTransform band)
        {
            RectTransform motors = CreateSection("Motor and CAN Status", band, 0.58f, 0.755f);
            CreateText(motors, "Motor Heading", "MOTORS", 8, TextMuted,
                new Vector2(4f, -4f), new Vector2(-4f, -14f), TextAnchor.UpperLeft);
            _motorText = CreateText(motors, "Individual Motor Status",
                "M1  00%  IDLE\nM2  00%  IDLE\nM3  00%  IDLE\nM4  00%  IDLE", 8, TextPrimary,
                new Vector2(4f, -17f), new Vector2(-92f, -5f), TextAnchor.UpperLeft);
            _canText = CreateText(motors, "CAN Status", "CAN PORT   OK\nCAN LINK     SIM", 8, TextMuted,
                new Vector2(114f, -17f), new Vector2(-3f, -5f), TextAnchor.UpperLeft);
        }

        private void BuildEmergencyStop(RectTransform band)
        {
            RectTransform area = CreateSection("Command and E Stop", band, 0.77f, 0.99f);
            GameObject accentObject = new GameObject("E Stop Red Accent");
            accentObject.transform.SetParent(area, false);
            RectTransform accentRect = accentObject.AddComponent<RectTransform>();
            accentRect.anchorMin = new Vector2(0f, 0f);
            accentRect.anchorMax = new Vector2(0f, 1f);
            accentRect.offsetMin = new Vector2(0f, 7f);
            accentRect.offsetMax = new Vector2(3f, -7f);
            _estopAccent = accentObject.AddComponent<Image>();

            GameObject buttonObject = new GameObject("Activate Deactivate Command Button");
            buttonObject.transform.SetParent(area, false);
            RectTransform buttonRect = buttonObject.AddComponent<RectTransform>();
            buttonRect.anchorMin = new Vector2(0.04f, 0.17f);
            buttonRect.anchorMax = new Vector2(0.40f, 0.83f);
            buttonRect.offsetMin = Vector2.zero;
            buttonRect.offsetMax = Vector2.zero;
            Image buttonImage = buttonObject.AddComponent<Image>();
            buttonImage.color = Cell;
            _commandButton = buttonObject.AddComponent<Button>();
            _commandButton.targetGraphic = buttonImage;
            _commandButton.onClick.AddListener(ToggleCommandState);
            _commandText = CreateText(buttonRect, "Activate Deactivate Text", "ACTIVATE", 9, RoverGreen,
                Vector2.zero, Vector2.zero, TextAnchor.MiddleCenter);

            RectTransform stop = CreateSection("Emergency Stop Control", area, 0.44f, 1f);
            Image stopBackground = stop.gameObject.GetComponent<Image>();
            stopBackground.color = new Color(0.10f, 0.035f, 0.04f, 1f);
            CreateDashedCautionBorder(stop);
            CreateText(stop, "E Stop Label", "HOLD TO STOP", 7, TextMuted,
                new Vector2(10f, -5f), new Vector2(-10f, -15f), TextAnchor.UpperCenter);
            _estopText = CreateText(stop, "Emergency Stop State", "E STOP", 12, CautionYellow,
                new Vector2(10f, -16f), new Vector2(-10f, 7f), TextAnchor.MiddleCenter);
            CreateText(stop, "E Stop Input", "B / ESC  1.2 s", 7, TextMuted,
                new Vector2(10f, 5f), new Vector2(-10f, 0f), TextAnchor.LowerCenter);
        }

        private void AddSeparators(RectTransform band)
        {
            float[] positions = { 0.195f, 0.38f, 0.575f, 0.765f };
            for (int i = 0; i < positions.Length; i++)
            {
                GameObject separator = new GameObject("Status Section Separator " + i);
                separator.transform.SetParent(band, false);
                RectTransform rect = separator.AddComponent<RectTransform>();
                rect.anchorMin = new Vector2(positions[i], 0.12f);
                rect.anchorMax = new Vector2(positions[i], 0.88f);
                rect.offsetMin = new Vector2(0f, 0f);
                rect.offsetMax = new Vector2(1f, 0f);
                Image image = separator.AddComponent<Image>();
                image.color = Separator;
                image.raycastTarget = false;
            }
        }

        private void BuildBattery(RectTransform parent, string label, string value, float x,
            out Text valueText, out Image fill)
        {
            RectTransform cell = CreateSection("Battery " + label, parent, x, x + 0.48f);
            CreateText(cell, label + " Label", label, 7, TextMuted,
                new Vector2(3f, -5f), new Vector2(-3f, -15f), TextAnchor.UpperCenter);

            GameObject outerObject = new GameObject(label + " Battery Rectangle");
            outerObject.transform.SetParent(cell, false);
            RectTransform outerRect = outerObject.AddComponent<RectTransform>();
            outerRect.anchorMin = new Vector2(0.08f, 0.20f);
            outerRect.anchorMax = new Vector2(0.92f, 0.64f);
            outerRect.offsetMin = Vector2.zero;
            outerRect.offsetMax = Vector2.zero;
            Image outer = outerObject.AddComponent<Image>();
            outer.color = new Color(0.15f, 0.16f, 0.19f, 1f);
            outer.raycastTarget = false;

            GameObject fillObject = new GameObject(label + " Battery Fill");
            fillObject.transform.SetParent(outerObject.transform, false);
            RectTransform fillRect = fillObject.AddComponent<RectTransform>();
            fillRect.anchorMin = new Vector2(0.06f, 0.18f);
            fillRect.anchorMax = new Vector2(0.94f, 0.82f);
            fillRect.offsetMin = Vector2.zero;
            fillRect.offsetMax = Vector2.zero;
            fill = fillObject.AddComponent<Image>();
            fill.type = Image.Type.Filled;
            fill.fillMethod = Image.FillMethod.Horizontal;
            fill.fillOrigin = 0;
            fill.color = TextPrimary;
            fill.raycastTarget = false;
            valueText = CreateText(cell, label + " Battery Percent", value, 9, TextPrimary,
                new Vector2(3f, 1f), new Vector2(-3f, 0f), TextAnchor.LowerCenter);
        }

        private void CreateDashedCautionBorder(RectTransform parent)
        {
            const int dashCount = 8;
            for (int i = 0; i < dashCount; i++)
            {
                float left = 0.06f + (0.88f * i / dashCount);
                float right = left + 0.88f / dashCount * 0.64f;
                CreateImage("Caution Top Dash " + i, parent, new Vector2(left, 0.89f),
                    new Vector2(right, 0.98f), CautionYellow);
                CreateImage("Caution Bottom Dash " + i, parent, new Vector2(left, 0.02f),
                    new Vector2(right, 0.11f), CautionYellow);
            }
            for (int i = 0; i < 2; i++)
            {
                float bottom = 0.18f + i * 0.32f;
                CreateImage("Caution Left Dash " + i, parent, new Vector2(0.02f, bottom),
                    new Vector2(0.08f, bottom + 0.20f), CautionYellow);
                CreateImage("Caution Right Dash " + i, parent, new Vector2(0.92f, bottom),
                    new Vector2(0.98f, bottom + 0.20f), CautionYellow);
            }
        }

        private void RefreshRibbon()
        {
            if (_moduleRibbon == null)
            {
                return;
            }

            _moduleRibbon.color = ModuleColor(_moduleKind);
            if (_statusVisor != null)
            {
                Destroy(_statusVisor.gameObject);
                _statusVisor = null;
            }

            if (_linkCondition == LinkCondition.Bluetooth || _linkCondition == LinkCondition.TemporaryLoss)
            {
                Color visorColor = _linkCondition == LinkCondition.Bluetooth ? ArmBlue : CautionYellow;
                _statusVisor = CreateImage("Status Visor", _moduleRibbon.transform.parent,
                    new Vector2(0.015f, 0f), new Vector2(0.050f, 1f), visorColor);
                _statusVisor.transform.SetAsLastSibling();
            }
            else if (_linkCondition == LinkCondition.Disconnected)
            {
                _moduleRibbon.color = EmergencyRed;
                _moduleRibbon.enabled = true;
                _blinkTimer = 0f;
            }
        }

        private void ToggleCommandState()
        {
            if (_mission != null && _mission.IsEmergencyStopped)
            {
                return;
            }
            _commandActive = !_commandActive;
            UpdateCommandState();
        }

        private static Color ModuleColor(ModuleKind module)
        {
            switch (module)
            {
                case ModuleKind.Arm:
                    return ArmBlue;
                case ModuleKind.Science:
                    return ScienceOrange;
                case ModuleKind.Rover:
                    return RoverGreen;
                default:
                    return SimPurple;
            }
        }

        private static int NormalizeHeading(float heading)
        {
            return Mathf.RoundToInt(Mathf.Repeat(heading, 360f));
        }

        private static RectTransform CreateSection(string name, Transform parent, float left, float right)
        {
            return CreatePanel(name, parent, new Vector2(left, 0f), new Vector2(right, 1f), Color.clear);
        }

        private static RectTransform CreatePanel(string name, Transform parent, Vector2 anchorMin,
            Vector2 anchorMax, Color color)
        {
            GameObject panel = new GameObject(name);
            panel.transform.SetParent(parent, false);
            RectTransform rect = panel.AddComponent<RectTransform>();
            rect.anchorMin = anchorMin;
            rect.anchorMax = anchorMax;
            rect.offsetMin = Vector2.zero;
            rect.offsetMax = Vector2.zero;
            Image image = panel.AddComponent<Image>();
            image.color = color;
            image.raycastTarget = false;
            return rect;
        }

        private static Image CreateImage(string name, Transform parent, Vector2 anchorMin,
            Vector2 anchorMax, Color color)
        {
            GameObject imageObject = new GameObject(name);
            imageObject.transform.SetParent(parent, false);
            RectTransform rect = imageObject.AddComponent<RectTransform>();
            rect.anchorMin = anchorMin;
            rect.anchorMax = anchorMax;
            rect.offsetMin = Vector2.zero;
            rect.offsetMax = Vector2.zero;
            Image image = imageObject.AddComponent<Image>();
            image.color = color;
            image.raycastTarget = false;
            return image;
        }

        private static Text CreateMetric(Transform parent, string label, string value, float left, float right)
        {
            RectTransform cell = CreateSection(label + " Metric", parent, left, right);
            CreateText(cell, label + " Label", label, 7, TextMuted,
                new Vector2(2f, -7f), new Vector2(-2f, -17f), TextAnchor.UpperCenter);
            return CreateText(cell, label + " Value", value, 10, TextPrimary,
                new Vector2(2f, -19f), new Vector2(-2f, 4f), TextAnchor.MiddleCenter);
        }

        private static Text CreateText(Transform parent, string name, string value, int size,
            Color color, Vector2 offsetMin, Vector2 offsetMax, TextAnchor alignment)
        {
            GameObject textObject = new GameObject(name);
            textObject.transform.SetParent(parent, false);
            RectTransform rect = textObject.AddComponent<RectTransform>();
            rect.anchorMin = new Vector2(0f, 0f);
            rect.anchorMax = new Vector2(1f, 1f);
            rect.offsetMin = offsetMin;
            rect.offsetMax = offsetMax;
            Text text = textObject.AddComponent<Text>();
            text.font = Resources.GetBuiltinResource<Font>("Arial.ttf");
            text.fontSize = size;
            text.fontStyle = FontStyle.Normal;
            text.color = color;
            text.alignment = alignment;
            text.horizontalOverflow = HorizontalWrapMode.Wrap;
            text.verticalOverflow = VerticalWrapMode.Truncate;
            text.lineSpacing = 0.95f;
            text.text = value;
            text.raycastTarget = false;
            return text;
        }
    }
}
