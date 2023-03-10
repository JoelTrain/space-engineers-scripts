/*
 * R e a d m e
 * -----------
 * 
 * In this file you can include any instructions or other comments you want to have injected onto the 
 * top of your final script. You can safely delete this file if you do not want any such comments.
 */

// Auto Piston Docking [APD]
// if you get an error message, try fixing the issue and recompiling the script

// you are welcome to change the variables in this section

string _broadCastTag = "Docking request channel 4A1E [APD]";
string scriptTag = "[APD]";    // only connectors on rovers with this tag in the name will be used for docking
string ignoreTag = "[ignore]"; // script will ignore this block (currently only used for cockpits and programmable block screens)

// OPTIONAL
// starts spinning after receiving a docking request
// spins red if the connectors are not algined straight
// spins yellow if the connector are aligned straight
string _lightName = "Rotating Light [APD]"; // optional name of a spinning light

// set this to false to stop receiving
bool enableDebugOutput = false;

bool enableStowing = true; // when true it will attempt to "stow" the connector into a safe place when not in use
bool retractWhenStowed = true; // pistons will fully retract to stow away when undocked. set to false to extend when stowed

// Connector offset
// The number of meters to offset the desired position from the center of the connector on the rover.
// Small Block Grid Connectors need a 0.75m offset
// Large Block Grid Connectors need a 1.5m offset
float smallConnectorOffset = 0.75f;
float largeConnectorOffset = 1.5f;
float connectorOffset = 0.75f;

// private variables not meant to be changed
int _runcount = 0;
IMyBroadcastListener _myBroadcastListener;
UpdateType triggerUpdate = UpdateType.Trigger | UpdateType.Terminal | UpdateType.Mod | UpdateType.Script;
List<IMyTextSurface> outputScreens = new List<IMyTextSurface>();
bool baseReady = false;
IMyReflectorLight light;
IMyShipConnector roverConnector;

List<IMyPistonBase> tempPistonList = new List<IMyPistonBase>();
List<IMyShipConnector> tempConnectorList = new List<IMyShipConnector>();

class PistonMinMax
{
    public IMyPistonBase piston;
    public float min = 0.0f;
    public float max = 10.0f;
}

class DockingBay
{
    public PistonMinMax root = new PistonMinMax();
    public PistonMinMax middle = new PistonMinMax();
    public PistonMinMax end = new PistonMinMax();
    public IMyShipConnector connector;
    public MyShipConnectorStatus lastStatus;
    public TimeSpan busyTime = TimeSpan.Zero;
    public bool stowed = false;
}

List<DockingBay> bays = new List<DockingBay>();

public Program()
{
    try
    {
        setupOutputSurface();

        Echo = EchoToLCD;
        ClearLCD();
        Debug("Compiled and loaded.");

        light = GridTerminalSystem.GetBlockWithName(_lightName) as IMyReflectorLight;
        spinLight(false, false);

        List<IMyPistonBase> pistonsDirectlyOnBase = new List<IMyPistonBase>();
        GridTerminalSystem.GetBlocksOfType(pistonsDirectlyOnBase, piston => piston.CubeGrid == Me.CubeGrid);

        foreach(IMyPistonBase piston in pistonsDirectlyOnBase)
        {
            DockingBay newPotentialBay = new DockingBay();
            newPotentialBay.stowed = false;
            newPotentialBay.busyTime = TimeSpan.Zero;
            newPotentialBay.root.piston = piston;
            getPistonCustomDataMinMax(newPotentialBay.root);
            IMyCubeGrid middleGrid = piston.TopGrid;

            // Look for middle piston
            tempPistonList.Clear();
            GridTerminalSystem.GetBlocksOfType(tempPistonList, tempMiddlePiston => tempMiddlePiston.CubeGrid == middleGrid);
            if(tempPistonList.Count != 1)
            {
                continue;
            }
            newPotentialBay.middle.piston = tempPistonList[0];
            getPistonCustomDataMinMax(newPotentialBay.middle);
            IMyCubeGrid endGrid = newPotentialBay.middle.piston.TopGrid;

            // Look for end piston
            tempPistonList.Clear();
            GridTerminalSystem.GetBlocksOfType(tempPistonList, tempEndPiston => tempEndPiston.CubeGrid == endGrid);
            if (tempPistonList.Count != 1)
            {
                continue;
            }
            newPotentialBay.end.piston = tempPistonList[0];
            getPistonCustomDataMinMax(newPotentialBay.end);

            // Look for connector on end
            tempConnectorList.Clear();
            GridTerminalSystem.GetBlocksOfType(tempConnectorList, connectorOnEnd => connectorOnEnd.CubeGrid == newPotentialBay.end.piston.TopGrid);
            if (tempConnectorList.Count != 1)
            {
                continue;
            }
            newPotentialBay.connector = tempConnectorList[0];
            newPotentialBay.lastStatus = newPotentialBay.connector.Status;

            bays.Add(newPotentialBay);
        }

        List<IMyRadioAntenna> antennae = new List<IMyRadioAntenna>();
        GridTerminalSystem.GetBlocksOfType(antennae, antenna => antenna.IsWorking);
        int activeAntennaeCount = antennae.Count;

        _myBroadcastListener = IGC.RegisterBroadcastListener(_broadCastTag);
        _myBroadcastListener.SetMessageCallback(_broadCastTag);

        if(activeAntennaeCount < 1)
        {
            Echo("Missing active antenna. Must have broadcasting antenna to tranceive docking requests.");
            return;
        }

        if (bays.Count < 1)
        {
            Echo("Enabling mode: Rover/Ship");
            tempConnectorList.Clear();
            GridTerminalSystem.GetBlocksOfType(tempConnectorList);

            if(tempConnectorList.Count > 1)
            {
                GridTerminalSystem.GetBlocksOfType(tempConnectorList, aRoverConnector => aRoverConnector.CustomName.Contains(scriptTag));

                if (tempConnectorList.Count > 1)
                {
                    Echo($"Error: only one connector allowed with {scriptTag} tag on connecting craft.");
                }

            }
            if(tempConnectorList.Count < 1)
            {
                Echo("Error: No connectors found. Please add a connector and recompile.");
                return;
            }

            roverConnector = tempConnectorList[0];
            MyCubeSize gridSize = roverConnector.CubeGrid.GridSizeEnum;

            if (gridSize == MyCubeSize.Large)
            {
                connectorOffset = largeConnectorOffset;
                Debug($"Craft connector is a large block, setting offset to {connectorOffset}");
            }
            else if (gridSize == MyCubeSize.Small)
            {
                connectorOffset = smallConnectorOffset;
                Debug($"Craft connector is a large block, setting offset to {connectorOffset}");
            }

            Echo("Ready to transmit! Run this script block when ready to dock.");

            return;
        }

        Echo("Enabling mode: Docking Bay/Station");
        Echo($"{bays.Count} docking bays found.");

        if(enableStowing)
        {
          Echo("Docking Bay stowing on disconnect: enabled");
          if(retractWhenStowed)
          {
            Echo($"Docking pistons will retract upon disconnect");
          }
          else
          {
            Echo($"docking pistons will extend upon disconnect");
          }
        }
        Echo("Base is ready to receive docking requests!");
        baseReady = true;
    }
    catch (Exception e)
    {
        ExceptionHandler(e);
    }
}

public void Save()
{
    // Called when the program needs to save its state. Use
    // this method to save your state to the Storage field
    // or some other means.
    //
    // This method is optional and can be removed if not
    // needed.
}

public void Main(string argument, UpdateType updateSource)
{
    try
    {

        if(_runcount > 10000)
        {
          _runcount = 0;
        }
        Echo(_runcount.ToString() + ":" + updateSource.ToString());

        if ((updateSource & triggerUpdate) > 0)
        {
            ClearLCD();
            if (!String.IsNullOrEmpty(argument))
            {
                IGC.SendBroadcastMessage(_broadCastTag, argument);
                Echo("Sending message:\n" + argument);
            }

            if (roverConnector != null)
            {
                var portOrientation = Vector3D.Normalize(roverConnector.WorldMatrix.Forward);
                var portPosition = roverConnector.GetPosition() + (portOrientation * connectorOffset);
                string requestBody = $"Requesting automatic docking for {roverConnector.CubeGrid.CustomName}";
                var request = new MyTuple<string, string, Vector3D, Vector3D>("Docking Request", requestBody, portPosition, portOrientation);
                Echo("Sending docking request");
                Debug(toGPS(roverConnector.GetPosition(), "connectorPos") + "\n");
                Debug(toGPS(portPosition, "offsetPos") + "\n");
                IGC.SendBroadcastMessage(_broadCastTag, request);

            }
        }
        // If the update source has this update flag, it means
        // that it's run from the frequency system, and we should
        // update our continuous logic.
        if ((updateSource & UpdateType.Update100) != 0)
        {
            if(_runcount % 100 == 0)
            {
                ClearLCD();
            }
            foreach(DockingBay bay in bays)
            {
                Debug(Runtime.TimeSinceLastRun.ToString());
                if(bay.busyTime > TimeSpan.Zero)
                {
                    bay.busyTime = bay.busyTime - Runtime.TimeSinceLastRun;
                    Debug(bay.busyTime.ToString());
                }
                if(bay.connector.Status != MyShipConnectorStatus.Connected)
                {
                    // we were connected but now we are not or we have been sitting idle without a connection for too long
                    // so we need to begin to stow
                    bool justDisconnected = bay.lastStatus == MyShipConnectorStatus.Connected;
                    if(justDisconnected || bay.busyTime <= TimeSpan.Zero)
                    {
                        if(enableStowing)
                        {
                            if(justDisconnected)
                            {
                                Echo($"Stowing bay: {bay.connector.CustomName}");
                            }
                            stowBay(bay, retractWhenStowed);
                        }
                    }
                    // we were not connected
                    // but now we can and our time limit is up
                    else if(bay.connector.Status == MyShipConnectorStatus.Connectable)
                    {
                        bay.connector.Connect();
                    }
                }

                bay.lastStatus = bay.connector.Status;
            }
        }
        if ((updateSource & UpdateType.IGC) > 0)
        {
            while (_myBroadcastListener.HasPendingMessage)
            {
                MyIGCMessage myIGCMessage = _myBroadcastListener.AcceptMessage();
                if (myIGCMessage.Tag == _broadCastTag)
                {
                    // ClearLCD();
                    if (myIGCMessage.Data is string)
                    {
                        string messageData = myIGCMessage.Data as string;
                        string str = myIGCMessage.Data.ToString();
                        Echo("Received IGC Public Message");
                        //Echo("Tag=" + myIGCMessage.Tag);
                        Echo("Data=" + myIGCMessage.Data.ToString());
                        //Echo("Source=" + myIGCMessage.Source.ToString("X"));
                    }
                    else if(myIGCMessage.Data is MyTuple<string, string, Vector3D, Vector3D>)
                    {
                        var messageData = (MyTuple<string, string, Vector3D, Vector3D>)myIGCMessage.Data;
                        if (messageData.Item1 != "Docking Request" || !baseReady)
                        {
                            continue;
                        }

                        var otherPortPosition = messageData.Item3;
                        var otherPortOrientation = messageData.Item4;

                        Debug("Received Docking request: ");
                        Debug(messageData.Item2);
                        Debug(toGPS(messageData.Item3, "Position"));
                        Debug($"Orientation: {messageData.Item4}");

                        DockingBay closestBay = findClosestBayToPosition(otherPortPosition, bays);

                        IMyShipConnector baseConnector = closestBay.connector;

                        if(baseConnector.Status == MyShipConnectorStatus.Connected)
                        {
                            IGC.SendBroadcastMessage(_broadCastTag, "Closest bay is currently occupied.");
                            continue;
                        }

                        var myOrientation = baseConnector.WorldMatrix.Forward;
                        var myPosition = baseConnector.GetPosition();

                        // time to see if the docking ports are aligned on orientation
                        bool rotationAligned = Vector3D.Dot(myOrientation, otherPortOrientation) < -0.9;

                        if(!rotationAligned)
                        {
                            IGC.SendBroadcastMessage(_broadCastTag, "Unable to position due to docking ports at wrong angles.");
                            continue;
                        }

                        var adjustedToOrigin = otherPortPosition - myPosition;
                        var distanceApart = Vector3D.ProjectOnPlane(ref adjustedToOrigin, ref myOrientation).Length();
                        bool positionAligned = distanceApart < 1;

                        if (!baseReady)
                        { //dont have pistons to move the connector
                            continue;
                        }

                        // we received a valid docking request, so we need to check back and rerun the script in 100 ticks
                        Runtime.UpdateFrequency |= UpdateFrequency.Update100;

                        spinLight(positionAligned, rotationAligned);

                        string movementResult = moveBayToPos(closestBay, otherPortPosition);

                        IGC.SendBroadcastMessage(_broadCastTag, movementResult);
                    }
                }
            }
        }
    }
    catch (Exception e)
    {
        ExceptionHandler(e);
    }
}

private DockingBay findClosestBayToPosition(Vector3D otherPortPosition, List<DockingBay> dockingBays)
{
    if(dockingBays.Count < 1)
    {
        throw new Exception("No docking bays found.");
    }
    DockingBay closestBay = dockingBays.First();
    double closestDistance = 99999999999.99;
    foreach(DockingBay bay in dockingBays)
    {
        double distance = (bay.connector.GetPosition() - otherPortPosition).Length();
        if (distance < closestDistance)
        {
            closestBay = bay;
            closestDistance = distance;
        }
        if(distance < 10.0)
        {
            return bay; // early out if the closest bay is within 10 meters. (Maybe this is not a great plan?)
        }
    }
    return closestBay;
}

struct BaseAndMax
{
    public Vector3D basePos; // the location of the piston head when the piston is fully retracted
    public Vector3D maxPos;  // the location of the piston head when the piston is fully extended
    public Vector3D currentTop; // the current location of the top of the piston head
}

private string toGPS(Vector3D vector, string name = "", string color = "")
{
    if(name.Length == 0)
    {
        name = new Random().ToString();
    }

    return $"GPS:{name}:{vector.X}:{vector.Y}:{vector.Z}:{color}";
}

private void getPistonCustomDataMinMax(PistonMinMax minMax)
{
    IMyPistonBase piston = minMax.piston;
    string data = piston.CustomData;
    string[] lines = data.Split('\n');

    foreach(string line in lines)
    {
        if(line == null || line == "")
        {
            continue;
        }
        string[] keyVal = line.Split('=');
        if(keyVal.Length != 2)
        {
            Echo($"Current data is malformed on piston named {piston.CustomName}.\nCannot have keyVal length of {keyVal.Length}");
            continue;
        }

        string key = keyVal[0];
        float value = float.Parse(keyVal[1]);

        if(key == "min")
        {
            minMax.min = value;
        }
        else if(key == "max")
        {
            minMax.max = value;
        }
    }
}

private BaseAndMax getPistonBaseAndMax(IMyPistonBase piston)
{
    Vector3D directionFromBaseToTop = getUnitFromBaseToTop(piston);
    Vector3D pistonBase = piston.GetPosition() + Vector3D.Multiply(directionFromBaseToTop, 2.5);
    Vector3D pistonMax = pistonBase + Vector3D.Multiply(directionFromBaseToTop, 10.0);
    Vector3D pistionHeadCurrentPos = piston.Top.GetPosition() + directionFromBaseToTop * 1.25;

    var ret = new BaseAndMax();
    ret.basePos = pistonBase;
    ret.maxPos = pistonMax;
    ret.currentTop = pistionHeadCurrentPos;

    return ret;
}

private Vector3D fromBaseToMax(BaseAndMax baseAndMax)
{
    return Vector3D.Subtract(baseAndMax.maxPos, baseAndMax.basePos);
}

private Vector3D getUnitFromBaseToTop(IMyPistonBase piston)
{
    Vector3D pistonBase = piston.GetPosition();
    Vector3D fromBaseToTop = piston.Top.GetPosition() - pistonBase;
    return Vector3D.Normalize(fromBaseToTop);
}

private double getNeededExtent(Vector3D min, Vector3D max, Vector3D target)
{
    Vector3D diff = max - min; //vector from the tip of min to max (or parallel vector based at origin)

    Vector3D minProjected = Vector3D.ProjectOnVector(ref min, ref diff);
    Vector3D maxProjected = Vector3D.ProjectOnVector(ref max, ref diff);
    Vector3D targetProjected = Vector3D.ProjectOnVector(ref target, ref diff);

    double minPLength = minProjected.Length();
    double maxPLength = maxProjected.Length();
    double targetPLength = targetProjected.Length();
    Echo((maxProjected.Length() - minProjected.Length()).ToString()); // should be 10 or -10 uh oh, 10 always?!

    if (maxPLength < minPLength)
        return minPLength - targetPLength;

    if (targetPLength < minPLength)
        throw new Exception("Cannot retract piston into negative ranges.");

    if (targetPLength > maxPLength)
        throw new Exception("Cannot extend piston past 10.");

    return targetPLength - minPLength;
}

private void setPistonExtent(PistonMinMax pistonMinMax, float target)
{
    IMyPistonBase piston = pistonMinMax.piston;
    if(pistonMinMax.min > target || pistonMinMax.max < target)
    {
        Echo($"Cannot move piston named {piston.CustomName} due to custom data bounds.");
        return;
    }

    float current = piston.CurrentPosition;

    if(current > target)
    {
        piston.Velocity = -2.0f;
        piston.MinLimit = target;
    }
    else
    {
        piston.Velocity = 2.0f;
        piston.MaxLimit = target;
    }
}

private void stowBay(DockingBay bay, bool retractWhenStowing)
{
    float rootStowedExtent = retractWhenStowing ? bay.root.min : bay.root.max;
    float middleStowedExtent = retractWhenStowing ? bay.middle.min : bay.middle.max;
    float endStowedExtent = retractWhenStowing ? bay.end.min : bay.end.max;

    bay.stowed = true;
    setPistonExtent(bay.root, rootStowedExtent);
    setPistonExtent(bay.middle, middleStowedExtent);
    setPistonExtent(bay.end, endStowedExtent);
}

private string moveBayToPos(DockingBay bay, Vector3D targetPosition)
{
    bay.stowed = false;
    bay.busyTime = TimeSpan.FromSeconds(20.0);

    IMyPistonBase firstPiston = bay.root.piston;
    IMyPistonBase secondPiston = bay.middle.piston;
    IMyPistonBase thirdPiston = bay.end.piston;

    BaseAndMax verticalBaseAndMax = getPistonBaseAndMax(firstPiston);
    BaseAndMax lateralBaseAndMax = getPistonBaseAndMax(secondPiston);
    BaseAndMax depthBaseAndMax = getPistonBaseAndMax(thirdPiston);

    Vector3D vertDisplacement = fromBaseToMax(verticalBaseAndMax);
    Vector3D latDisplacement = fromBaseToMax(lateralBaseAndMax);
    Vector3D depDisplacement = fromBaseToMax(depthBaseAndMax);

    Debug($"vert {vertDisplacement.Length()}");
    Debug($"lat {latDisplacement.Length()}");
    Debug($"dep {depDisplacement.Length()}");

    Vector3D vertHeadToLatBase = Vector3D.Subtract(lateralBaseAndMax.basePos, verticalBaseAndMax.currentTop);
    Vector3D latHeadToDepBase = Vector3D.Subtract(depthBaseAndMax.basePos, lateralBaseAndMax.currentTop);

    Vector3D connectorPos = bay.connector.GetPosition() + Vector3D.Normalize(bay.connector.WorldMatrix.Forward) * 1.25;
    //@TODO this assumes the connector is attached to the depth piston and the depth is the outermost piston from the main grid
    Vector3D depHeadToConnector = Vector3D.Subtract(connectorPos, depthBaseAndMax.currentTop);

    Vector3D minimumConnectorPos = verticalBaseAndMax.basePos + vertHeadToLatBase + latHeadToDepBase + depHeadToConnector;
    Vector3D maximimConnectorPos = minimumConnectorPos + vertDisplacement + latDisplacement + depDisplacement;

    string result = "";

    List<Vector3D> connectorPosPoints = new List<Vector3D>();
    connectorPosPoints.Add(minimumConnectorPos);
    connectorPosPoints.Add(minimumConnectorPos + latDisplacement);
    connectorPosPoints.Add(minimumConnectorPos + depDisplacement);
    connectorPosPoints.Add(minimumConnectorPos + latDisplacement + depDisplacement);
    connectorPosPoints.Add(minimumConnectorPos + vertDisplacement);
    connectorPosPoints.Add(minimumConnectorPos + vertDisplacement + latDisplacement);
    connectorPosPoints.Add(minimumConnectorPos + vertDisplacement + depDisplacement);
    connectorPosPoints.Add(minimumConnectorPos + vertDisplacement + latDisplacement + depDisplacement);

    for (int i = 0; i < connectorPosPoints.Count; ++i)
    {
        Vector3D point = connectorPosPoints[i];
        Debug(toGPS(point, $"point{i}") + "\n");
    }

    // @TODO turn all of this into debug output
    Debug(toGPS(verticalBaseAndMax.basePos, "verticalPistonBase") + "\n");
    Debug(toGPS(verticalBaseAndMax.maxPos, "verticalPistonMax") + "\n");
    Debug(toGPS(lateralBaseAndMax.basePos, "lateralBase") + "\n");
    Debug(toGPS(lateralBaseAndMax.maxPos, "lateralMax") + "\n");
    Debug(toGPS(depthBaseAndMax.basePos, "depthBase") + "\n");
    Debug(toGPS(depthBaseAndMax.maxPos, "depthMax") + "\n");
    Debug(toGPS(connectorPos, "connectorPos") + "\n");
    Debug(toGPS(depthBaseAndMax.currentTop, "depthBaseAndMax.currentTop") + "\n");
    Debug(toGPS(targetPosition, "targetPosition") + "\n");

    try
    {
        double neededVerticalExtent = getNeededExtent(minimumConnectorPos, minimumConnectorPos + vertDisplacement, targetPosition);
        double neededLateralExtent = getNeededExtent(minimumConnectorPos, minimumConnectorPos + latDisplacement, targetPosition);
        double neededDepthExtent = getNeededExtent(minimumConnectorPos, minimumConnectorPos + depDisplacement, targetPosition);

        result += $"{neededVerticalExtent} neededVerticalExtent\n";
        result += $"{neededLateralExtent} neededLateralExtent\n";
        result += $"{neededDepthExtent} neededDepthExtent\n";

        if (neededVerticalExtent < bay.root.min || neededVerticalExtent > bay.root.max
            || neededLateralExtent < bay.middle.min || neededLateralExtent > bay.middle.max
            || neededDepthExtent < bay.end.min || neededDepthExtent > bay.end.max)
        {
            result += "Error: Docking port cannot make it to target because target is out of bounds (check custom data on pistons to verify bounds).";
            return result;
        }

        setPistonExtent(bay.root, (float)neededVerticalExtent);
        setPistonExtent(bay.middle, (float)neededLateralExtent);
        setPistonExtent(bay.end, (float)neededDepthExtent);
    }
    catch (Exception e)
    {
        result += e.Message + '\n';
    }
    return result;
}

private void spinLight(bool positionAligned, bool rotationAligned)
{
    if (light != null)
    {
        light.Enabled = rotationAligned;
        light.Color = positionAligned ? Color.Yellow : Color.Red;
    }
}

private void setupOutputSurface()
{
    List<IMyTextPanel> lcdScreens = new List<IMyTextPanel>();
    GridTerminalSystem.GetBlocksOfType(lcdScreens, screen => screen.CustomName.Contains(scriptTag));

    if (!Me.CustomName.Contains(ignoreTag))
    {
        outputScreens.Add(Me.GetSurface(0));
    }

    foreach(IMyTextSurface panel in lcdScreens)
    {
        if (panel != null)
        {
            outputScreens.Add(panel);
        }
    }

    // Fetch a log text panel
    if (outputScreens.Count < 1)
    {
        List<IMyShipController> cockpits = new List<IMyShipController>();
        GridTerminalSystem.GetBlocksOfType(cockpits, cockpit => cockpit is IMyTextSurfaceProvider);
        if (cockpits.Count > 0)
        {
            var mainCockpit = cockpits[0] as IMyTextSurfaceProvider;
            if (mainCockpit != null && !cockpits[0].CustomName.Contains(ignoreTag))
            {
                IMyTextSurface cockpitScreen = mainCockpit.GetSurface(0);
                if (cockpitScreen != null)
                {
                    outputScreens.Add(cockpitScreen);
                }
            }
        }
    }
}

public void Debug(string text)
{
    if (enableDebugOutput)
    {
        Echo(text);
    }
}

public void EchoToLCD(string text)
{
    bool append = true;
    foreach (IMyTextSurface screen in outputScreens)
    {
        if (screen == null)
        {
            continue;
        }
        screen.ContentType = ContentType.TEXT_AND_IMAGE;

        string screenText = screen.GetText();
        string[] lines = screenText.Split('\n');
        if(lines.Length > 15) 
        {
          string newText = screenText.Remove(0, lines[0].Length + 1);
          screen?.WriteText(newText, false);
        }
        screen?.WriteText($"{text}\n", append);
    }
}

public void ClearLCD()
{
    foreach (IMyTextSurface screen in outputScreens)
    {
        // Append the text and a newline to the logging LCD
        // A nice little C# trick here:
        // - The ?. after _logOutput means "call only if _logOutput is not null".
        //_logOutput?.WritePublicText($"{text}\n", true);
        screen.ContentType = ContentType.TEXT_AND_IMAGE;
        screen?.WriteText("", false);
    }
}

public void ExceptionHandler(Exception e)
{
    // Dump the exception content to the
    Echo("An error occurred during script execution.");
    Echo($"Exception: {e}\n---");

    // Rethrow the exception to make the programmable block halt execution properly
    throw e;
}
