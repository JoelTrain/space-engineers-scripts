// Rotor Automatic Docking [RAD]
// if you get an error message, try fixing the issue and recompiling the script

// you are welcome to change the variables in this section

string _broadCastTag = "Docking request channel 4A1E [RAD]";
string scriptTag = "[RAD]";    // only connectors on craft with this tag in the name will be used for docking
string ignoreTag = "[ignore]"; // script will ignore this block (currently only used for cockpits and programmable block screens)

// set this to false to stop receiving
bool enableDebugOutput = false;

bool enableStowing = true; // when true it will attempt to "stow" the connector into a safe place when not in use
bool retractWhenStowed = true; // pistons will fully retract to stow away when undocked. set to false to extend when stowed

// Connector offset
// The number of meters to offset the desired position from the center of the connector on the craft.
// Small Block Grid Connectors need a 0.75m offset
// Large Block Grid Connectors need a 1.5m offset
float smallConnectorOffset = 0.75f;
float largeConnectorOffset = 1.5f;
float connectorOffset = 0.75f;

float piF = (float)Math.PI;

float rotorRotationSpeed = (float)(Math.PI / 2 / 5); // this is in radians per second

float pistonSpeed = 2.0f;

// It provides a neat affect if the final piston with the connector on it moves a bit slower than the two that line it up, 
// and it can help avoid obstacles better if the craft connector is recessed in a pit surrounded by other components.
// float connectorPistonSpeed = 0.5f;

// private variables not meant to be changed
int _runcount = 0;
IMyBroadcastListener _myBroadcastListener;
UpdateType triggerUpdate = UpdateType.Trigger | UpdateType.Terminal | UpdateType.Mod | UpdateType.Script;
List<IMyTextSurface> outputScreens = new List<IMyTextSurface>();
bool baseReady = false;
IMyShipConnector craftConnector;
IMyTurretControlBlock turretControl;

List<IMyPistonBase> tempPistonList = new List<IMyPistonBase>();
List<IMyMotorAdvancedStator> tempRotorList = new List<IMyMotorAdvancedStator>();
List<IMyShipConnector> tempConnectorList = new List<IMyShipConnector>();
List<IMyTurretControlBlock> turretControlList = new List<IMyTurretControlBlock>();

class DockingBay
{
    public IMyPistonBase root;
    public IMyMotorAdvancedStator middle;
    public IMyMotorAdvancedStator end;
    public IMyShipConnector connector;
    public MyShipConnectorStatus lastStatus;
    public TimeSpan busyTime;
    public bool stowed;
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

        
        GridTerminalSystem.GetBlocksOfType(turretControlList, aturretControl => aturretControl.CustomName.Contains(scriptTag));
        if(turretControlList.Count > 0)
        {
          turretControl = turretControlList[0];
          Echo($"Custom Turret Controller found: {turretControl.CustomName}");
        }

        List<IMyPistonBase> pistonsDirectlyOnBase = new List<IMyPistonBase>();
        GridTerminalSystem.GetBlocksOfType(pistonsDirectlyOnBase, piston => piston.CubeGrid == Me.CubeGrid);

        foreach(IMyPistonBase piston in pistonsDirectlyOnBase)
        {
            DockingBay newPotentialBay = new DockingBay();
            newPotentialBay.stowed = false;
            newPotentialBay.busyTime = TimeSpan.Zero;
            newPotentialBay.root = piston;
            IMyCubeGrid middleGrid = piston.TopGrid;

            // Look for middle rotor
            tempPistonList.Clear();
            tempRotorList.Clear();
            GridTerminalSystem.GetBlocksOfType(tempRotorList, tempMiddleRotor => tempMiddleRotor.CubeGrid == middleGrid);
            if(tempRotorList.Count != 1)
            {
                continue;
            }
            newPotentialBay.middle = tempRotorList[0];
            IMyCubeGrid endGrid = newPotentialBay.middle.TopGrid;

            // Look for end rotor
            tempRotorList.Clear();
            GridTerminalSystem.GetBlocksOfType(tempRotorList, tempEndRotor => tempEndRotor.CubeGrid == endGrid);
            if (tempRotorList.Count != 1)
            {
                continue;
            }
            newPotentialBay.end = tempRotorList[0];

            // Look for connector on end
            tempConnectorList.Clear();
            GridTerminalSystem.GetBlocksOfType(tempConnectorList, connectorOnEnd => connectorOnEnd.CubeGrid == newPotentialBay.end.TopGrid);
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
            Echo("Enabling mode: craft/Ship");
            tempConnectorList.Clear();
            GridTerminalSystem.GetBlocksOfType(tempConnectorList);

            if(tempConnectorList.Count > 1)
            {
                GridTerminalSystem.GetBlocksOfType(tempConnectorList, acraftConnector => acraftConnector.CustomName.Contains(scriptTag));

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

            craftConnector = tempConnectorList[0];
            MyCubeSize gridSize = craftConnector.CubeGrid.GridSizeEnum;

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

        setRotorRotation(bays[0].middle, piF / 2);
        setRotorRotation(bays[0].end, piF / 2);
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
        _runcount++;

        if(_runcount > 10000)
        {
          _runcount = 0;
        }
        Debug(_runcount.ToString() + ":" + updateSource.ToString());

        if ((updateSource & triggerUpdate) > 0)
        {
            // ClearLCD();
            if (!String.IsNullOrEmpty(argument))
            {
                IGC.SendBroadcastMessage(_broadCastTag, argument);
                Echo("Sending message:\n" + argument);
            }

            if (craftConnector != null)
            {
                var portOrientation = Vector3D.Normalize(craftConnector.WorldMatrix.Forward);
                var portPosition = craftConnector.GetPosition() + (portOrientation * connectorOffset);
                string requestBody = $"Requesting automatic docking for {craftConnector.CubeGrid.CustomName}";
                var request = new MyTuple<string, string, Vector3D, Vector3D>("Docking Request", requestBody, portPosition, portOrientation);
                Echo("Sending docking request");
                Debug(toGPS(craftConnector.GetPosition(), "connectorPos"));
                Debug(toGPS(portPosition, "offsetPos"));
                IGC.SendBroadcastMessage(_broadCastTag, request);

            }
            else 
            {
                Runtime.UpdateFrequency |= (UpdateFrequency.Update100 | UpdateFrequency.Update1);
            }
        }
        // If the update source has this update flag, it means
        // that it's run from the frequency system, and we should
        // update our continuous logic.
        // if ((updateSource & UpdateType.Update100) != 0)
        if ((updateSource & UpdateType.Update1) != 0)
        {
            if(_runcount % 10000 == 0)
            {
                ClearLCD();
            }

						try
						{
							Vector3D playerPos = getPlayerPos();

							DockingBay closestBay = findClosestBayToPosition(playerPos, bays);

							// Echo(toGPS(playerPos));
							string movementResult = moveBayToPos(closestBay, playerPos);
							// Echo(movementResult);
						}
						catch (Exception e)
						{
							// Echo(e.Message + '\n');
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
            //ClearLCD();
            while (_myBroadcastListener.HasPendingMessage)
            {
                MyIGCMessage myIGCMessage = _myBroadcastListener.AcceptMessage();
                if (myIGCMessage.Tag == _broadCastTag)
                { // This is our tag
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
                        if (messageData.Item1 != "Docking Request")
                        {
                            continue;
                        }

                        var otherPortPosition = messageData.Item3;
                        var otherPortOrientation = messageData.Item4;

                        Debug("Received Docking request:");
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

private Vector3D getPlayerPos()
{
  if(turretControl == null) 
  {
    throw new Exception($"No Custom Turret Controller found with script tag {scriptTag}. Cannot find player.");
  }

  string controllerName = turretControl.CustomName;

  if(!turretControl.TargetFriends) 
  {
    throw new Exception($"Custom Turret controller {controllerName} not set to \"Target Friends\"");
  }

  if(!turretControl.TargetCharacters) 
  {
    throw new Exception($"Custom Turret controller {controllerName} not set to \"Target Characters\"");
  }

  MyDetectedEntityInfo detectedEntity = turretControl.GetTargetedEntity();

  if(detectedEntity.IsEmpty())
  {
    throw new Exception($"Custom Turret controller {controllerName} does not have a target.");
  }

  // Echo(detectedEntity.Name);

	// static Vector3D Transform(Vector3D position, MatrixD matrix)

	Vector3D playerUp = new Vector3D(0.0, 2.5, 0.0);

  return detectedEntity.Position + (Vector3D.Transform(playerUp, detectedEntity.Orientation));
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

private string toGPS(Vector3D vector, string name = "", string color = "FF75C9F1")
{
    if(name.Length == 0)
    {
        name = new Random().Next().ToString();
    }

    return $"GPS:{name}:{vector.X}:{vector.Y}:{vector.Z}:{color}:";
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
    double maxPLength = maxProjected.Length(); // should always be 10 if we are projecting max on the diff vector of which is a point.
    double targetPLength = targetProjected.Length();
    Debug($"Max extent from projection: {(maxProjected.Length() - minProjected.Length()).ToString()}"); // should always be 10 for a single piston


    if (targetPLength < minPLength)
			targetPLength = minPLength;

    if (targetPLength > maxPLength)
			targetPLength = maxPLength;

    if (maxPLength < minPLength)
        throw new Exception("Why are the min and max reversed for the needed piston extent?");

    // if (targetPLength < minPLength)
    //     throw new Exception("Cannot retract piston into negative ranges.");

    // if (targetPLength > maxPLength)
    //     throw new Exception("Cannot extend piston past 10.");

    return targetPLength - minPLength;
}

private void setRotorRotation(IMyMotorAdvancedStator rotor, float targetRadians)
{
  float current = rotor.Angle;

  if(targetRadians > current)
  {
    rotor.TargetVelocityRad = rotorRotationSpeed;
    rotor.UpperLimitRad = targetRadians;
  }
  else 
  {
    rotor.TargetVelocityRad = -1 * rotorRotationSpeed;
    rotor.LowerLimitRad = targetRadians;
  }
}

private void setPistonExtent(IMyPistonBase piston, float target)
{
    float current = piston.CurrentPosition;

    if(target > current)
    {
        piston.Velocity = pistonSpeed;
        piston.MaxLimit = target;
    }
    else
    {
        piston.Velocity = -1 * pistonSpeed;
        piston.MinLimit = target;
    }
}

private void stowBay(DockingBay bay, bool retractWhenStowing)
{
    if(bay.stowed)
    {
        return;
    }

    float stowedExtent = retractWhenStowing ? 0.0f : 10.0f;

    bay.stowed = true;
    setPistonExtent(bay.root, stowedExtent);
    // setPistonExtent(bay.middle, stowedExtent);
    // setPistonExtent(bay.end, stowedExtent);
    setRotorRotation(bay.middle, 0.0f);
    setRotorRotation(bay.end, 0.0f);
}

// get vector from base of piston to tip
// normalize this to get direction of normal vector for the target plane (not needed)
// get the position of the connector
// offset the position of the connector to the edge 
// project the edge point onto the direction vector
// this projected point is a normal vector with the correct magnitude to represent a plane in the disc
// project the player location onto the plane
// take the distance of that point from the center of the disc (project onto di)
// get the center of the disc by projecting the middle rotor onto the plane

private string moveBayToPos(DockingBay bay, Vector3D targetPosition)
{
    bay.stowed = false;
    bay.busyTime = TimeSpan.FromSeconds(20.0);

    IMyPistonBase firstPiston = bay.root;
    IMyMotorAdvancedStator middleRotor = bay.middle;
    IMyMotorAdvancedStator endRotor = bay.end;

    BaseAndMax verticalBaseAndMax = getPistonBaseAndMax(firstPiston);

    Vector3D vertDisplacement = fromBaseToMax(verticalBaseAndMax);
		Vector3D pistonDir = Vector3D.Normalize(vertDisplacement);

 		// in the direction from the base-end of the connector on the base toward the docking-port-end.
    Vector3D connectorDir = Vector3D.Normalize(bay.connector.WorldMatrix.Forward);

    Vector3D connectorPos = bay.connector.GetPosition() + connectorDir * largeConnectorOffset;

		// these are good
		Vector3D middleRotorPos = middleRotor.GetPosition();
		Debug(toGPS(middleRotorPos, "middleRotorPos"));
		Vector3D endRotorPos = endRotor.GetPosition();
		Debug(toGPS(endRotorPos, "endRotorPos"));

		Vector3D connectorDiscNormal = Vector3D.ProjectOnVector(ref connectorPos, ref pistonDir);
		Debug(toGPS(connectorDiscNormal, "connectorDiscNormal"));

		Vector3D targetOnPistonDir = Vector3D.ProjectOnVector(ref targetPosition, ref pistonDir);
		Vector3D targetToPlaneIntersect = Vector3D.Subtract(connectorDiscNormal, targetOnPistonDir);
		Vector3D targetProjectedOnDisc = Vector3D.Add(targetPosition, targetToPlaneIntersect);
		Debug(toGPS(targetProjectedOnDisc, "targetProjectedOnDisc"));

		Vector3D middleRotorOnPistonDir = Vector3D.ProjectOnVector(ref middleRotorPos, ref pistonDir);
		Vector3D centerOfDiscToPlane = Vector3D.Subtract(connectorDiscNormal, middleRotorOnPistonDir);
		Vector3D centerOfDisc = Vector3D.Add(middleRotorPos, centerOfDiscToPlane);
		Debug(toGPS(centerOfDisc, "centerOfDisc"));

		Vector3D endRotorOnPistonDir = Vector3D.ProjectOnVector(ref endRotorPos, ref connectorDiscNormal);
		Vector3D endRotorToPlane = Vector3D.Subtract(connectorDiscNormal, endRotorOnPistonDir);
		Vector3D endRotorOnDisc = Vector3D.Add(endRotorPos, endRotorToPlane);
		Debug(toGPS(endRotorOnDisc, "endRotorOnDisc"));

    Vector3D minimumConnectorVert = connectorPos - firstPiston.CurrentPosition;
		Vector3D rotorZeroDir = Vector3D.Normalize(middleRotor.WorldMatrix.Forward);

		
		Debug(toGPS(centerOfDisc + rotorZeroDir, "rotorZeroDir"));
		Vector3D middleRotorOnPlaneToTargetOnPlane = Vector3D.Subtract(targetProjectedOnDisc, centerOfDisc);
		Vector3D middleRotorOnPlaneToTargetOnPlaneUnit = Vector3D.Normalize(middleRotorOnPlaneToTargetOnPlane);
		// a dot b = ||a|| * ||b|| * cos(theta); where theta is the angle between them

		// the signed angle of vector a rotated to vector b where n is a unit vector normal to the plane containing a and b
		//  β = atan2((a cross b) dot n, a dot b)

		float angleRadsFromMiddleRotorToTarget;
		{
			float sin = Vector3.Dot(Vector3.Cross(middleRotorOnPlaneToTargetOnPlaneUnit, rotorZeroDir), middleRotor.WorldMatrix.Up); 
			float cos = Vector3.Dot(rotorZeroDir, middleRotorOnPlaneToTargetOnPlaneUnit);
			angleRadsFromMiddleRotorToTarget = (float)Math.Atan2(sin, cos);
		}


		Echo($"angle: {toDegrees(angleRadsFromMiddleRotorToTarget).ToString()}");

		// if(angleRadsFromMiddleRotorToTargetSine < 0)
		// 	angleRadsFromMiddleRotorToTarget *= -1;

		// alpha + x = theta1; 
		// where theta1 is the angle between the target and the middle rotor firward (when it is at 0 degrees)
		// x is the angle the middle rotor needs to be set to
		// alpha is the angle of the triangle centered at the middle rotor

		float theta1 = angleRadsFromMiddleRotorToTarget;
		Debug($"theta1: {toDegrees(theta1).ToString()}");

    string result = "";

    try
    {
        double neededVerticalExtent = getNeededExtent(minimumConnectorVert, minimumConnectorVert + vertDisplacement, targetPosition);
        result += $"{neededVerticalExtent} neededVerticalExtent\n";

				double middleRotorToEndRotorLength = Vector3D.Distance(centerOfDisc, endRotorOnDisc);
				double endRotorToConnectorLength = Vector3D.Distance(endRotorOnDisc, connectorPos);
				double radiusOfTargetFromDiscCenterLength = Vector3D.Distance(targetProjectedOnDisc, centerOfDisc);

				double lateralMinimum = middleRotorToEndRotorLength - endRotorToConnectorLength;
				double lateralMaximum = middleRotorToEndRotorLength + endRotorToConnectorLength;

				Debug(toGPS(verticalBaseAndMax.basePos, "verticalPistonBase"));
				Debug(toGPS(verticalBaseAndMax.maxPos, "verticalPistonMax"));
				Debug(toGPS(connectorPos, "connectorPos"));
				Debug(toGPS(targetPosition, "targetPosition"));
				Debug($"lateralMaximum: {lateralMaximum}");
				Debug($"middleRotorToEndRotorLength: {middleRotorToEndRotorLength}");

				double neededLateralExtent = radiusOfTargetFromDiscCenterLength;
        result += $"{neededLateralExtent} neededLateralExtent\n";

        if (neededVerticalExtent < 0 || neededVerticalExtent > 10
            || neededLateralExtent < lateralMinimum || neededLateralExtent > lateralMaximum)
        {
            result += "Error: Docking port cannot make it to target because target is out of bounds.\n";
            // return result;
        }

        if (neededLateralExtent < lateralMinimum || neededLateralExtent > lateralMaximum)
        {
            result += "Error: Docking port cannot make it to target because target is out of disc bound.";
						result += $"neededLateral: {neededLateralExtent}\n";
						result += $"lateralMinimum: {lateralMinimum}\n";
            return result;
        }

				// angles
				// alpha = middle rotor angle
				// beta = end rotor angle
				// gamma = no rotor here, angle connector makes back to middle rotor

				// calculate triagle sides
				// a = end rotor to connector (across from angle alpha)
				// b = target on plane to middle rotor (across from angle beta)
				// c = middle rotor to end rotor (across from angle gamma)

				double a = endRotorToConnectorLength;
				double b = radiusOfTargetFromDiscCenterLength;
				double c = middleRotorToEndRotorLength;
				Debug($"a: {a.ToString()}");
				Debug($"b: {b.ToString()}");
				Debug($"c: {c.ToString()}");

				double cosa = ((b * b) + (c * c) - (a * a)) / (2 * b * c);
				// Echo($"cos(alpha): {cosa.ToString()}");

				float alpha = MathHelper.MonotonicAcos(2.0f - (float)cosa);
				Debug($"alpha ANGLE middle rotor degrees: {toDegrees(alpha).ToString()}");
				float beta = MathHelper.MonotonicAcos(2.0f - (float)(((a * a) + (c * c) - (b * b)) / (2 * a * c)));
				Debug($"beta ANGLE end rotor degrees: {toDegrees(beta).ToString()}");
				float gamma = MathHelper.MonotonicAcos(2.0f - (float)(((a * a) + (b * b) - (c * c)) / (2 * a * b)));
				Debug($"gamma ANGLE target to middle: {toDegrees(gamma).ToString()}");

				if(Math.Pow(piF - (alpha + beta + gamma), 2) > 0.0003f)
					throw new Exception((alpha + beta + gamma).ToString() + " angles do not add up to Pi radians");

				double middleRotorAngleRadians = theta1 - alpha;
				double endRotorAngleRadians = beta;

        setPistonExtent(bay.root, (float)neededVerticalExtent);
				setRotorRotation(bay.middle, (float)(middleRotorAngleRadians));
				setRotorRotation(bay.end, (float)endRotorAngleRadians);
    }
    catch (Exception e)
    {
        result += e.Message + '\n';
    }
    return result;
}

private float toDegrees(float radians) {
	return radians * 180.0f / piF;
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

				string[] inputLines = text.Split('\n');
				int inputCount = inputLines.Length;

        string screenText = screen.GetText();
        string[] lines = screenText.Split('\n');
				int existingLineCount = lines.Length;
        if(existingLineCount > 15)
				{ 
					StringBuilder output = new StringBuilder();
					// ClearLCD();
					for (int i = 0; i < inputCount; i++)
					{
						output.Append(inputLines[i] + '\n');
					}
					for (int i = 0; i + inputCount < 15; i++)
					{
						output.Append(lines[i] + '\n');
					}
					text = output.ToString();
					screen?.WriteText($"{text}\n", false);
				}
				else
				{
					screen?.WriteText($"{text}\n", append);
				}
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
