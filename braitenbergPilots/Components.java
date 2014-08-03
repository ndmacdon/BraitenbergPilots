package braitenbergPilots;
/****
 *
 * Project:     Flotsam Braitenberg Vehicles
 * 
 * Author:      Nicholas Macdonald
 *
 * Date:        January 16, 2014
 * 
 * File:        Components.java
 *
 * Description: A series of robots implemented inside of Asteroids. 
 *              The robots are modeled after those described in Valentino 
 *              Braitenberg's Vehicles. They are equipped with; sensors which 
 *              are excited by various qualities of the environment, Modulator 
 *              devices which modulate the raw output of sensors, and 'motors' 
 *              which have various effects on the robot and its environment.
 ****/

import java.util.*;
import java.util.List;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.geom.util.AffineTransformation;

/*
  Define devices which are sources or receivers of signals:
*/
interface Source    { double getOutput(); }
interface Receiver  { void input(double input); }


/*
  Initializes and returns sensors of various types:
*/
class SensorFactory {
  
  // Return a sensor with a circular detection area with radius <r>
  public static DistanceSensor makeDistanceRadiusSensor(double r) {
    // Create a sensor and assign a shape to it.
    DistanceSensor sensor = new DistanceSensor();
    SingleGeometryFactory.getShapeFactory().setSize(r*2);
    SingleGeometryFactory.getShapeFactory().setBase(new Coordinate(
        -r, 
        -r));
    
    Polygon circle = new PolyWrapper(SingleGeometryFactory.getShapeFactory().createCircle());
    sensor.localShape = circle;
    sensor.shapeSize = r;
    
    return sensor;
  }
  
  // Return a sensor with a ray detection area with length <l>
  public static DistanceSensor makeDistanceRaySensor(double l) {
    // Create a sensor and assign a shape to it.
    DistanceSensor sensor = new DistanceSensor();
    
    // Generate the points of our detection line:
    Coordinate[] coords = new Coordinate[4];
    coords[0] = new Coordinate(0, 0);   // Start point.
    coords[1] = new Coordinate(0, -l);  // End point.
    coords[2] = new Coordinate(0, -l/2);// Midpoint (satisfies polygon requirements)
    coords[3] = new Coordinate(0, 0);   // Close the polygon.
    
    Polygon line = SingleGeometryFactory.getFactory().createPolygon(coords);
    sensor.localShape = line;
    sensor.shapeSize = l;
    
    return sensor;
  }
  
  // Return a sensor with a cone detection area described by <l, theta>
  public static DistanceSensor makeDistanceConeSensor(double l, double theta) {
    // Create a sensor and assign a shape to it.
    DistanceSensor sensor = new DistanceSensor();
    SingleGeometryFactory.getShapeFactory().setSize(l*2);
    SingleGeometryFactory.getShapeFactory().setBase(new Coordinate(-l, -l));

    Polygon cone = new PolyWrapper(SingleGeometryFactory.getShapeFactory().createArcPolygon(-(Math.PI+theta)/2, theta));
    sensor.localShape = cone;
    sensor.shapeSize = l;
    
    return sensor;
  }
  
  // Return a sensor:
  public static ShipSpeedSensor makeShipSpeedSensor() {
    // Create a sensor.
    ShipSpeedSensor sensor = new ShipSpeedSensor();
    SingleGeometryFactory.getShapeFactory().setSize(4);
    SingleGeometryFactory.getShapeFactory().setBase(new Coordinate(0,0));
    
    Polygon circle = new PolyWrapper(SingleGeometryFactory.getShapeFactory().createCircle());
    sensor.localShape = circle;
    sensor.maxSpeed = Asteroids.MAX_SHIP_SPEED; // Set the speed limit.
    return sensor;
  }
  
}


/*
  A sensor which analyzes the GameState and returns a Double <output> which is
  some measure of the environment. The nature of <output> depends on the implementation
  of the particular sensor.
*/
abstract class Sensor {
  double output = 0.0;
  PolyWrapper worldShape;
  Polygon localShape;
  
  // Sense some aspect of the GameState <world> given the sensor is at:
  // <loc> with a heading of <rot>.
  abstract void sense(Point loc, double rot, GameState world);
  
  // Return the AsteroidsSprite which intersects worldShape and is nearest to <loc>:
  public AsteroidsSprite getNearestIntersected(Point loc, double rot, GameState world) {
    
    // Create worldTransform to transform to the local-space shape to world-space:
    AffineTransformation trans = new AffineTransformation();
    trans.rotate(-rot);
    trans.translate(loc.getX(), loc.getY());
    worldShape = new PolyWrapper((Polygon)trans.transform(localShape));
    
    // List of AsteroidsSprites that intersect with a circle described by
    // <loc, distanceToNearest>
    List<AsteroidsSprite> intersected = world.intersects(worldShape);
    
    // Nearest AsteroidsSprite:
    AsteroidsSprite nearest = AsteroidsSprite.nearest(loc, intersected);

    return nearest;
  }
  
  // Return a polygon in the shape of the detection field of the sensor:
  PolyWrapper getWorldShape() { return worldShape; }
  public double getOutput() { return output; }
}


/*
  Detects all colliders which intersect a shape described by worldShape.
  
  WorldShape is a generated from transforming localShape to a location
  indicated by <loc, rot>.
  
  Assigns <output> the complement of the distance to the nearest intersected 
  collider as a fraction of <detectionRadius>.
  i.e. <output> = 1 - (<distanceToNearest> / <detectionRadius>)
  This gives:
    ~one when objects are exactly on the sensor
    ~values close to one when objects are very near
    ~values close to zero when objects are very far
    ~zero when all objects are outside of <detectionRadius>
*/
class DistanceSensor extends Sensor {
  double shapeSize; // Upper limit on distance.
  
  // Sense some aspect of the GameState <world> given the sensor is at:
  // <loc> with a heading of <rot>.
  public void sense(Point loc, double rot, GameState world) {
    AsteroidsSprite nearest = this.getNearestIntersected(loc, rot, world);
    double result = 0.0f;
    
    // Calculate result if something was detected:
    if (nearest != null) {
      double distanceToNearest = loc.distance(nearest.sprite); 
      double fraction = distanceToNearest / shapeSize;
      result = (fraction > 1.0) ? 0.0 : 1.0-fraction;
    }

    output = result;
  }
}


/*
  Detects all colliders intersecting a line described by <loc, orientation>.
  Assigns <output> the complement of the speed to the nearest intersected collider 
  as a fraction of <baseSpeed>.
  i.e. <output> = 1 - (<speed> / <maxSpeed>)
  This gives:
    ~one when objects are moving at speed >= <maxSpeed>
    ~values close to one when objects are moving at nearly <maxSpeed>
    ~values close to zero when objects are moving at nearly zero
    ~zero when all objects are outside of detection beam
*/
class SpeedSensor extends Sensor {
  double maxSpeed; // Upper limit on speed.
  
  public void sense(Point loc, double rot, GameState world) {
    double result = 0.0f;
    AsteroidsSprite nearest = this.getNearestIntersected(loc, rot, world);
    
    // Calculate result if something was detected:
    if (nearest != null) {
      // Normalize the speed of <nearest>:
      double speed = nearest.getSpeed();
      double fraction = speed / maxSpeed;
      if (fraction > 1.0) { fraction = 1.0; }
      if (fraction < 0) { fraction = 0; }
      output = fraction;
    }

    output = result;
  }
}

/*
  Detects the ships current speed.
  Assigns <output> the normalized speed of the ship.
  i.e. <output> = 1 - (<speed> / <baseSpeed>)
  This gives:
    ~one when ship is moving at speed >= <maxSpeed>
    ~values close to one when ship is moving at nearly <maxSpeed>
    ~values close to zero when ship is moving slowly
    ~zero when ship is stationary
*/
class ShipSpeedSensor extends Sensor {
  double maxSpeed; // Upper limit on speed.
  
  public void sense(Point loc, double rot, GameState world) {
    this.getNearestIntersected(loc, rot, world);
    double speed = world.ship.getSpeed();
    double fraction = speed / maxSpeed;
    if (fraction > 1.0) { fraction = 1.0; }
    if (fraction < 0) { fraction = 0; }
    output = fraction;
  }
}


/*
  Defines a modulation device with double input and signal:
  Inputs are accumulated and fed to a function which sets the signal value.
  Resetting the Modulator discharges any accumulated input.
  The signal of an implementation results from an arbitrary function of the 
  accumulated input.
*/
interface Modulator extends Source, Receiver {
  double MAX_OUTPUT = 1.0;
  public void reset();            // Reset the accumulated input.
  public Modulator copy();
}

/*
  Returns output equal to accumulated <input>:
*/
class ModulatorIdentity implements Modulator {
  double charge;
  double output;

  // Constructor:
  public ModulatorIdentity() {
    charge = 0;
    output = 0;
  }

  // Returns a new instance of this class:
  public Modulator copy() { return new ModulatorIdentity(); }

  // Copy Constructor:
  public ModulatorIdentity(ModulatorIdentity original) {
    charge = original.charge;
    output = original.charge;
  }

  // Accumulate input outputs:
  public void input(double input) {
    charge += input;
  }

  // Return the output of the accumulated charge:
  // (this very simple Modulator returns output = charge)
  public double getOutput() {
    output = charge;
    // Upper and lower bounds for output:
    output = (output > MAX_OUTPUT) ? MAX_OUTPUT : output;
    output = (output < -MAX_OUTPUT) ? -MAX_OUTPUT : output;
    return output;
  }

  // Discharge any accumulated input:
  public void reset() {
    charge = 0;
    output = 0;
  }
}

/*
  Returns MAX_OUTPUT if accumulated input does not equal 0:
*/
class ModulatorBinary extends ModulatorIdentity {
  // Returns a new instance of this class:
  public Modulator copy() { return new ModulatorBinary(); }

  // Return the output of the accumulated charge:
  public double getOutput() {
    // Clamp output between +-MAX_OUTPUT:
    output = (charge > 0) ? MAX_OUTPUT : 0;
    return output;
  }
}

/*
  Returns <charge>^2:
*/
class ModulatorSquared extends ModulatorIdentity {
  // Returns a new instance of this class:
  public Modulator copy() { return new ModulatorSquared(); }

  // Return the output of the accumulated charge:
  public double getOutput() {
    output = Math.pow(charge, 2);
    output = (output > MAX_OUTPUT) ? MAX_OUTPUT : output;
    return output;
  }
}

/*
  Returns <charge>^.5:
*/
class ModulatorSqrt extends ModulatorIdentity {
  // Returns a new instance of this class:
  public Modulator copy() { return new ModulatorSqrt(); }

  // Return the output of the accumulated charge:
  public double getOutput() {
    output = Math.sqrt(charge);
    output = (output > MAX_OUTPUT) ? MAX_OUTPUT : output;
    return output;
  }
}

/*
  Models an inverted parabola with:
  - f(0) = 0
  - f(.5) = y-max = 1
  - f(1) = 0
*/
class ModulatorParabolic extends ModulatorIdentity {
  // Returns a new instance of this class:
  public Modulator copy() { return new ModulatorParabolic(); }

  // Return the output of the accumulated charge:
  public double getOutput() {
    output = -4*(Math.pow(charge, 2)-charge) - .25;
    //output = -16*(Math.pow(charge, 2)-charge) - 3;
    // Clamp output between +-MAX_OUTPUT:
    output = (output > MAX_OUTPUT) ? MAX_OUTPUT : output;
    output = (output < 0) ? 0 : output;
    return output;
  }
}



/*
Models a function where:
- f(x) = ( (x - .5)^3 ) * 8
- f(0)  = -1
- f(.5) = 0
- f(1)  = 1
*/
class ModulatorPolynomial extends ModulatorIdentity {
// Returns a new instance of this class:
public Modulator copy() { return new ModulatorParabolic(); }

// Return the output of the accumulated charge:
public double getOutput() {
  output = Math.pow(charge-.5f, 3) * 8;
  
  // Clamp output between +-MAX_OUTPUT:
  output = (output > MAX_OUTPUT) ? MAX_OUTPUT : output;
  output = (output < -MAX_OUTPUT) ? -MAX_OUTPUT : output;
  return output;
}
}

/*
  Registers a connection between an output device and an input device:
  If a wire is an inhibiter it returns the reciprocal of any signal passing through.
*/
class Wire {
  int source;
  int destination;
  float scale;
  boolean inhibitor;

  // Constructor:
  public Wire(int source, int destination, boolean inhibitor, float scale) {
    this.source = source;
    this.destination = destination;
    this.scale = scale;
    this.inhibitor = inhibitor;
  }

  // Copy Constructor:
  public Wire(Wire original) {
    source = original.source;
    destination = original.source;
    inhibitor = original.inhibitor;
  }
}


/*
  Represents a 'sensor-socket' on a vehicle.
  Hardpoints may be attached to vehicles.
  Sensors added to Hardpoints may be queried.
*/
class Hardpoint implements Source{
  Vector2D locationOffset;   // Hardpoint location relative to vehicle's origin.
  double rotationOffset;  // Hardpoint rotation relative to vehicle's orientation.
  Sensor sensor;      // Sensor attached to this Hardpoint.
  double output = 0.0;

  // Constructor:
  public Hardpoint(Vector2D loc, double rot) {
    locationOffset = loc;
    rotationOffset = rot;
  }

  // Copy Constructor:
  public Hardpoint(Hardpoint original) {
    locationOffset = original.locationOffset;
    rotationOffset = original.rotationOffset;
    sensor = original.sensor;
  }

  // Attach a sensor to this Hardpoint:
  void addSensor(Sensor s) {

    sensor = s;
  }

  // Activate the 'attached' sensor and return the result:
  void sense(GameState world, Point loc, double orientation) {
    // Get the location of this hardpoint in world-space:
    Point worldLocation = getWorldLocation(loc, orientation);
    double result = 0.0;

    if (sensor != null) {
      sensor.sense(worldLocation, orientation + rotationOffset, world);
      result = sensor.getOutput();
    }

    output = result;
  }

  public double getOutput() {
    return output;
  }

  // Get the offset of this hardpoint in world-space:
  Point getWorldLocation(Point loc, double rot) {
    // Find the xComponent of the world-translation:
    Coordinate xComponent = new Coordinate(
      Math.cos(rot) * locationOffset.getX(),
      -Math.sin(rot) * locationOffset.getX()
      );

    // Find the yComponent of the world-translation:
    Coordinate yComponent = new Coordinate(
      Math.sin(rot) * locationOffset.getY(),
      Math.cos(rot) * locationOffset.getY()
      );

    // Add the original <loc> and the translation components:
    Coordinate translatedOffset = new Coordinate(
        xComponent.x + yComponent.x + loc.getX(),
        xComponent.y + yComponent.y + loc.getY());
    
    return SingleGeometryFactory.getFactory().createPoint(translatedOffset);
  }
}

/*
  Provides the basic facilities necessary for a Braitenberg Vehicle.
  Can sense the gamestate and generate command signals.
*/
class BraitenbergVehicle {
  long lifetime;      // Number of updates this agent has survived.
  GameState world = null; // Gamestate representing this agent's environment.

  // Control signals generated by the vehicle
  double[] controlSignals;

  // This vehicle's Hardpoints equipped with sensors:
  // Used to 'sense' some quality of <world>.
  // Results of sensing are output to Modulators.
  List<Hardpoint> hardpoints = new ArrayList<Hardpoint>();

  // This vehicle's double-Modulator devices:
  // Receive input from sensors, modulates that input and outputs the result.
  List<Modulator> modulators = new ArrayList<Modulator>();

  // Registers a connection between a: (sensor, Modulator) or 
  // (Modulator, controlSignal) pair.
  // The first element of the tuple produces an output and this is used as input
  // to the second element.
  List<Wire> sensorWires = new ArrayList<Wire>();
  List<Wire> controlWires = new ArrayList<Wire>();

  // Constructor:
  public BraitenbergVehicle(GameState world) {
    this.lifetime = 0;
    this.world = world;
    controlSignals = new double[]{0,0,0,0,0,0};
  }

  // Copy constructor:
  public BraitenbergVehicle(BraitenbergVehicle original) {
    this.lifetime = 0;
    this.world = original.world;
    controlSignals = new double[]{0,0,0,0,0,0};

    // Copy Hardpoints:
    for (Hardpoint h : original.hardpoints) {
      hardpoints.add(new Hardpoint(h));
    }

    // Copy Modulators:
    for (Modulator dh : original.modulators) {
      modulators.add(dh.copy());
    }

    // Copy SensorWires:
    for (Wire sw : original.sensorWires) {
      sensorWires.add(new Wire(sw));
    }

    // Copy ControlWires:
    for (Wire cw : original.controlWires) {
      controlWires.add(new Wire(cw));
    }
  }

  // Prime each sensor with output:
  void sense() {
    AsteroidsSprite ship = world.getShip();
    
    // Sense with each Hardpoint:
    for (Hardpoint cur : hardpoints) {
      cur.sense(world, ship.getLocation(), ship.angle );
    }
  }

  // Query a list of Sources and use their output as input to a list of sources.
  // <Source, Destination> pairings are indicated by <wires>.
  void process(List<? extends Source> sourceList, 
    List<? extends Receiver> destinationList,
    List<Wire> wires) {
    
    // Read the source from <wire> and apply it to <wire>'s destination
    for (Wire wire : wires) {
      // Get the source:
      Source source = sourceList.get(wire.source);
      double output = wire.scale*source.getOutput();

      // Get the destination:
      Receiver destination = destinationList.get(wire.destination);

      // If <wire> is an inhibitor, negate the output:
      output = (wire.inhibitor) ? 1.0-output : output;
      output = (output < 0) ? 0 : output;

      // Apply source's output to the destination:
      destination.input(output);
    }
  }

  // Read data from Modulators and enter it into control-signals
  void signal() {
    
    // Query a Modulator indicated by the sensorWire <wire> and use the result as 
    // input to the Modulator also indicated by <wire>:
    for (Wire wire : controlWires) {
      Modulator source = modulators.get(wire.source); // Get the source.
      double output = wire.scale*source.getOutput(); // Get the source's output.

      // If wire <wire> is an inhibitor, invert the result:
      output = (wire.inhibitor) ? 1.0-output : output;
      output = (output < 0) ? 0 : output;

      // Apply source's output to the destination:
      controlSignals[wire.destination] += output;
    }
  }

  // Reset all controlSignals and Modulator accumulated-charges:
  // Avoids seizure-states wherein the vehicle is over-stimulated.
  void relax() {
    for (Modulator t : modulators) {
      t.reset();
    }

    // Reset control signals to 0:
    for (int i = 0; i < controlSignals.length; i++) {
      controlSignals[i] = 0;
    }
  }


  // End the life of this vehicle and return its lifetime:
  public long expire() {
    long preLifetime = lifetime;
    relax();
    lifetime = 0;
    return preLifetime;
  }

  // Analyze the gamestate and generate commands:
  void update() {
    lifetime++;
    relax();
    sense();
    process(hardpoints, modulators, sensorWires);
    signal();
  }

  // Accessors:
  long   getLifetime()     { return lifetime; }
  double getTurnLeft()     { return controlSignals[0]; }
  double getTurnRight()    { return controlSignals[1]; }
  double getAccForward()   { return controlSignals[2]; }
  double getAccBackward()  { return controlSignals[3]; }
  double getFirePhoton()   { return controlSignals[4]; }
  double getFireHyper()    { return controlSignals[5]; }
}


/*
  BraitenbergVehicleFactory
  Creates Braitenberg Vehicles from components:
*/
class BRVFactory {
  static final double SHORT_LENGTH = 200.0;
  static final double  MEDIUM_LENGTH = 300.0;
  static final double  LONG_LENGTH = 2000.0;
  static final double  SLOW_SPEED = 5.0;
  static final double HALF_PI = Math.PI / 2;
  static final double QUARTER_PI = Math.PI / 4;
  

  static final int LEFT    = 0;
  static final int RIGHT   = 1;
  static final int FORWARD = 2;
  static final int BACK    = 3;
  static final int SHOOT   = 4;
  static final int JUMP    = 5;
  
  
  // A pile of sensors we may draw from to equip a Hardpoint:
  List<Sensor> sensorPile = new ArrayList<Sensor>();

  // A pile of Hardpoints we may draw from to equip a BraitenbergVehicle:
  List<Hardpoint> hardpointPile = new ArrayList<Hardpoint>();

  // A pile of Modulators we may draw from to equip a BraitenbergVehicle:
  List<Modulator> ModulatorPile = new ArrayList<Modulator>();

  // List of Braitenberg Vehicles equipped with hardpoints but no sensors or
  // wires.
  // Serves as the base for a 'model' or 'line' of similar vehicles.
  List<BraitenbergVehicle> chassisPile = new ArrayList<BraitenbergVehicle>();
  
  

  GameState world;
  
  // Constructor:
  public BRVFactory(GameState world) {
    this.world = world;

    // Supply the sensor pile:
    sensorPile.add(SensorFactory.makeDistanceConeSensor(SHORT_LENGTH, Math.PI/6));

    // Supply the ModulatorPile:
    ModulatorPile.add(new ModulatorIdentity());
    ModulatorPile.add(new ModulatorParabolic());

    // Supply the chassis pile:
    chassisPile.add(makeStarfishChassis());
  }
  
  // Populate the participants for a tournament:
  List<BraitenbergVehicle> makeTourneyRoster() {
    // Vehicles included in the tournament:
    List<BraitenbergVehicle> tourneyRoster = new ArrayList<BraitenbergVehicle>();
    
    tourneyRoster.add(makeVehicleReverseCone());
    tourneyRoster.add(makeVehicleStationary());
    tourneyRoster.add(makeVehiclePolarRegions());
    tourneyRoster.add(makeVehicleRayEye());
    
    return tourneyRoster;
  }

  BraitenbergVehicle makeVehicleOneRay () {
      BraitenbergVehicle v = new BraitenbergVehicle(world);
      
      // Hardpoint on the Vehicle's Nose:
      Hardpoint nosePoint = new Hardpoint(new Vector2D(0,-10), 0.0);

      // Add a sensor to the nose hardpoint:
      nosePoint.addSensor(SensorFactory.makeDistanceRaySensor(SHORT_LENGTH));

      // Add the hardpoint to the ship:
      v.hardpoints.add(nosePoint);

      // Add a Modulator to the vehicle:
      v.modulators.add(new ModulatorIdentity());

      // Wire the hardpoint to the Modulator:
      v.sensorWires.add(new Wire(0,0, false, 1.0f));

      // Wire the modulator to a Control Signal:
      v.controlWires.add(new Wire(0, FORWARD, false, 1.0f));
      
      return v;
  }
  
  BraitenbergVehicle makeVehicleSensorTestRadius() {
    BraitenbergVehicle v = new BraitenbergVehicle(world);
    
    // Hardpoint on the Vehicle's Nose:
    Hardpoint nosePoint = new Hardpoint(new Vector2D(0,-10), 0.0);

    // Add a sensor to the nose hardpoint:
    nosePoint.addSensor(SensorFactory.makeDistanceRadiusSensor(SHORT_LENGTH));

    // Add the hardpoint to the ship:
    v.hardpoints.add(nosePoint);
    
    return v;
}
  
  BraitenbergVehicle makeVehicleOneRadius() {
      BraitenbergVehicle v = new BraitenbergVehicle(world);
      
      // Hardpoint on the Vehicle's Nose:
      Hardpoint nosePoint = new Hardpoint(new Vector2D(0,-10), 0.0);

      // Add a sensor to the nose hardpoint:
      nosePoint.addSensor(SensorFactory.makeDistanceRadiusSensor(SHORT_LENGTH));

      // Add the hardpoint to the ship:
      v.hardpoints.add(nosePoint);

      // Add a Modulator to the vehicle:
      v.modulators.add(new ModulatorIdentity());

      // Wire the hardpoint to the Modulator:
      v.sensorWires.add(new Wire(0,0, false, 1.0f));

      // Wire the modulator to a Control Signal:
      v.controlWires.add(new Wire(0, FORWARD, false, 1.0f));
      
      return v;
  }
  
  BraitenbergVehicle makeVehicleOneCone() {
      BraitenbergVehicle v = new BraitenbergVehicle(world);
      
      // Hardpoint on the Vehicle's Nose:
      Hardpoint nosePoint = new Hardpoint(new Vector2D(0,-10), 0.0);

      // Add a sensor to the nose hardpoint:
      nosePoint.addSensor(SensorFactory.makeDistanceConeSensor(SHORT_LENGTH, Math.PI/6));

      // Add the hardpoint to the ship:
      v.hardpoints.add(nosePoint);

      // Add a Modulator to the vehicle:
      v.modulators.add(new ModulatorIdentity());

      // Wire the hardpoint to the Modulator:
      v.sensorWires.add(new Wire(0,0, false, 1.0f));

      // Wire the modulator to a Control Signal:
      v.controlWires.add(new Wire(0,LEFT, false, 1.0f));
      
      return v;
  }
  
  BraitenbergVehicle makeVehicleTwoARadius () {
      BraitenbergVehicle v = new BraitenbergVehicle(world);
      
      // Hardpoint on the Vehicle's Nose:
      Hardpoint leftNosePoint = new Hardpoint(new Vector2D(-25,-10), 0.0);
      Hardpoint rightNosePoint = new Hardpoint(new Vector2D(25,-10), 0.0);

      // Add a sensor to the nose hardpoints:
      leftNosePoint.addSensor(SensorFactory.makeDistanceRadiusSensor(SHORT_LENGTH));
      rightNosePoint.addSensor(SensorFactory.makeDistanceRadiusSensor(SHORT_LENGTH));

      // Add the hardpoint to the ship:
      v.hardpoints.add(leftNosePoint);
      v.hardpoints.add(rightNosePoint);

      // Add a Modulator to the vehicle:
      v.modulators.add(new ModulatorIdentity());
      v.modulators.add(new ModulatorIdentity());

      // Wire the hardpoints to Modulators:
      v.sensorWires.add(new Wire(0,0, false, 1.0f));
      v.sensorWires.add(new Wire(1,1, false, 1.0f));

      // Wire the modulator to a Control Signal:

      v.controlWires.add(new Wire(0,FORWARD, false, 1.0f));
      v.controlWires.add(new Wire(0,RIGHT,   false, 1.0f));
      v.controlWires.add(new Wire(1,FORWARD, false, 1.0f));
      v.controlWires.add(new Wire(1,LEFT,    false, 1.0f));
      
      return v;
  }
  
  BraitenbergVehicle makeVehicleTwoACone () {
      BraitenbergVehicle v = new BraitenbergVehicle(world);

      // Hardpoint on the Vehicle's Nose:
      Hardpoint leftNosePoint = new Hardpoint(new Vector2D(0,-10), Math.PI/6);
      Hardpoint rightNosePoint = new Hardpoint(new Vector2D(0,-10), -Math.PI/6);

      // Add a sensor to the nose hardpoints:
      leftNosePoint.addSensor(SensorFactory.makeDistanceConeSensor(SHORT_LENGTH, Math.PI/6));
      rightNosePoint.addSensor(SensorFactory.makeDistanceConeSensor(SHORT_LENGTH, Math.PI/6));

      // Add the hardpoint to the ship:
      v.hardpoints.add(leftNosePoint);
      v.hardpoints.add(rightNosePoint);

      // Add a Modulator to the vehicle:
      v.modulators.add(new ModulatorIdentity());
      v.modulators.add(new ModulatorIdentity());

      // Wire the hardpoints to Modulators:
      v.sensorWires.add(new Wire(0,0, false, 1.0f));
      v.sensorWires.add(new Wire(1,1, false, 1.0f));

      // Wire the modulator to a Control Signal:
      v.controlWires.add(new Wire(0,FORWARD, false, 1.0f));
      v.controlWires.add(new Wire(0,RIGHT,   false, 1.0f));
      v.controlWires.add(new Wire(1,FORWARD, false, 1.0f));
      v.controlWires.add(new Wire(1,LEFT,    false, 1.0f));
      
      return v;
  }
  
  BraitenbergVehicle makeVehicleTwoBRadius () {
      BraitenbergVehicle v = new BraitenbergVehicle(world);

      // Hardpoint on the Vehicle's Nose:
      Hardpoint leftNosePoint = new Hardpoint(new Vector2D(-25,-10), 0.0);
      Hardpoint rightNosePoint = new Hardpoint(new Vector2D(25,-10), 0.0);

      // Add a sensor to the nose hardpoints:
      leftNosePoint.addSensor(SensorFactory.makeDistanceRadiusSensor(SHORT_LENGTH));
      rightNosePoint.addSensor(SensorFactory.makeDistanceRadiusSensor(SHORT_LENGTH));

      // Add the hardpoint to the ship:
      v.hardpoints.add(leftNosePoint);
      v.hardpoints.add(rightNosePoint);

      // Add a Modulator to the vehicle:
      v.modulators.add(new ModulatorIdentity());
      v.modulators.add(new ModulatorIdentity());

      // Wire the hardpoints to Modulators:
      v.sensorWires.add(new Wire(0,0, false, 1.0f));
      v.sensorWires.add(new Wire(1,1, false, 1.0f));

      // Wire the modulator to a Control Signal:
      v.controlWires.add(new Wire(0,FORWARD, false, 1.0f));
      v.controlWires.add(new Wire(0,LEFT, false, 1.0f));
      v.controlWires.add(new Wire(1,FORWARD, false, 1.0f));
      v.controlWires.add(new Wire(1,RIGHT, false, 1.0f));
      
      return v;
  }
  
  BraitenbergVehicle makeVehicleTwoBCone () {
      BraitenbergVehicle v = new BraitenbergVehicle(world);

      // Hardpoint on the Vehicle's Nose:
      Hardpoint leftNosePoint = new Hardpoint(new Vector2D(0,-10), Math.PI/6);
      Hardpoint rightNosePoint = new Hardpoint(new Vector2D(0,-10), -Math.PI/6);

      // Add a sensor to the nose hardpoints:
      leftNosePoint.addSensor(SensorFactory.makeDistanceConeSensor(SHORT_LENGTH, Math.PI/6));
      rightNosePoint.addSensor(SensorFactory.makeDistanceConeSensor(SHORT_LENGTH, Math.PI/6));

      // Add the hardpoint to the ship:
      v.hardpoints.add(leftNosePoint);
      v.hardpoints.add(rightNosePoint);

      // Add a Modulator to the vehicle:
      v.modulators.add(new ModulatorIdentity());
      v.modulators.add(new ModulatorIdentity());

      // Wire the hardpoints to Modulators:
      v.sensorWires.add(new Wire(0,0, false, 1.0f));
      v.sensorWires.add(new Wire(1,1, false, 1.0f));

      // Wire the modulator to a Control Signal:
      v.controlWires.add(new Wire(0,FORWARD, false, 1.0f));
      v.controlWires.add(new Wire(0,LEFT, false, 1.0f));
      v.controlWires.add(new Wire(1,FORWARD, false, 1.0f));
      v.controlWires.add(new Wire(1,RIGHT, false, 1.0f));
      
      return v;
  }
  
  BraitenbergVehicle makeVehicleThreeARadius () {
      BraitenbergVehicle v = new BraitenbergVehicle(world);
      
      // Hardpoint on the Vehicle's Nose:
      Hardpoint leftNosePoint = new Hardpoint(new Vector2D(-25,-10), 0.0);
      Hardpoint rightNosePoint = new Hardpoint(new Vector2D(25,-10), 0.0);

      // Add a sensor to the nose hardpoints
      leftNosePoint.addSensor(SensorFactory.makeDistanceRadiusSensor(SHORT_LENGTH));
      rightNosePoint.addSensor(SensorFactory.makeDistanceRadiusSensor(SHORT_LENGTH));

      // Add the hardpoint to the ship:
      v.hardpoints.add(leftNosePoint);
      v.hardpoints.add(rightNosePoint);

      // Add a Modulator to the vehicle:
      v.modulators.add(new ModulatorIdentity());
      v.modulators.add(new ModulatorIdentity());

      // Wire the hardpoints to Modulators:
      v.sensorWires.add(new Wire(0,0, true, 1.0f));
      v.sensorWires.add(new Wire(1,1, true, 1.0f));

      // Wire the modulator to a Control Signal:
      v.controlWires.add(new Wire(0,FORWARD, false, 1.0f));
      v.controlWires.add(new Wire(0,RIGHT, false, 1.0f));
      v.controlWires.add(new Wire(1,FORWARD, false, 1.0f));
      v.controlWires.add(new Wire(1,LEFT, false, 1.0f));
      
      return v;
  }
  
  BraitenbergVehicle makeVehicleThreeBRadius () {
      BraitenbergVehicle v = new BraitenbergVehicle(world);

      // Hardpoint on the Vehicle's Nose:
      Hardpoint leftNosePoint = new Hardpoint(new Vector2D(-30,-10), 0.0);
      Hardpoint rightNosePoint = new Hardpoint(new Vector2D(30,-10), 0.0);

      // Add a sensor to the nose hardpoints:
      leftNosePoint.addSensor(SensorFactory.makeDistanceRadiusSensor(SHORT_LENGTH));
      rightNosePoint.addSensor(SensorFactory.makeDistanceRadiusSensor(SHORT_LENGTH));

      // Add the hardpoint to the ship:
      v.hardpoints.add(leftNosePoint);
      v.hardpoints.add(rightNosePoint);

      // Add a Modulator to the vehicle:
      v.modulators.add(new ModulatorParabolic());
      v.modulators.add(new ModulatorParabolic());

      // Wire the hardpoints to Modulators:
      v.sensorWires.add(new Wire(0,0, false, 1f));
      v.sensorWires.add(new Wire(1,1, false, 1f));

      // Wire the modulator to a Control Signal:
      v.controlWires.add(new Wire(0,FORWARD, true, 1.35f));// leftThrustWire
      v.controlWires.add(new Wire(0,RIGHT, true, 1.0f));   // rightSteerWire
      v.controlWires.add(new Wire(1,FORWARD, true, 1.35f));// rightThrustWire
      v.controlWires.add(new Wire(1,LEFT, true, 1.0f));    // leftSteerWire
      
      return v;
  }
  
  BraitenbergVehicle makeVehicleThreeACone () {
      BraitenbergVehicle v = new BraitenbergVehicle(world);

      // Hardpoint on the Vehicle's Nose:
      Hardpoint leftNosePoint = new Hardpoint(new Vector2D(0,-10), Math.PI/6);
      Hardpoint rightNosePoint = new Hardpoint(new Vector2D(0,-10), -Math.PI/6);

      // Add a sensor to the nose hardpoints:
      leftNosePoint.addSensor(SensorFactory.makeDistanceConeSensor(SHORT_LENGTH, Math.PI/6));
      rightNosePoint.addSensor(SensorFactory.makeDistanceConeSensor(SHORT_LENGTH, Math.PI/6));

      // Add the hardpoint to the ship:
      v.hardpoints.add(leftNosePoint);
      v.hardpoints.add(rightNosePoint);

      // Add a Modulator to the vehicle:
      v.modulators.add(new ModulatorIdentity());
      v.modulators.add(new ModulatorIdentity());

      // Wire the hardpoints to Modulators:
      v.sensorWires.add(new Wire(0,0, true, 1.0f));
      v.sensorWires.add(new Wire(1,1, true, 1.0f));

      // Wire the modulator to a Control Signal:
      v.controlWires.add(new Wire(0,FORWARD, false, .4f));
      v.controlWires.add(new Wire(0,RIGHT, false, 1.0f));
      v.controlWires.add(new Wire(1,FORWARD, false, .4f));
      v.controlWires.add(new Wire(1,LEFT, false, 1.0f));
      
      return v;
}
  
  BraitenbergVehicle makeVehicleThreeBCone () {
      BraitenbergVehicle v = new BraitenbergVehicle(world);

      // Hardpoint on the Vehicle's Nose:
      Hardpoint leftNosePoint = new Hardpoint(new Vector2D(0,-10), Math.PI/6);
      Hardpoint rightNosePoint = new Hardpoint(new Vector2D(0,-10), -Math.PI/6);

      // Add a sensor to the nose hardpoints:
      leftNosePoint.addSensor(SensorFactory.makeDistanceConeSensor(SHORT_LENGTH, Math.PI/6));
      rightNosePoint.addSensor(SensorFactory.makeDistanceConeSensor(SHORT_LENGTH, Math.PI/6));

      // Add the hardpoint to the ship:
      v.hardpoints.add(leftNosePoint);
      v.hardpoints.add(rightNosePoint);

      // Add a Modulator to the vehicle:
      v.modulators.add(new ModulatorIdentity());
      v.modulators.add(new ModulatorIdentity());

      // Wire the hardpoints to Modulators:
      v.sensorWires.add(new Wire(0,0, true, 1.0f));
      v.sensorWires.add(new Wire(1,1, true, 1.0f));

      // Wire the modulator to a Control Signal:
      v.controlWires.add(new Wire(0,FORWARD, false, 0.4f));
      v.controlWires.add(new Wire(0,LEFT, false, 1.0f));
      v.controlWires.add(new Wire(1,FORWARD, false, 0.4f));
      v.controlWires.add(new Wire(1,RIGHT, false, 1.0f));
      
      return v;
  }
  
  BraitenbergVehicle makeVehicleFourARadius () {
      BraitenbergVehicle v = new BraitenbergVehicle(world);

      // Left nose point:
      Hardpoint leftNosePoint = new Hardpoint(new Vector2D(30,-10), Math.PI/6);
      leftNosePoint.addSensor(SensorFactory.makeDistanceRadiusSensor(SHORT_LENGTH) );
      v.hardpoints.add(leftNosePoint);
      v.modulators.add(new ModulatorParabolic());
      v.modulators.add(new ModulatorIdentity());
      

      v.sensorWires.add  (new Wire(0,0, false, 1.0f));
      v.controlWires.add (new Wire(0,LEFT, true, 1.0f));
      
      v.sensorWires.add  (new Wire(0,1, false, 1.0f));
      v.controlWires.add (new Wire(1,FORWARD, true, .5f));
      
      // Right nose point:
      Hardpoint rightNosePoint = new Hardpoint(new Vector2D(-30,-10), -Math.PI/6);
      rightNosePoint.addSensor(SensorFactory.makeDistanceRadiusSensor(SHORT_LENGTH));
      v.hardpoints.add(rightNosePoint);
      v.modulators.add(new ModulatorParabolic());
      v.modulators.add(new ModulatorIdentity());
      
      v.sensorWires.add (new Wire(1,2, false, 1.0f));
      v.controlWires.add (new Wire(2,RIGHT, true, 1.0f));
      
      v.sensorWires.add (new Wire(1,3, false, 1.0f));
      v.controlWires.add(new Wire(3,FORWARD, true, .5f));
      
      // Speed point:
      Hardpoint speedPoint = new Hardpoint(new Vector2D(0,0), 0);
      speedPoint.addSensor(SensorFactory.makeShipSpeedSensor());
      v.hardpoints.add(speedPoint);
      
      v.modulators.add(new ModulatorIdentity());
      
      v.sensorWires.add(new Wire(2,4, false, 1.0f));
      v.controlWires.add(new Wire(4,BACK, false, 1.7f));

      
      return v;
  }
  
  BraitenbergVehicle makeVehicleFourBRadius () {
    BraitenbergVehicle v = new BraitenbergVehicle(world);

    // Left nose point:
    Hardpoint leftNosePoint = new Hardpoint(new Vector2D(35,-10), Math.PI/6);
    leftNosePoint.addSensor(SensorFactory.makeDistanceRadiusSensor(SHORT_LENGTH) );
    v.hardpoints.add(leftNosePoint);
    v.modulators.add(new ModulatorParabolic());
    v.modulators.add(new ModulatorIdentity());
    

    v.sensorWires.add (new Wire(0,0, false, 1.0f));
    v.controlWires.add (new Wire(0,LEFT, true, .5f));
    
    v.sensorWires.add (new Wire(0,1, false, 1.0f));
    v.controlWires.add(new Wire(1,FORWARD, true, .2f));
    
    // Right nose point:
    Hardpoint rightNosePoint = new Hardpoint(new Vector2D(-35,-10), -Math.PI/6);
    rightNosePoint.addSensor(SensorFactory.makeDistanceRadiusSensor(SHORT_LENGTH));
    v.hardpoints.add(rightNosePoint);
    v.modulators.add(new ModulatorParabolic());
    v.modulators.add(new ModulatorIdentity());
    
    v.sensorWires.add (new Wire(1,2, false, 1.0f));
    v.controlWires.add (new Wire(2,RIGHT, true, .5f));
    
    v.sensorWires.add (new Wire(1,3, false, 1.0f));
    v.controlWires.add(new Wire(3,FORWARD, true, .2f));
    
    // Speed point:
    Hardpoint speedPoint = new Hardpoint(new Vector2D(0,0), 0);
    speedPoint.addSensor(SensorFactory.makeShipSpeedSensor());
    v.hardpoints.add(speedPoint);
    
    v.modulators.add(new ModulatorIdentity());
    
    v.sensorWires.add(new Wire(2,4, false, 1.0f));
    v.controlWires.add(new Wire(4,BACK, false, 1.7f));

    return v;
}
  
  
  // A vehicle with sensors arranged to approximate a basic eye:
  BraitenbergVehicle makeVehicleRayEye () {
    BraitenbergVehicle v = new BraitenbergVehicle(world);
    
    int numRays = 32; // Should be even number.
    double raySpread = 2 * Math.PI / numRays; 
    
    // Create HardPoints arranged like spokes on a wheel:
    for (int i = 0; i < numRays; i++ ) {
      Hardpoint h = new Hardpoint(new Vector2D(0,0), i * raySpread);
      h.addSensor(SensorFactory.makeDistanceRaySensor(MEDIUM_LENGTH)); // Add a sensor to the hardpoint:
      v.hardpoints.add(h);                           // Add the hardpoint to the chassis:
      v.modulators.add(new ModulatorIdentity());     // Add Modulators to the vehicle:
      v.sensorWires.add(new Wire(i,i, false, 1.0f)); // Wire the hardpoint to the Modulator:
    }
    
    // We now have 16 sensors wired into the vehicle in a spoke-configuration.
    // The wire i maps to the sensor pointing to raySpread*i (counter-clockwise).
    
    // Flee from objects behind:
    v.controlWires.add(new Wire(numRays/2,FORWARD, false, 1f));
    
    // Thrust except when objects are near:
    v.controlWires.add(new Wire(0,        FORWARD, true, 5f) );
    v.controlWires.add(new Wire(1,        FORWARD, true, 5f) );
    v.controlWires.add(new Wire(numRays-1,FORWARD, true, 5f) );
    
    // Shoot objects passing in front:
    v.controlWires.add(new Wire(0,         SHOOT, false, 1f) );
    v.controlWires.add(new Wire(1,         SHOOT, false, 1f) );
    v.controlWires.add(new Wire(numRays-1, SHOOT, false, 1f) );
    
    // Slow down when objects passing in front:
    v.controlWires.add(new Wire(0,        BACK, false, 1.0f) );
    v.controlWires.add(new Wire(1,        BACK, false, 1.0f) );
    v.controlWires.add(new Wire(1,        BACK, false, 1.0f) );
    v.controlWires.add(new Wire(numRays-1,BACK, false, 1.0f) );
    v.controlWires.add(new Wire(numRays-2,BACK, false, 1.0f) );
    
    // Turn toward objects on the left:
    for (int k = 1; k < numRays/2; k++) {
      v.controlWires.add(new Wire(k,LEFT, false, 1f));
    }
    
    // Turn toward objects on the right:
    for (int j = numRays/2+1; j < numRays; j++) {
      v.controlWires.add(new Wire(j,RIGHT, false, 1f));
    }
    
    // Add one hardpoint to control speed:
    // Activated when the 
    Hardpoint speedControl = new Hardpoint(new Vector2D(0,0), 0);
    speedControl.addSensor(SensorFactory.makeShipSpeedSensor());
    v.hardpoints.add(speedControl);
    v.modulators.add(new ModulatorIdentity());
    v.sensorWires.add(new Wire(numRays, numRays, false, 1.0f));
    v.controlWires.add(new Wire(numRays, BACK, false, 1.5f));
    
    return v;
  }

  BraitenbergVehicle makeVehicleRadialRegions() {
    BraitenbergVehicle v = new BraitenbergVehicle(world);

    // Hardpoints on the Vehicle:
    Hardpoint layer0 = new Hardpoint(new Vector2D(0,0), 0);
    Hardpoint layer1 = new Hardpoint(new Vector2D(0,0), 0);
    Hardpoint layer2 = new Hardpoint(new Vector2D(0,0), 0);
    Hardpoint layer3 = new Hardpoint(new Vector2D(0,0), 0);
    Hardpoint layer4 = new Hardpoint(new Vector2D(0,0), 0);
    Hardpoint layer5 = new Hardpoint(new Vector2D(0,0), 0);

    // Add sensors to the hardpoints:
    layer0.addSensor(SensorFactory.makeDistanceRadiusSensor(MEDIUM_LENGTH/4));
    layer1.addSensor(SensorFactory.makeDistanceRadiusSensor(MEDIUM_LENGTH/3));
    layer2.addSensor(SensorFactory.makeDistanceRadiusSensor(MEDIUM_LENGTH/2));
    layer3.addSensor(SensorFactory.makeDistanceRadiusSensor(MEDIUM_LENGTH));
    layer4.addSensor(SensorFactory.makeDistanceRadiusSensor(MEDIUM_LENGTH*1.25));
    layer5.addSensor(SensorFactory.makeDistanceRadiusSensor(MEDIUM_LENGTH*1.5));

    // Add the hardpoints to the ship:
    v.hardpoints.add(layer0);
    v.hardpoints.add(layer1);
    v.hardpoints.add(layer2);
    v.hardpoints.add(layer3);
    v.hardpoints.add(layer4);
    v.hardpoints.add(layer5);

    // Add Modulators to the vehicle:
    v.modulators.add(new ModulatorIdentity());
    v.modulators.add(new ModulatorIdentity());
    v.modulators.add(new ModulatorIdentity());
    v.modulators.add(new ModulatorIdentity());
    v.modulators.add(new ModulatorIdentity());
    v.modulators.add(new ModulatorIdentity());

    // Wire the hardpoints to Modulators:
    v.sensorWires.add(new Wire(0,0, false, 1.0f));
    v.sensorWires.add(new Wire(1,1, false, 1.0f));
    v.sensorWires.add(new Wire(2,2, false, 1.0f));
    v.sensorWires.add(new Wire(3,3, true,  1.0f));
    v.sensorWires.add(new Wire(4,4, false, 1.0f));
    v.sensorWires.add(new Wire(5,5, false, 1.0f));

    // Wire the modulator to a Control Signal:
    v.controlWires.add(new Wire(0,RIGHT,  false, 3.0f));
    v.controlWires.add(new Wire(1,BACK,   false, 1.0f));
    v.controlWires.add(new Wire(2,RIGHT,  false, 1.0f));
    v.controlWires.add(new Wire(3,FORWARD,false, 0.3f));
    v.controlWires.add(new Wire(4,LEFT,   false, 1.0f));
    v.controlWires.add(new Wire(5,JUMP,   false, 5.0f));
    
    return v;
  }
  
  BraitenbergVehicle makeVehiclePolarRegions() {
    BraitenbergVehicle v = new BraitenbergVehicle(world);

    // Hardpoints on the Vehicle:
    Hardpoint forwardNose = new Hardpoint(new Vector2D(0,-10), QUARTER_PI/8);
    Hardpoint forwardProx = new Hardpoint(new Vector2D(0,0), 0);
    Hardpoint rightProx   = new Hardpoint(new Vector2D(0,0), HALF_PI);
    Hardpoint rearProx    = new Hardpoint(new Vector2D(0,0), Math.PI);
    Hardpoint leftProx    = new Hardpoint(new Vector2D(0,0), -HALF_PI);

    // Add sensors to the hardpoints:
    forwardNose.addSensor(SensorFactory.makeDistanceConeSensor(MEDIUM_LENGTH*3, QUARTER_PI/3));
    forwardProx.addSensor(SensorFactory.makeDistanceConeSensor(SHORT_LENGTH, QUARTER_PI));
    rightProx.addSensor(SensorFactory.makeDistanceConeSensor(SHORT_LENGTH, QUARTER_PI*3));
    rearProx.addSensor(SensorFactory.makeDistanceConeSensor(SHORT_LENGTH, QUARTER_PI));
    leftProx.addSensor(SensorFactory.makeDistanceConeSensor(SHORT_LENGTH, QUARTER_PI*3));

    // Add the hardpoints to the ship:
    v.hardpoints.add(forwardNose);
    v.hardpoints.add(forwardProx);
    v.hardpoints.add(rightProx);
    v.hardpoints.add(rearProx);
    v.hardpoints.add(leftProx);

    // Add Modulators to the vehicle:
    v.modulators.add(new ModulatorBinary());
    v.modulators.add(new ModulatorIdentity());
    v.modulators.add(new ModulatorIdentity());
    v.modulators.add(new ModulatorIdentity());
    v.modulators.add(new ModulatorIdentity());

    // Wire the hardpoints to Modulators:
    v.sensorWires.add(new Wire(0,0, false, 1.0f));
    v.sensorWires.add(new Wire(1,1, false, 1.0f));
    v.sensorWires.add(new Wire(2,2, true, 1.0f));
    v.sensorWires.add(new Wire(3,3, true,  1.0f));
    v.sensorWires.add(new Wire(4,4, false, 1.0f));

    // Wire the modulator to a Control Signal:
    v.controlWires.add(new Wire(0,SHOOT,    false,2.0f));
    v.controlWires.add(new Wire(1,BACK,    false, 1.0f));
    v.controlWires.add(new Wire(2,RIGHT,   true,  0.95f));
    v.controlWires.add(new Wire(3,FORWARD, false, 0.3f));
    v.controlWires.add(new Wire(4,LEFT,    false, 1.15f));
    
    return v;
  }
  
  BraitenbergVehicle makeVehicleStationary() {
    BraitenbergVehicle v = new BraitenbergVehicle(world);

    // Hardpoints on the Vehicle:
    Hardpoint rightTrackingCone = new Hardpoint(new Vector2D(0,-10), 0);
    Hardpoint leftTrackingBeam = new Hardpoint(new Vector2D(0,-10), QUARTER_PI/6);
    Hardpoint targetingCone = new Hardpoint(new Vector2D(0,-10), 0);
    Hardpoint panicRadius = new Hardpoint(new Vector2D(0,0), 0);


    // Add sensors to the hardpoints:
    rightTrackingCone.addSensor(SensorFactory.makeDistanceConeSensor(LONG_LENGTH, QUARTER_PI/8));
    leftTrackingBeam.addSensor(SensorFactory.makeDistanceRaySensor(LONG_LENGTH));
    targetingCone.addSensor(SensorFactory.makeDistanceConeSensor(LONG_LENGTH, QUARTER_PI/8));
    panicRadius.addSensor(SensorFactory.makeDistanceRadiusSensor(SHORT_LENGTH/2));

    // Add the hardpoints to the ship:
    v.hardpoints.add(rightTrackingCone);
    v.hardpoints.add(leftTrackingBeam);
    v.hardpoints.add(targetingCone);
    v.hardpoints.add(panicRadius);

    // Add Modulators to the vehicle:
    v.modulators.add(new ModulatorIdentity());
    v.modulators.add(new ModulatorBinary());
    v.modulators.add(new ModulatorBinary());
    v.modulators.add(new ModulatorBinary());

    // Wire the hardpoints to Modulators:
    v.sensorWires.add(new Wire(0,0, false, 1.0f)); // Rotate right to scan the play field
    v.sensorWires.add(new Wire(0,0, true,  1.0f)); 
    
    v.sensorWires.add(new Wire(1,1, false, 5.0f)); // Rotate LEFT to keep target in sights
    v.sensorWires.add(new Wire(2,2, false, 5.0f)); // Fire when a target is in the targeting cone

    v.sensorWires.add(new Wire(3,3, false,  5.0f)); // Escape when objects come near.

    // Wire the modulator to a Control Signal:
    v.controlWires.add(new Wire(0,RIGHT, false, 0.3f));
    v.controlWires.add(new Wire(1,LEFT,  false, 0.32f));
    v.controlWires.add(new Wire(2,SHOOT, false, 1.0f));
    v.controlWires.add(new Wire(3,JUMP,  false, 1.0f));
    
    return v;
  }
  
  // TODO L FINISH THIS VEHICLE
  BraitenbergVehicle makeVehicleReverseCone() {
    BraitenbergVehicle v = new BraitenbergVehicle(world);
    
    // Create HardPoints fanned like spokes on a wheel:
    // Which will be used to target objects
    int numRays = 3;
    double raySpread = (QUARTER_PI/16) / numRays; 
    for (int i = 0; i < numRays; i++ ) {
      Hardpoint h = new Hardpoint(new Vector2D(0,0), (i-1) * raySpread);
      h.addSensor(SensorFactory.makeDistanceRaySensor(LONG_LENGTH)); // Add a sensor to the hardpoint:
      v.hardpoints.add(h);                           // Add the hardpoint to the chassis:
      v.modulators.add(new ModulatorIdentity());     // Add Modulators to the vehicle:
      v.sensorWires.add(new Wire(i,i, false, 1.0f)); // Wire the hardpoint to the Modulator:
      v.controlWires.add(new Wire(i, SHOOT, false, 1f) );
    }
    
    
    // Hardpoints on the Vehicle for steering:
    Hardpoint leftNosePoint = new Hardpoint(new Vector2D(0,-10), Math.PI-QUARTER_PI/2);
    Hardpoint rightNosePoint = new Hardpoint(new Vector2D(0,-10), Math.PI+QUARTER_PI/2);
    Hardpoint emergencyTurn = new Hardpoint(new Vector2D(0,-10), Math.PI);

    // Add sensors to the hardpoints:
    leftNosePoint.addSensor(SensorFactory.makeDistanceConeSensor(MEDIUM_LENGTH, QUARTER_PI));
    rightNosePoint.addSensor(SensorFactory.makeDistanceConeSensor(MEDIUM_LENGTH, QUARTER_PI));
    emergencyTurn.addSensor(SensorFactory.makeDistanceConeSensor(MEDIUM_LENGTH/3, Math.PI));
    
    // Add the hardpoints to the ship:
    v.hardpoints.add(leftNosePoint);
    v.hardpoints.add(rightNosePoint);
    v.hardpoints.add(emergencyTurn);

    // Add Modulators to the vehicle:
    v.modulators.add(new ModulatorIdentity());
    v.modulators.add(new ModulatorIdentity());
    v.modulators.add(new ModulatorIdentity());
    v.modulators.add(new ModulatorIdentity());

    // Wire the hardpoints to Modulators:
    v.sensorWires.add(new Wire(numRays + 0, numRays + 0, false, 1.0f));
    v.sensorWires.add(new Wire(numRays + 0, numRays + 1, true, 1.0f));
    v.sensorWires.add(new Wire(numRays + 1, numRays + 2, false, 1.0f));
    v.sensorWires.add(new Wire(numRays + 2, numRays + 3, false, 1.0f));


    // Wire the modulator to a Control Signal:
    v.controlWires.add(new Wire(numRays + 0,LEFT, false, 0.9f)); // Bias turning to escape head-on collisions.
    v.controlWires.add(new Wire(numRays + 1,BACK, false, 0.7f));
    v.controlWires.add(new Wire(numRays + 1,LEFT, false, 0.0f));
    v.controlWires.add(new Wire(numRays + 2,RIGHT, false, 1.0f));
    v.controlWires.add(new Wire(numRays + 3,RIGHT, false, 2.0f));
    


    
    return v;
  }  

  // TODO: Replace this section...
  // Make starfish...
  List<BraitenbergVehicle> makeStarfish() {
    List<BraitenbergVehicle> vehicles = equipSensors(chassisPile.get(0));

    vehicles = equipFixemodulators(vehicles);
    vehicles = equipWires(vehicles);

    return vehicles;
  }

  // Create a Braitenberg vehicle with empty hardpoints arranged in a 
  // star-pattern:
  BraitenbergVehicle makeStarfishChassis() {
    // Sensors arranged in a star pattern:
    BraitenbergVehicle starfish = new BraitenbergVehicle(world);
    // Front
    starfish.hardpoints.add(
      new Hardpoint(new Vector2D(0,-100), 0.0));

    // Front-Left
    starfish.hardpoints.add(
      new Hardpoint(new Vector2D(-100,-100), Math.PI/4));

    // Rear-Left
    starfish.hardpoints.add(
      new Hardpoint(new Vector2D(-100,100), 3*Math.PI/4));

    // Rear-Right
    starfish.hardpoints.add(
      new Hardpoint(new Vector2D(100,100), 5*Math.PI/4));

    // Front-Right
    starfish.hardpoints.add(
      new Hardpoint(new Vector2D(100,-100), 7*Math.PI/4));

    return starfish;
  }

  // Equip a chassis with every combination of sensors possible and return the
  // resulting list of BraitenbergVehicles:
  List<BraitenbergVehicle> equipSensors(BraitenbergVehicle chassis) {
    List<BraitenbergVehicle> equippedVehicles = new ArrayList<BraitenbergVehicle>();
    BraitenbergVehicle nextVehicle;
    int numHardpoints = chassis.hardpoints.size();
    int numSensors = sensorPile.size();
    List<int[]>permutations = getPermutations(numHardpoints, numSensors);

    
    // <permutation> helps us generate each permutation of sensors:
    // Digit index corresponds to a hardpoint on <chassis>.
    // Digit value corresponds to a sensor type from <sensorPile>.
    for (int[] permutation : permutations) {
      nextVehicle = new BraitenbergVehicle(chassis); // Get a clone of the chassis.

      // Equip <nextVehicle> according to the current permutation...
      for (int i = 0; i < numHardpoints; i++) {
        Sensor selectedSensor = sensorPile.get(permutation[i]);
        nextVehicle.hardpoints.get(i).addSensor(selectedSensor);
      }

      equippedVehicles.add(nextVehicle); // Add the equipped.
    }

    return equippedVehicles;
  }

  // Equip Five Equality Modulators to the BraitenbergVehicles in <vehicles>:
  List<BraitenbergVehicle> equipFixemodulators(List<BraitenbergVehicle> vehicles) {
    for (BraitenbergVehicle nextVehicle : vehicles) {
      nextVehicle.modulators.add(ModulatorPile.get(0).copy());
      nextVehicle.modulators.add(ModulatorPile.get(0).copy());
      nextVehicle.modulators.add(ModulatorPile.get(0).copy());
      nextVehicle.modulators.add(ModulatorPile.get(0).copy());
      nextVehicle.modulators.add(ModulatorPile.get(0).copy());
    }

    return vehicles;
  }

  // TODO: Replace this section...
  // Equip a list of vehicles with every combination of wires possible and return
  // the resulting list of BraitenbergVehicles:
  List<BraitenbergVehicle> equipWires(List<BraitenbergVehicle> vehicles) {
    List<BraitenbergVehicle> wiredVehicles = new ArrayList<BraitenbergVehicle>();
    BraitenbergVehicle nextVehicle;
    int numHardpoints = vehicles.get(0).hardpoints.size();
    // Exclude hyperspace signal:
    int numControlSignals = vehicles.get(0).controlSignals.length - 1;
    List<int[]>permutations = getPermutationsNoRepetions( numHardpoints, 
                                                          numControlSignals);

    for (BraitenbergVehicle original : vehicles) {
      // <permutation> helps us generate each permutation of wirings:
      // Digit index corresponds to a hardpoint on <chassis>.
      // Digit value corresponds to a control signal on <chassis>.
      for (int[] permutation : permutations) {
        nextVehicle = new BraitenbergVehicle(original); // Get a clone of the chassis.

        // Wire <nextVehicle> according to the current permutation...
        for (int i = numHardpoints-1; i >= 0; i--) {
          nextVehicle.sensorWires.add(new Wire(i, i, false, 1.0f));
          nextVehicle.controlWires.add(new Wire(i, permutation[i], false, 1.0f));
        }

        wiredVehicles.add(nextVehicle); // Add the wired vehicle.
      }
    }


    return wiredVehicles;
  }

  // TODO: Replace this section...
  // Return all permutations:
  static List<int[]> getPermutations(int digits, int base) {
    List<int[]> permutations = new ArrayList<int[]>();
    int[] curPermutation = new int[digits];
    int digitSum = 0; // Sum of the digits in the current permutation.

    permutations.add(curPermutation.clone()); // Add the first permutation.

    // While we have not reached the last permutation of digits:
    while (digitSum != (base-1) * digits) {
      digitSum = 0;

      // Increment permutation modulus(maxValues)
      curPermutation[0]++;
      for (int j = 0; j < digits; j++) {
        // Carry overflowed digit to the next column:
        if (curPermutation[j] == base) {
          curPermutation[j] = 0;
          curPermutation[j+1]++;
        }

        // Track the sum of the digits of the new permutation:
        digitSum += curPermutation[j];
      }
      // Store the permutation:
      permutations.add(curPermutation.clone());
    }

    return permutations;
  }

  // TODO: Replace this section...
  // Return all permutations in which no digit occurs more than once:
  // Perhaps inefficient but simple.
  static List<int[]> getPermutationsNoRepetions(int digits, int base) {
    List<int[]> permutations = getPermutations(digits, base);
    Iterator<int[]> iter = permutations.iterator();
    int[] permutation;  // Permutation being inspected.

    // Remove any permutation in which the same digit occurs more than once:
    while (iter.hasNext()) {
      permutation = iter.next();

      for (int i = 0; i < permutation.length - 1; i++) {
        for (int j = i+1; j < permutation.length; j++) {
          // If any digit occurs twice:
          // Remove it.
          if (permutation[i] == permutation[j]) {
            j = permutation.length; // Stop checking this permutation.
            i = permutation.length; // Stop checking this permutation.
            iter.remove();
          }
        }
      }
    }

    return permutations;
  }


}