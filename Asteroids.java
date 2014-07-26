/******************************************************************************
  Asteroids, Version 1.3

  Copyright 1998-2001 by Mike Hall.
  Please see http://www.brainjar.com for terms of use.

  Revision History:

  1.01, 12/18/1999: Increased number of active photons allowed.
                    Improved explosions for more realism.
                    Added progress bar for loading of sound clips.
  1.2,  12/23/1999: Increased frame rate for smoother animation.
                    Modified code to calculate game object speeds and timer
                    counters based on the frame rate so they will remain
                    constant.
                    Improved speed limit checking for ship.
                    Removed wrapping of photons around screen and set a fixed
                    firing rate.
                    Added sprites for ship's thrusters.
  1.3,  01/25/2001: Updated to JDK 1.1.8.

  Usage:

  <applet code="Asteroids.class" width=w height=h></applet>

  Keyboard Controls:

  S            - Start Game    P           - Pause Game
  Cursor Left  - Rotate Left   Cursor Up   - Fire Thrusters
  Cursor Right - Rotate Right  Cursor Down - Fire Retro Thrusters
  Spacebar     - Fire Cannon   H           - Hyperspace
  M            - Toggle Sound  D           - Toggle Graphics Detail

******************************************************************************/

/****
 *
 * Project:    Flotsam Braitenberg Vehicles
 * 
 * Author:      Nicholas Macdonald
 *
 * Date:        January 16, 2014
 *
 * Description: A series of robots implemented inside of Asteroids. 
 *              The robots are modeled after those described in Valentino 
 *              Braitenberg's Vehicles. They are equipped with; sensors which 
 *              are excited by various qualities of the environment, Modulator 
 *              devices which modulate the raw output of sensors, and 'motors' 
 *              which have various effects on the robot and its environment.
 ****/

import java.awt.*;
import java.awt.event.*;
import java.util.*;
import java.util.List;
import java.applet.Applet;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.LinearRing;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.operation.distance.DistanceOp;
import com.vividsolutions.jts.util.GeometricShapeFactory;


// A singleton used to generate Geometry objects in JTS:
class SingleGeometryFactory {
  static final int NUM_POINTS = 30;

  private static GeometryFactory gFactory = null;
  
  protected SingleGeometryFactory() { /* Prevent instantiaton */ }
  public static GeometryFactory getFactory() {
    // Lazily instantiate a factory:
    if(gFactory == null) {
      gFactory = new GeometryFactory();
    }
    return gFactory;
  }
  
  private static GeometricShapeFactory sFactory = null;
  public static GeometricShapeFactory getShapeFactory() {
    // Lazily instantiate a factory:
    if(sFactory == null) {
      sFactory = new GeometricShapeFactory(getFactory());
      sFactory.setNumPoints(NUM_POINTS);
    }
    return sFactory;
  }
}

/*
  Provides some standard Vector Operations such as dot product
  or distance between vectors.
*/
class Vector2D {
  double x, y;

  // Constructor
  public Vector2D(double x, double y) {
    this.x = x;
    this.y = y;
  }

  // Copy Constructor
  public Vector2D(Vector2D v) {
    this.x = v.getX();
    this.y = v.getY();
  }

  // Distance-Squared between <v1> and <v2>:
  public static double distanceSQ(Vector2D v1, Vector2D v2) {
    double vX = v1.getX() - v2.getX();
    double vY = v1.getY() - v2.getY();
    return (vX*vX + vY*vY);
  }

  // Distance between <v1> and <v2>:
  public static double distance(Vector2D v1, Vector2D v2) {
    return Math.sqrt(distanceSQ(v1, v2));
  }

  // Euclidean Norm of <v>:
  public static double norm(Vector2D v) {
    return Math.sqrt(v.getX()*v.getX() + v.getY()*v.getY());
  }

  // Unit vector of <v>:
  public static Vector2D unit(Vector2D v) {
    double vNorm = Vector2D.norm(v);
    double xNorm = v.getX() / vNorm;
    double yNorm = v.getY() / vNorm;
    return new Vector2D(xNorm, yNorm);
  }

  // Dot-Product of <v1> and <v2>:
  public static double dot(Vector2D v1, Vector2D v2) {
    return (v1.getX()*v2.getX() + v1.getY()*v2.getY());
  }

  // Return the angle, in radians, from <v1> to <v2>:
  public static double angleFromTo(Vector2D v1, Vector2D v2) {
    double r1 = Math.atan2(v1.getY(), v1.getX());
    double r2 = Math.atan2(v2.getY(), v2.getX());
    return (r2 - r1);
  }

  // Sum of <v1> and <v2>:
  public static Vector2D sum(Vector2D v1, Vector2D v2) {
    return new Vector2D(v1.getX() + v2.getX(), v1.getY() + v2.getY());
  }

  // Difference between <v1> and <v2>:
  public static Vector2D difference(Vector2D v1, Vector2D v2) {
    return new Vector2D(v1.getX() - v2.getX(), v1.getY() - v2.getY());
  }

  public String toString() {
    return String.format("X:%,5.2f, Y:%,5.2f", this.x, this.y);
  }

  // Mutators:
  public void setX(double x) { this.x = x; }
  public void setY(double y) { this.y = y; }
  public void set(double x, double y) {
    this.x = x;
    this.y = y;
  }

  // Accessors:
  public double getY() { return y; }
  public double getX() { return x; }
}

/*
  Define devices which are sources or receivers of signals:
*/
interface Source    { double getOutput(); }
interface Receiver  { void input(double input); }

/*
  Defines the interface to a sensor device which returns a double when queried:
*/
interface Sensor extends Source{
  double SHORT_LENGTH = 200.0;
  double MEDIUM_LENGTH = 300.0;
  double LONG_LENGTH = 2000.0;
  double SLOW_SPEED = 5.0;
  
  // Sense some aspect of the GameState <world> given the sensor is at:
  // <loc> with a heading of <rot>.
  void sense(Point loc, double rot, GameState world);
  
  // Return a polygon in the shape of the detection field of the sensor:
  PolyWrapper getShape();
}


/*
   A sensor which analyzes the GameState and returns a Double <output> which is
   some measure of the environment. The nature of <output> depends on the implementation
   of the particular sensor.
 */
class DoubleSensor implements Sensor {
  double output = 0.0;
  PolyWrapper shape;
  
  // Sense some aspect of the GameState <world> given the sensor is at:
  // <loc> with a heading of <rot>.
  public void sense(Point loc, double rot, GameState world) {}
  public double getOutput() { return output; }
  public PolyWrapper getShape() { return shape; }
}

/*
  Detects all colliders within a circle described by <loc, detectionRadius>.
  Assigns <output> the complement of the distance to the nearest intersected 
  collider as a fraction of <detectionRadius>.
  i.e. <output> = 1 - (<distanceToNearest> / <detectionRadius>)
  This gives:
    ~one when objects are exactly on the sensor
    ~values close to one when objects are very near
    ~values close to zero when objects are very far
    ~zero when all objects are outside of <detectionRadius>
*/
class SensorRadiusShortDist extends DoubleSensor {
  public void sense(Point loc, double orientation, GameState world) {
    double detectionRadius = SHORT_LENGTH; // Radius of the detection circle
    double result = 0.0f;
    
    // List of AsteroidsSprites that intersect with a circle described by
    // <loc, distanceToNearest>
    List<AsteroidsSprite> intersected = 
      world.intersectRadius(detectionRadius, loc);
    
    // Update the sensor's shape:
    SingleGeometryFactory.getShapeFactory().setSize(detectionRadius*2);
    SingleGeometryFactory.getShapeFactory().setBase(new Coordinate(
        loc.getX()-SHORT_LENGTH, 
        loc.getY()-SHORT_LENGTH));
    
    shape = new PolyWrapper(SingleGeometryFactory.getShapeFactory().createCircle());

    // The nearest AsteroidsSprite:
    AsteroidsSprite nearest = AsteroidsSprite.nearest(loc, intersected);

    // Calculate result if something was detected:
    if (nearest != null) {
      double distanceToNearest = loc.distance(nearest.sprite); 
      double fraction = distanceToNearest / detectionRadius;
      result = (fraction > 1.0) ? 0.0 : 1.0-fraction;
    }

    output = result;
  }
}

/*
  Detects all colliders within an equi-triangle described by <loc, orientation>.
  Assigns <output> the complement of the distance to the nearest intersected 
  collider as a fraction of <detectionRadius>.
  i.e. <output> = 1 - (<distanceToNearest> / <sideLength>)
  This gives:
    ~one when objects are exactly on the sensor
    ~values close to one when objects are very near
    ~values close to zero when objects are very far
    ~zero when all objects are outside of <detectionRadius>
*/
class SensorTriMedDist extends DoubleSensor {
  public void sense(Point loc, double orientation, GameState world) {
    double sideLength = MEDIUM_LENGTH;// Side length of the triangle
    double spread = Math.PI/6;        // Angle of the corner: side-<loc>-side.
    double result = 0.0f;

    // Generate the points of our detection triangle:
    Coordinate[] coords = new Coordinate[4];
    
    coords[0] = new Coordinate(loc.getX(), loc.getY());
    
    coords[1] = new Coordinate(
        loc.getX() - Math.sin(orientation + (spread/2)) * sideLength,
        loc.getY() - Math.cos(orientation + (spread/2)) * sideLength);
    
    coords[2] = new Coordinate(
        loc.getX() - Math.sin(orientation - (spread/2)) * sideLength,
        loc.getY() - Math.cos(orientation - (spread/2)) * sideLength);
    
    // Close the polygon:
    coords[3] = new Coordinate(coords[0]);
    
    Polygon detectionTri = SingleGeometryFactory.getFactory().createPolygon(coords);
    
    // List of AsteroidsSprites that intersect with a circle described by
    // <loc, distanceToNearest>
    List<AsteroidsSprite> intersected = 
      world.intersectTri(detectionTri);

    // Find the nearest AsteroidsSprite:
    AsteroidsSprite nearest = AsteroidsSprite.nearest(loc, intersected);

    // Calculate result only if something was detected:
    if (nearest != null) {
      // Normalize the distance to <nearest>:
      double distanceToNearest = loc.distance(nearest.sprite);
      double fraction = distanceToNearest / sideLength;
      result = (fraction > 1.0) ? 0.0 : 1.0-fraction;
    }

    output = result;
  }
}

/*
  Detects all colliders intersecting a line described by <loc, orientation>.
  Assigns <output> the complement of the distance to the nearest intersected 
  collider as a fraction of <lineLength>.
  i.e. <output> = 1 - (<distanceToNearest> / <lineLength>)
  This gives:
    ~one when objects are exactly on the sensor
    ~values close to one when objects are very near
    ~values close to zero when objects are very far
    ~zero when all objects are outside of detection beam
*/
class SensorRayLongDist extends DoubleSensor {
  public void sense(Point loc, double orientation, GameState world) {
    double lineLength = MEDIUM_LENGTH; // Length of the detection-beam.
    double result = 0.0f;

    // Generate the points of our detection line:
    Coordinate[] coords = new Coordinate[2];
    // Start point:
    coords[0] = new Coordinate(loc.getX(), loc.getY());
    
    // End point:
    coords[1] = new Coordinate(
        loc.getX() -Math.sin(orientation)*lineLength, 
        loc.getY() -Math.cos(orientation)*lineLength);
    
    LineString line = SingleGeometryFactory.getFactory().createLineString(coords);
    
    // List of colliders that intersect with <line>:
    List<AsteroidsSprite> intersected = world.intersectLine(line);

    // The nearest AsteroidsSprite:
    AsteroidsSprite nearest = AsteroidsSprite.nearest(loc, intersected);

    // Calculate result if something was detected:
    if (nearest != null) {
      // Normalize the distance to <nearest>:
      double distanceToNearest = loc.distance(nearest.sprite);
      double fraction = distanceToNearest / lineLength;
      result = (fraction > 1.0) ? 0.0 : 1.0-fraction;
    }

    output = result;
  }
}

/*
  Detects all colliders intersecting a line described by <loc, orientation>.
  Assigns <output> the complement of the speed to the nearest intersected collider 
  as a fraction of <baseSpeed>.
  i.e. <output> = 1 - (<speed> / <baseSpeed>)
  This gives:
    ~one when objects are moving at speed >= <baseSpeed>
    ~values close to one when objects are moving at nearly <baseSpeed>
    ~values close to zero when objects are moving at nearly zero
    ~zero when all objects are outside of detection beam
*/
class SensorRayLongSpeed extends DoubleSensor {
  public void sense(Point loc, double orientation, GameState world) {
    double lineLength = LONG_LENGTH; // Length of the detection-beam.
    double baseSpeed = SLOW_SPEED;
    double result = 0.0f;

    // Generate the points of our detection line:
    Coordinate[] coords = new Coordinate[2];
    // Start point:
    coords[0] = new Coordinate(
        loc.getX() + (AsteroidsSprite.width / 2), 
        loc.getY() + (AsteroidsSprite.height / 2));
    
    // End point:
    coords[1] = new Coordinate(
        loc.getX() -Math.sin(orientation)*lineLength + (AsteroidsSprite.width / 2), 
        loc.getY() -Math.cos(orientation)*lineLength + (AsteroidsSprite.height / 2));
    
    LineString line = SingleGeometryFactory.getFactory().createLineString(coords);
    
    // List of colliders that intersect with <line>:
    List<AsteroidsSprite> intersected = world.intersectLine(line);

    // The nearest AsteroidsSprite:
    AsteroidsSprite nearest = AsteroidsSprite.nearest(loc, intersected);

    // Calculate result if something was detected:
    if (nearest != null) {
      // Normalize the speed of <nearest>:
      double speed = nearest.getSpeed();
      double fraction = speed / baseSpeed;
      result = (fraction > 1.0) ? 0.0 : 1.0-fraction;
    }

    output = result;
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
    output = (charge > 0) ? MAX_OUTPUT : charge;
    output = (charge < 0) ? -MAX_OUTPUT : charge;
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
  Registers a connection between an output device and an input device:
  If a wire is an inhibiter it inverts any signal passing through.
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
  Point getWorldLocation(Point loc, double rotation) {
    // Find the xComponent of the world-translation:
    Coordinate xComponent = new Coordinate(
      Math.cos(rotation) * locationOffset.getX(),
      -Math.sin(rotation) * locationOffset.getX()
      );

    // Find the yComponent of the world-translation:
    Coordinate yComponent = new Coordinate(
      Math.sin(rotation) * locationOffset.getY(),
      Math.cos(rotation) * locationOffset.getY()
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

  // Accessors
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
    sensorPile.add(new SensorRadiusShortDist());
    sensorPile.add(new SensorRayLongDist());
    sensorPile.add(new SensorTriMedDist());

    // Supply the ModulatorPile:
    ModulatorPile.add(new ModulatorIdentity());
    ModulatorPile.add(new ModulatorParabolic());

    // Supply the chassis pile:
    chassisPile.add(makeStarfishChassis());
  }

  BraitenbergVehicle makeVehicleOneRay () {
	    BraitenbergVehicle v = new BraitenbergVehicle(world);
	    
	    // Hardpoint on the Vehicle's Nose:
	    Hardpoint nosePoint = new Hardpoint(new Vector2D(0,-10), 0.0);

	    // Add a sensor to the nose hardpoint:
	    SensorRayLongDist sensorRayDist = new SensorRayLongDist();
	    nosePoint.addSensor(sensorRayDist);

	    // Add the hardpoint to the ship:
	    v.hardpoints.add(nosePoint);

	    // Add a Modulator to the vehicle:
	    ModulatorIdentity mod = new ModulatorIdentity();
	    v.modulators.add(mod);

	    // Wire the hardpoint to the Modulator:
	    Wire sensWire = new Wire(0,0, false, 1.0f);
	    v.sensorWires.add(sensWire);

	    // Wire the modulator to a Control Signal:
	    Wire contWire = new Wire(0,2, false, 1.0f);
	    v.controlWires.add(contWire);
	    
	    return v;
  }
  
  BraitenbergVehicle makeVehicleOneRadius() {
	    BraitenbergVehicle v = new BraitenbergVehicle(world);
	    
	    // Hardpoint on the Vehicle's Nose:
	    Hardpoint nosePoint = new Hardpoint(new Vector2D(0,-10), 0.0);

	    // Add a sensor to the nose hardpoint:
	    SensorRadiusShortDist sensorRadiusDist = new SensorRadiusShortDist();
	    nosePoint.addSensor(sensorRadiusDist);

	    // Add the hardpoint to the ship:
	    v.hardpoints.add(nosePoint);

	    // Add a Modulator to the vehicle:
	    ModulatorIdentity mod = new ModulatorIdentity();
	    v.modulators.add(mod);

	    // Wire the hardpoint to the Modulator:
	    Wire sensWire = new Wire(0,0, false, 1.0f);
	    v.sensorWires.add(sensWire);

	    // Wire the modulator to a Control Signal:
	    Wire contWire = new Wire(0,2, false, 1.0f);
	    v.controlWires.add(contWire);
	    
	    return v;
  }
  
  BraitenbergVehicle makeVehicleOneTriangle() {
	    BraitenbergVehicle v = new BraitenbergVehicle(world);
	    
	    // Hardpoint on the Vehicle's Nose:
	    Hardpoint nosePoint = new Hardpoint(new Vector2D(0,-10), 0.0);

	    // Add a sensor to the nose hardpoint:
	    SensorTriMedDist sensorTriDist = new SensorTriMedDist();
	    nosePoint.addSensor(sensorTriDist);

	    // Add the hardpoint to the ship:
	    v.hardpoints.add(nosePoint);

	    // Add a Modulator to the vehicle:
	    ModulatorIdentity mod = new ModulatorIdentity();
	    v.modulators.add(mod);

	    // Wire the hardpoint to the Modulator:
	    Wire sensWire = new Wire(0,0, false, 1.0f);
	    v.sensorWires.add(sensWire);

	    // Wire the modulator to a Control Signal:
	    Wire contWire = new Wire(0,1, false, 1.0f);
	    v.controlWires.add(contWire);
	    
	    return v;
  }
  
  BraitenbergVehicle makeVehicleTwoARadius () {
	    BraitenbergVehicle v = new BraitenbergVehicle(world);
	    
	    // Hardpoint on the Vehicle's Nose:
	    Hardpoint leftNosePoint = new Hardpoint(new Vector2D(-25,-10), 0.0);
	    Hardpoint rightNosePoint = new Hardpoint(new Vector2D(25,-10), 0.0);

	    // Add a sensor to the nose hardpoints:
	    SensorRadiusShortDist sensorRadiusDist = new SensorRadiusShortDist();
	    leftNosePoint.addSensor(sensorRadiusDist);
	    rightNosePoint.addSensor(sensorRadiusDist);

	    // Add the hardpoint to the ship:
	    v.hardpoints.add(leftNosePoint);
	    v.hardpoints.add(rightNosePoint);

	    // Add a Modulator to the vehicle:
	    ModulatorIdentity leftMod = new ModulatorIdentity();
	    ModulatorIdentity rightMod = new ModulatorIdentity();
	    v.modulators.add(leftMod);
	    v.modulators.add(rightMod);

	    // Wire the hardpoints to Modulators:
	    Wire leftSensWire =   new Wire(0,0, false, 1.0f);
	    Wire rightSensWire =  new Wire(1,1, false, 1.0f);
	    v.sensorWires.add(leftSensWire);
	    v.sensorWires.add(rightSensWire);

	    // Wire the modulator to a Control Signal:
	    Wire leftThrustWire =   new Wire(0,2, false, 1.0f);
	    Wire leftSteerWire =    new Wire(0,1, false, 1.0f);
	    Wire rightThrustWire =  new Wire(1,2, false, 1.0f);
	    Wire rightSteerWire =   new Wire(1,0, false, 1.0f);

	    v.controlWires.add(leftThrustWire);
	    v.controlWires.add(leftSteerWire);
	    v.controlWires.add(rightThrustWire);
	    v.controlWires.add(rightSteerWire);
	    
	    return v;
  }
  
  BraitenbergVehicle makeVehicleTwoATriangle () {
	    BraitenbergVehicle v = new BraitenbergVehicle(world);

	    // Hardpoint on the Vehicle's Nose:
	    Hardpoint leftNosePoint = new Hardpoint(new Vector2D(0,-10), Math.PI/6);
	    Hardpoint rightNosePoint = new Hardpoint(new Vector2D(0,-10), -Math.PI/6);

	    // Add a sensor to the nose hardpoints:
	    SensorTriMedDist sensorTriDist = new SensorTriMedDist();
	    leftNosePoint.addSensor(sensorTriDist);
	    rightNosePoint.addSensor(sensorTriDist);

	    // Add the hardpoint to the ship:
	    v.hardpoints.add(leftNosePoint);
	    v.hardpoints.add(rightNosePoint);

	    // Add a Modulator to the vehicle:
	    ModulatorIdentity leftMod = new ModulatorIdentity();
	    ModulatorIdentity rightMod = new ModulatorIdentity();
	    v.modulators.add(leftMod);
	    v.modulators.add(rightMod);

	    // Wire the hardpoints to Modulators:
	    Wire leftSensWire =   new Wire(0,0, false, 1.0f);
	    Wire rightSensWire =  new Wire(1,1, false, 1.0f);
	    v.sensorWires.add(leftSensWire);
	    v.sensorWires.add(rightSensWire);

	    // Wire the modulator to a Control Signal:
	    Wire leftThrustWire =   new Wire(0,2, false, 1.0f);
	    Wire leftSteerWire =    new Wire(0,1, false, 1.0f);
	    Wire rightThrustWire =  new Wire(1,2, false, 1.0f);
	    Wire rightSteerWire =   new Wire(1,0, false, 1.0f);

	    v.controlWires.add(leftThrustWire);
	    v.controlWires.add(leftSteerWire);
	    v.controlWires.add(rightThrustWire);
	    v.controlWires.add(rightSteerWire);
	    
	    return v;
  }
  
  BraitenbergVehicle makeVehicleTwoBRadius () {
	    BraitenbergVehicle v = new BraitenbergVehicle(world);

	    // Hardpoint on the Vehicle's Nose:
	    SensorRadiusShortDist sensorRadiusDist = new SensorRadiusShortDist();
	    Hardpoint leftNosePoint = new Hardpoint(new Vector2D(-25,-10), 0.0);
	    Hardpoint rightNosePoint = new Hardpoint(new Vector2D(25,-10), 0.0);

	    // Add a sensor to the nose hardpoints:
	    leftNosePoint.addSensor(sensorRadiusDist);
	    rightNosePoint.addSensor(sensorRadiusDist);

	    // Add the hardpoint to the ship:
	    v.hardpoints.add(leftNosePoint);
	    v.hardpoints.add(rightNosePoint);

	    // Add a Modulator to the vehicle:
	    ModulatorIdentity leftMod = new ModulatorIdentity();
	    ModulatorIdentity rightMod = new ModulatorIdentity();
	    v.modulators.add(leftMod);
	    v.modulators.add(rightMod);

	    // Wire the hardpoints to Modulators:
	    Wire leftSensWire = new   Wire(0,0, false, 1.0f);
	    Wire rightSensWire = new  Wire(1,1, false, 1.0f);
	    v.sensorWires.add(leftSensWire);
	    v.sensorWires.add(rightSensWire);

	    // Wire the modulator to a Control Signal:
	    Wire leftThrustWire =   new Wire(0,2, false, 1.0f);
	    Wire leftSteerWire =    new Wire(0,0, false, 1.0f);
	    Wire rightThrustWire =  new Wire(1,2, false, 1.0f);
	    Wire rightSteerWire =   new Wire(1,1, false, 1.0f);

	    v.controlWires.add(leftThrustWire);
	    v.controlWires.add(leftSteerWire);
	    v.controlWires.add(rightThrustWire);
	    v.controlWires.add(rightSteerWire);
	    
	    return v;
  }
  
  BraitenbergVehicle makeVehicleTwoBTriangle () {
	    BraitenbergVehicle v = new BraitenbergVehicle(world);

	    // Hardpoint on the Vehicle's Nose:
	    Hardpoint leftNosePoint = new Hardpoint(new Vector2D(0,-10), Math.PI/6);
	    Hardpoint rightNosePoint = new Hardpoint(new Vector2D(0,-10), -Math.PI/6);

	    // Add a sensor to the nose hardpoints:
	    SensorTriMedDist sensorTriDist = new SensorTriMedDist();
	    leftNosePoint.addSensor(sensorTriDist);
	    rightNosePoint.addSensor(sensorTriDist);

	    // Add the hardpoint to the ship:
	    v.hardpoints.add(leftNosePoint);
	    v.hardpoints.add(rightNosePoint);

	    // Add a Modulator to the vehicle:
	    ModulatorIdentity leftMod = new ModulatorIdentity();
	    ModulatorIdentity rightMod = new ModulatorIdentity();
	    v.modulators.add(leftMod);
	    v.modulators.add(rightMod);

	    // Wire the hardpoints to Modulators:
	    Wire leftSensWire = new   Wire(0,0, false, 1.0f);
	    Wire rightSensWire = new  Wire(1,1, false, 1.0f);
	    v.sensorWires.add(leftSensWire);
	    v.sensorWires.add(rightSensWire);

	    // Wire the modulator to a Control Signal:
	    Wire leftThrustWire =   new Wire(0,2, false, 1.0f);
	    Wire leftSteerWire =    new Wire(0,0, false, 1.0f);
	    Wire rightThrustWire =  new Wire(1,2, false, 1.0f);
	    Wire rightSteerWire =   new Wire(1,1, false, 1.0f);

	    v.controlWires.add(leftThrustWire);
	    v.controlWires.add(leftSteerWire);
	    v.controlWires.add(rightThrustWire);
	    v.controlWires.add(rightSteerWire);
	    
	    return v;
  }
  
  BraitenbergVehicle makeVehicleThreeARadius () {
	    BraitenbergVehicle v = new BraitenbergVehicle(world);
	    
	    // Hardpoint on the Vehicle's Nose:
	    Hardpoint leftNosePoint = new Hardpoint(new Vector2D(-25,-10), 0.0);
	    Hardpoint rightNosePoint = new Hardpoint(new Vector2D(25,-10), 0.0);

	    // Add a sensor to the nose hardpoints:
	    SensorRadiusShortDist sensorRadiusDist = new SensorRadiusShortDist();
	    leftNosePoint.addSensor(sensorRadiusDist);
	    rightNosePoint.addSensor(sensorRadiusDist);

	    // Add the hardpoint to the ship:
	    v.hardpoints.add(leftNosePoint);
	    v.hardpoints.add(rightNosePoint);

	    // Add a Modulator to the vehicle:
	    ModulatorIdentity leftMod = new ModulatorIdentity();
	    ModulatorIdentity rightMod = new ModulatorIdentity();
	    v.modulators.add(leftMod);
	    v.modulators.add(rightMod);

	    // Wire the hardpoints to Modulators:
	    Wire leftSensWire =   new Wire(0,0, true, 1.0f);
	    Wire rightSensWire =  new Wire(1,1, true, 1.0f);
	    v.sensorWires.add(leftSensWire);
	    v.sensorWires.add(rightSensWire);

	    // Wire the modulator to a Control Signal:
	    Wire leftThrustWire =   new Wire(0,2, false, 1.0f);
	    Wire leftSteerWire =    new Wire(0,1, false, 1.0f);
	    Wire rightThrustWire =  new Wire(1,2, false, 1.0f);
	    Wire rightSteerWire =   new Wire(1,0, false, 1.0f);

	    v.controlWires.add(leftThrustWire);
	    v.controlWires.add(leftSteerWire);
	    v.controlWires.add(rightThrustWire);
	    v.controlWires.add(rightSteerWire);
	    
	    return v;
  }
  
  BraitenbergVehicle makeVehicleThreeBRadius () {
	    BraitenbergVehicle v = new BraitenbergVehicle(world);

	    // Hardpoint on the Vehicle's Nose:
	    Hardpoint leftNosePoint = new Hardpoint(new Vector2D(-25,-10), 0.0);
	    Hardpoint rightNosePoint = new Hardpoint(new Vector2D(25,-10), 0.0);

	    // Add a sensor to the nose hardpoints:
	    SensorRadiusShortDist sensorRadiusDist = new SensorRadiusShortDist();
	    leftNosePoint.addSensor(sensorRadiusDist);
	    rightNosePoint.addSensor(sensorRadiusDist);

	    // Add the hardpoint to the ship:
	    v.hardpoints.add(leftNosePoint);
	    v.hardpoints.add(rightNosePoint);

	    // Add a Modulator to the vehicle:
	    ModulatorIdentity leftMod = new ModulatorIdentity();
	    ModulatorIdentity rightMod = new ModulatorIdentity();
	    v.modulators.add(leftMod);
	    v.modulators.add(rightMod);

	    // Wire the hardpoints to Modulators:
	    Wire leftSensWire =   new Wire(0,0, true, 1.0f);
	    Wire rightSensWire =  new Wire(1,1, true, 1.0f);
	    v.sensorWires.add(leftSensWire);
	    v.sensorWires.add(rightSensWire);

	    // Wire the modulator to a Control Signal:
	    Wire leftThrustWire =   new Wire(0,2, false, 1.0f);
	    Wire leftSteerWire =    new Wire(0,0, false, 1.0f);
	    Wire rightThrustWire =  new Wire(1,2, false, 1.0f);
	    Wire rightSteerWire =   new Wire(1,1, false, 1.0f);

	    v.controlWires.add(leftThrustWire);
	    v.controlWires.add(leftSteerWire);
	    v.controlWires.add(rightThrustWire);
	    v.controlWires.add(rightSteerWire);
	    
	    return v;
  }
  
  BraitenbergVehicle makeVehicleThreeATriangle () {
	    BraitenbergVehicle v = new BraitenbergVehicle(world);

	    // Hardpoint on the Vehicle's Nose:
	    Hardpoint leftNosePoint = new Hardpoint(new Vector2D(0,-10), Math.PI/6);
	    Hardpoint rightNosePoint = new Hardpoint(new Vector2D(0,-10), -Math.PI/6);

	    // Add a sensor to the nose hardpoints:
	    SensorTriMedDist sensorTriDist = new SensorTriMedDist();
	    leftNosePoint.addSensor(sensorTriDist);
	    rightNosePoint.addSensor(sensorTriDist);

	    // Add the hardpoint to the ship:
	    v.hardpoints.add(leftNosePoint);
	    v.hardpoints.add(rightNosePoint);

	    // Add a Modulator to the vehicle:
	    ModulatorIdentity leftMod = new ModulatorIdentity();
	    ModulatorIdentity rightMod = new ModulatorIdentity();
	    v.modulators.add(leftMod);
	    v.modulators.add(rightMod);

	    // Wire the hardpoints to Modulators:
	    Wire leftSensWire =   new Wire(0,0, true, 1.0f);
	    Wire rightSensWire =  new Wire(1,1, true, 1.0f);
	    v.sensorWires.add(leftSensWire);
	    v.sensorWires.add(rightSensWire);

	    // Wire the modulator to a Control Signal:
	    Wire leftThrustWire =   new Wire(0,2, false, 1.0f);
	    Wire leftSteerWire =    new Wire(0,1, false, 1.0f);
	    Wire rightThrustWire =  new Wire(1,2, false, 1.0f);
	    Wire rightSteerWire =   new Wire(1,0, false, 1.0f);

	    v.controlWires.add(leftThrustWire);
	    v.controlWires.add(leftSteerWire);
	    v.controlWires.add(rightThrustWire);
	    v.controlWires.add(rightSteerWire);
	    
	    return v;
}
  
  BraitenbergVehicle makeVehicleThreeBTriangle () {
	    BraitenbergVehicle v = new BraitenbergVehicle(world);

	    // Hardpoint on the Vehicle's Nose:
	    Hardpoint leftNosePoint = new Hardpoint(new Vector2D(0,-10), Math.PI/6);
	    Hardpoint rightNosePoint = new Hardpoint(new Vector2D(0,-10), -Math.PI/6);

	    // Add a sensor to the nose hardpoints:
	    SensorTriMedDist sensorTriDist = new SensorTriMedDist();
	    leftNosePoint.addSensor(sensorTriDist);
	    rightNosePoint.addSensor(sensorTriDist);

	    // Add the hardpoint to the ship:
	    v.hardpoints.add(leftNosePoint);
	    v.hardpoints.add(rightNosePoint);

	    // Add a Modulator to the vehicle:
	    ModulatorIdentity leftMod = new ModulatorIdentity();
	    ModulatorIdentity rightMod = new ModulatorIdentity();
	    v.modulators.add(leftMod);
	    v.modulators.add(rightMod);

	    // Wire the hardpoints to Modulators:
	    Wire leftSensWire =   new Wire(0,0, true, 1.0f);
	    Wire rightSensWire =  new Wire(1,1, true, 1.0f);
	    v.sensorWires.add(leftSensWire);
	    v.sensorWires.add(rightSensWire);

	    // Wire the modulator to a Control Signal:
	    Wire leftThrustWire =   new Wire(0,2, false, 1.0f);
	    Wire leftSteerWire =    new Wire(0,0, false, 1.0f);
	    Wire rightThrustWire =  new Wire(1,2, false, 1.0f);
	    Wire rightSteerWire =   new Wire(1,1, false, 1.0f);

	    v.controlWires.add(leftThrustWire);
	    v.controlWires.add(leftSteerWire);
	    v.controlWires.add(rightThrustWire);
	    v.controlWires.add(rightSteerWire);
	    
	    return v;
}
  
  BraitenbergVehicle makeVehicleFourATriangle () {
	    BraitenbergVehicle v = new BraitenbergVehicle(world);

	    // Hardpoint on the Vehicle's Nose:
	    Hardpoint leftNosePoint = new Hardpoint(new Vector2D(0,-10), Math.PI/6);
	    Hardpoint rightNosePoint = new Hardpoint(new Vector2D(0,-10), -Math.PI/6);

	    // Add a sensor to the nose hardpoints:
	    SensorTriMedDist sensorTriDist = new SensorTriMedDist();
	    leftNosePoint.addSensor(sensorTriDist);
	    rightNosePoint.addSensor(sensorTriDist);

	    // Add the hardpoint to the ship:
	    v.hardpoints.add(leftNosePoint);
	    v.hardpoints.add(rightNosePoint);

	    // Add a Modulator to the vehicle:
	    ModulatorParabolic leftMod = new ModulatorParabolic();
	    ModulatorParabolic rightMod = new ModulatorParabolic();
	    v.modulators.add(leftMod);
	    v.modulators.add(rightMod);

	    // Wire the hardpoints to Modulators:
	    Wire leftSensWire =   new Wire(0,0, true, 1.0f);
	    Wire rightSensWire =  new Wire(1,1, true, 1.0f);
	    v.sensorWires.add(leftSensWire);
	    v.sensorWires.add(rightSensWire);

	    // Wire the modulator to a Control Signal:
	    Wire leftThrustWire =   new Wire(0,2, false, 1.0f);
	    Wire leftSteerWire =    new Wire(0,0, false, 1.0f);
	    Wire rightThrustWire =  new Wire(1,2, false, 1.0f);
	    Wire rightSteerWire =   new Wire(1,1, false, 1.0f);

	    v.controlWires.add(leftThrustWire);
	    v.controlWires.add(leftSteerWire);
	    v.controlWires.add(rightThrustWire);
	    v.controlWires.add(rightSteerWire);
	    
	    return v;
  }
  
  
  // A vehicle with sensors arranged to approximate a basic eye:
  BraitenbergVehicle makeVehicleRayEye () {
    BraitenbergVehicle v = new BraitenbergVehicle(world);
    
    SensorRayLongDist sensor = new SensorRayLongDist();
    
    int numRays = 16; // Should be even number.
    double raySpread = 2 * Math.PI / numRays; 
    
    // Create HardPoints arranged like spokes on a wheel:
    for (int i = 0; i < numRays; i++ ) {
      Hardpoint h = new Hardpoint(new Vector2D(0,0), i * raySpread);
      
      // Add a sensor to the hardpoint:
      h.addSensor(sensor);
      
      // Add the hardpoint to the chassis:
      v.hardpoints.add(h);
      
      // Add Modulators to the vehicle:
      ModulatorIdentity mod = new ModulatorIdentity();
      v.modulators.add(mod);
      
      // Wire the hardpoint to the Modulator:
      Wire sensWire = new Wire(i,i, false, 1.0f);
      v.sensorWires.add(sensWire);

    }
    
    // We now have 16 sensors wired into the vehicle in a spoke-configuration.
    // The wire i maps to the sensor pointing to raySpread*i (counter-clockwise).
    
    // Flee from objects behind:
    Wire wFlee = new Wire(numRays/2,2, false, 1f);
    v.controlWires.add(wFlee);
    
    // Thrust except when objects are near:
    Wire wThrust1 = new Wire(0,        2, true, 5f);
    Wire wThrust2 = new Wire(1,        2, true, 5f);
    Wire wThrust3 = new Wire(numRays-1,2, true, 5f);
    v.controlWires.add(wThrust1);
    v.controlWires.add(wThrust2);
    v.controlWires.add(wThrust3);
    
    // Shoot objects passing in front:
    Wire wShoot1 = new Wire(0,        4, false, 1f);
    Wire wShoot2 = new Wire(1,        4, false, 1f);
    Wire wShoot3 = new Wire(numRays-1,4, false, 1f);
    v.controlWires.add(wShoot1);
    v.controlWires.add(wShoot2);
    v.controlWires.add(wShoot3);
    
    // Turn toward objects on the left:
    for (int k = 1; k < numRays/2; k++) {
      Wire wLeft = new Wire(k,0, false, 1f);
      v.controlWires.add(wLeft);
    }
    
    // Turn toward objects on the right:
    for (int j = numRays/2+1; j < numRays; j++) {
      Wire wLeft = new Wire(j,1, false, 1f);
      v.controlWires.add(wLeft);
    }
    
    return v;
}
  
  
  // TODO: Improve comments here...
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

  // TODO: Improve comments here...
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

  // TODO: Improve comments here...
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

  // TODO: Improve comments here...
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

/*
  Provides a structured interface for reading the game's state.
*/
class GameState {
  AsteroidsSprite ship = null;
  AsteroidsSprite ufo = null;
  AsteroidsSprite missle = null;
  AsteroidsSprite[] photons = null;
  AsteroidsSprite[] asteroids = null;
  Dimension screen = null;

  // Constructor
  public GameState( AsteroidsSprite ship,
                    AsteroidsSprite[] photons,
                    AsteroidsSprite[] asteroids,
                    AsteroidsSprite missle,
                    AsteroidsSprite ufo) {
    this.ship = ship;
    this.photons = photons;
    this.asteroids = asteroids;
    this.missle = missle;
    this.ufo = ufo;
  }

  // Accessors
  public AsteroidsSprite getShip() {
    AsteroidsSprite copyShip = (ship == null) ? null : new AsteroidsSprite(ship);
    return copyShip;
  }

  public void setShip(AsteroidsSprite ship) { this.ship = ship; }

  // Return an ArrayList of all active colliders:
  // Anything which may collide with the Ship is a collider.
  public List<AsteroidsSprite> getActiveColliders() {
    List<AsteroidsSprite> activeColliders = new ArrayList<AsteroidsSprite>();

    // Add all active colliders to <activeColliders>:
    for (int i = 0; i != asteroids.length; i++) {
      if (asteroids[i].active) {
        activeColliders.add(asteroids[i]);
      }
    }
    
    if (ufo.active)     { activeColliders.add(ufo); }
    if (missle.active)  { activeColliders.add(missle); }

    return activeColliders;
  }

  // Return the AsteroidsSprites which intersect a circle defined by 
  // <loc, radius>:
  public List<AsteroidsSprite> intersectRadius(double radius, Point loc) {
    // List of the colliders we inspect:
    List<AsteroidsSprite> activeColliders = this.getActiveColliders();

    // List of colliders within <loc, radius>:
    List<AsteroidsSprite> intersected = new ArrayList<AsteroidsSprite>();

    // Check for intersection between <loc, radius> and every active-collider:
    for (AsteroidsSprite cur : activeColliders) {

        // If distance between <cur> and <loc> is < radius:
        if (loc.distance(cur.sprite) <= radius) {
          intersected.add(cur);
        }
    }

    return intersected;
  }

  // Return the AsteroidsSprites which intersect a line defined by <line>:
  public List<AsteroidsSprite> intersectLine(LineString line) {
    // List of the colliders we inspect:
    List<AsteroidsSprite> activeColliders = this.getActiveColliders();

    // List of colliders intersected by <line>:
    List<AsteroidsSprite> intersected = new ArrayList<AsteroidsSprite>();

    // Check for intersection between <line> and every active-collider:
    for (AsteroidsSprite cur : activeColliders) {

      // If <line> intersects <cur>:
      // Add <cur> to the list of intersected colliders.
      if (cur.sprite.intersects(line) ) {
        intersected.add(cur);
      }
    }

    return intersected;
  }

  // Return the AsteroidsSprites which intersect a triangle defined by 
  // <locA, locB, locC>:
  public List<AsteroidsSprite> intersectTri(Polygon tri) {
    // List of the colliders we inspect:
    List<AsteroidsSprite> activeColliders = this.getActiveColliders();

    // List of colliders intersected by <triangle>:
    List<AsteroidsSprite> intersected = new ArrayList<AsteroidsSprite>();

    // Check for intersection between <triangle> and every active-collider:
    for (AsteroidsSprite cur : activeColliders) {

      // If <tri> intersects <cur>:
      // Add <cur> to the list of intersected colliders.
      if (tri.intersects(cur.sprite)) {
        intersected.add(cur);
      }
    }

    return intersected;
  }

  // Does <p1> intersect <p2>?
  public static boolean intersects(Polygon p1, Polygon p2) {
	  return p1.intersects(p2);
  }

  // Does <line> intersect <poly>?
  // Based on: http://bit.ly/1qIdDkf
  public static boolean intersects(final Polygon poly, LineString line) {//final Line2D.Double line) {
	  
	return poly.intersects(line);
  }
}

/*
  Wraps JST Polygon with methods to provide interoperability between
  JST Polygon and the JavaStandardLibrary Polygon class:
*/
@SuppressWarnings("serial")

class PolyWrapper extends Polygon {
  
  public PolyWrapper(Polygon p) {
     super(
         (LinearRing)p.getExteriorRing(), 
         (LinearRing[]) null, 
         SingleGeometryFactory.getFactory());
  }

  // Return the X or Y coordinates of this Polygon's
  // vertices.
  public int[] getPoints(char dimension) {
    int[] points = new int[this.getNumPoints()];
    Coordinate[] coords = this.getCoordinates();
    
    for (int i = 0; i < points.length; i++) {
      if (dimension == 'x') { points[i] = (int)coords[i].x; }
      if (dimension == 'y') { points[i] = (int)coords[i].y; }
    }
    
    return points;
  }
  
  public int[] xPoints() { return this.getPoints('x'); }
  public int[] yPoints() { return this.getPoints('y'); }
}


/******************************************************************************
  The AsteroidsSprite class defines a game object, including it's shape,
  position, movement and rotation. It also can determine if two objects collide.
******************************************************************************/

class AsteroidsSprite {
  // Fields:
  
  static int width;          // Dimensions of the graphics area.
  static int height;

  PolyWrapper shape;         // Base sprite shape, centered at the origin (0,0).
  boolean active;            // Active flag.
  double  angle;             // Current angle of rotation.
  double  deltaAngle;        // Amount to change the rotation angle.
  double  x, y;              // Current position on screen.
  double  deltaX, deltaY;    // Amount to change the screen position.
  PolyWrapper sprite;        // Final location and shape of sprite after
                             // applying rotation and translation to get screen
                             // position. Used for drawing on the screen and in
                             // detecting collisions.

  // Constructors:

  public AsteroidsSprite() {

    this.shape = new PolyWrapper(SingleGeometryFactory.getFactory().createPolygon(
        new Coordinate[] {
            new Coordinate(0,0), 
            new Coordinate(1,1),
            new Coordinate(1,0),
            new Coordinate(0,0)}) );
    this.active = false;
    this.angle = 0.0;
    this.deltaAngle = 0.0;
    this.x = 0.0;
    this.y = 0.0;
    this.deltaX = 0.0;
    this.deltaY = 0.0;
    this.sprite =  new PolyWrapper( SingleGeometryFactory.getFactory().createPolygon(
        new Coordinate[] {
            new Coordinate(0,0), 
            new Coordinate(1,1),
            new Coordinate(1,0),
            new Coordinate(0,0)}) );
  }

  // Copy constructor
  public AsteroidsSprite(AsteroidsSprite p) {
    this.shape = new PolyWrapper( (Polygon)SingleGeometryFactory.getFactory().createGeometry(p.shape) );
    this.active = p.active;
    this.angle = p.angle;
    this.deltaAngle = p.deltaAngle;
    this.x = p.x;
    this.y = p.y;
    this.deltaX = p.deltaX;
    this.deltaY = p.deltaY;
    this.sprite =  new PolyWrapper( (Polygon)SingleGeometryFactory.getFactory().createGeometry(p.sprite));
  }

  // Methods:

  public boolean advance() {

    boolean wrapped;

    // Update the rotation and position of the sprite based on the delta
    // values. If the sprite moves off the edge of the screen, it is wrapped
    // around to the other side and TRUE is returned.

    this.angle += this.deltaAngle;
    if (this.angle < 0)
      this.angle += 2 * Math.PI;
    if (this.angle > 2 * Math.PI)
      this.angle -= 2 * Math.PI;
    wrapped = false;
    this.x += this.deltaX;
    if (this.x < -width / 2) {
      this.x += width;
      wrapped = true;
    }
    if (this.x > width / 2) {
      this.x -= width;
      wrapped = true;
    }
    this.y -= this.deltaY;
    if (this.y < -height / 2) {
      this.y += height;
      wrapped = true;
    }
    if (this.y > height / 2) {
      this.y -= height;
      wrapped = true;
    }

    return wrapped;
  }

  public void render() {

    // Render the sprite's shape and location by rotating it's base shape and
    // moving it to it's proper screen position.
    
    Coordinate[] coords = new Coordinate[this.shape.getNumPoints()];
    for (int i = 0; i < this.shape.getNumPoints(); i++) {
      
      Double x =  (double) Math.round( 
                              this.shape.xPoints()[i] * Math.cos(this.angle) + 
                              this.shape.yPoints()[i] * Math.sin(this.angle) ) + 
                              Math.round(this.x) + width / 2;
      
      Double y = (double) Math.round(
                            this.shape.yPoints()[i] * Math.cos(this.angle) -
                            this.shape.xPoints()[i] * Math.sin(this.angle) ) +
                            Math.round(this.y) + height / 2;
      
      coords[i] = new Coordinate(x,y);
    }
    
    this.sprite =  new PolyWrapper(SingleGeometryFactory.getFactory().createPolygon(coords));
  }

  //Determine if one sprite overlaps with another, i.e., if any vertices
  // of one sprite lands inside the other.
  public boolean isColliding(AsteroidsSprite s) { return s.sprite.intersects(this.sprite); }

  // Return the AsteroidsSprite in <list> nearest to <sprite>:
  public static AsteroidsSprite nearest(List<AsteroidsSprite> list, AsteroidsSprite sprite) {
    AsteroidsSprite nearest = null;
    double distanceToNearest = Double.MAX_VALUE;
    double curDistance = 0;
    
    for (AsteroidsSprite cur : list) {
      curDistance = DistanceOp.distance(sprite.sprite, cur.sprite);
      
      if (curDistance < distanceToNearest) {
        distanceToNearest = curDistance;
      }
    }
    
    return nearest;
  }
  
  // Return the AsteroidsSprite in <list> nearest to <loc>:
  public static AsteroidsSprite nearest(Point loc, List<AsteroidsSprite> list) {
    double distanceToNearest = Double.MAX_VALUE;
    AsteroidsSprite nearest = null;

    double curDistance;
    for (AsteroidsSprite cur : list) {
      curDistance = loc.distance(cur.sprite);

      // If <cur> is the closest AsteroidsSprite yet:
      if (curDistance < distanceToNearest) {
        distanceToNearest = curDistance;
        nearest = cur;
      }
    }

    return nearest;
  }

  public Point getLocation() { 
    return SingleGeometryFactory.getFactory().createPoint(new Coordinate(x + AsteroidsSprite.width/2, y + AsteroidsSprite.height/2));
  }

  public double getSpeed() {
    double speed = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    return speed;
  }
}

/******************************************************************************
  Main applet code.
******************************************************************************/

public class Asteroids extends Applet implements Runnable, KeyListener {

  private static final long serialVersionUID = 1L;
  
  public static Frame frame = null;

  // Structured collection of AsteroidsSprites.
  // Used by agent's to provide awareness of game state.
  // MAXCOLLIDERS = MAX_ROCKS + 1 UFO + 1 MISSILE
  GameState currentState;

  // Testing information.
  // Used for performing tests and gathering statistics.
  
  // Records details of an agent's life.
  private List<String> lifeData = new ArrayList<String>(); 
  
  static final int STATISTICS_POLLRATE = 2; // Record statistics every n frames.
  
  public enum Stat {
    SENSOR0SIGNAL,
    FRAME
  }

  // Agent Variables.

  static final double CONTROL_SCALING = 2.0; // Amount to scale pilot's control signals by.

  BRVFactory factory;
  BraitenbergVehicle pilot;

  // Debug information

  static final int MAX_DEBUG_LINES = 10;
  boolean debugging = false;
  boolean sensorsShown = false;
  ArrayList<String> debugInfo = new ArrayList<String>();

  
  // Menu Information.
  
  ArrayList<String> menuInfo = new ArrayList<String>();
  String menuInput;
  
  
  // Copyright information.

  String copyName = "Asteroids";
  String copyVers = "Version 1.3";
  String copyInfo = "Copyright 1998-2001 by Mike Hall";
  String copyLink = "http://www.brainjar.com";
  String copyText = copyName + '\n' + copyVers + '\n'
                  + copyInfo + '\n' + copyLink;
  

  // Thread control variables.

  Thread loopThread;

  // Constants

  static final int FRAME_LENGTH = 1000;     // Milliseconds in one frame.
  static final int DELAY = 15;              // Milliseconds between screen and
  static final int FPS   =                  // the resulting frame rate.
    Math.round(FRAME_LENGTH / DELAY);

  static final int MAX_SHOTS =  12;         // Maximum number of sprites
  static final int MAX_ROCKS =  1;         // for photons, asteroids and
  static final int MAX_SCRAP = 40;          // explosions.
  
  int maxSpawnableRocks;

  static final int SCRAP_COUNT  = 2 * FPS;  // Timer counter starting values
  static final int HYPER_COUNT  = 3 * FPS;  // calculated using number of
  static final int MISSLE_COUNT = 4 * FPS;  // seconds x frames per second.
  static final int STORM_PAUSE  = 2 * FPS;

  static final int    MIN_ROCK_SIDES =   6; // Ranges for asteroid shape, size
  static final int    MAX_ROCK_SIDES =  16; // speed and rotation.
  static final int    MIN_ROCK_SIZE  =  20;
  static final int    MAX_ROCK_SIZE  =  40;
  static final double MIN_ROCK_SPEED =  40.0 / FPS;
  static final double MAX_ROCK_SPEED = 240.0 / FPS;
  static final double MAX_ROCK_SPIN  = Math.PI / FPS;

  static final int MAX_SHIPS = 3;           // Starting number of ships for
                                            // each game.
  static final int UFO_PASSES = 3;          // Number of passes for flying
                                            // saucer per appearance.

  // Ship's rotation and acceleration rates and maximum speed.

  static final double SHIP_ANGLE_STEP = Math.PI / FPS;
  static final double SHIP_SPEED_STEP = 15.0 / FPS;
  static final double MAX_SHIP_SPEED  = 1.5 * MAX_ROCK_SPEED;
  static final double FRICTION        = .95; // Applies only to the ship.

  static final int FIRE_DELAY = 50;         // Minimum number of milliseconds
                                            // required between photon shots.

  // Probability of flying saucer firing a missile during any given frame
  // (other conditions must be met).

  static final double MISSLE_PROBABILITY = 0.45 / FPS;

  static final int BIG_POINTS    =  25;     // Points scored for shooting
  static final int SMALL_POINTS  =  50;     // various objects.
  static final int UFO_POINTS    = 250;
  static final int MISSLE_POINTS = 500;

  // Number of points the must be scored to earn a new ship or to cause the
  // flying saucer to appear.

  static final int NEW_SHIP_POINTS = 4000;
  static final int NEW_UFO_POINTS  = 3000;

  // Background stars.

  int     numStars;
  Point[] stars;

  // Game data.

  int score;
  int highScore;
  int newShipScore;
  int newUfoScore;

  // Flags for game state and options.

  boolean paused;
  boolean playing;
  boolean detail;

  // Control Variables:
  // Read from <pilot> during UpdateControl(), acted on during UpdateShip()

  // Range: [0,1] where 1 => Maximum turn-speed/effect.

  double turnLeft = 0;
  double turnRight = 0;
  double accForward = 0;
  double accBackward = 0;

  boolean firePhoton = false;
  boolean fireHyper = false;

  // Sprite objects.

  AsteroidsSprite   ship;
  AsteroidsSprite   fwdThruster, revThruster;
  AsteroidsSprite   ufo;
  AsteroidsSprite   missle;
  AsteroidsSprite[] photons    = new AsteroidsSprite[MAX_SHOTS];
  AsteroidsSprite[] asteroids  = new AsteroidsSprite[MAX_ROCKS];
  AsteroidsSprite[] explosions = new AsteroidsSprite[MAX_SCRAP];

  // Ship data.

  int shipsLeft;       // Number of ships left in game, including current one.
  int shipCounter;     // Timer counter for ship explosion.
  int hyperCounter;    // Timer counter for hyperspace.

  // Photon data.

  int   photonIndex;    // Index to next available photon sprite.
  long  photonTime;     // Time value used to keep firing rate constant.

  // Flying saucer data.

  int ufoPassesLeft;    // Counter for number of flying saucer passes.
  int ufoCounter;       // Timer counter used to track each flying saucer pass.

  // Missile data.

  int missleCounter;    // Counter for life of missile.

  // Asteroid data.

  boolean[] asteroidIsSmall = new boolean[MAX_ROCKS];    // Asteroid size flag.
  int       asteroidsCounter;                            // Break-time counter.
  double    asteroidsSpeed;                              // Asteroid speed.
  int       asteroidsLeft;                               // Number of active asteroids.

  // Explosion data.

  int[] explosionCounter = new int[MAX_SCRAP];  // Time counters for explosions.
  int   explosionIndex;                         // Next available explosion sprite.

  // Off screen image.

  Dimension offDimension;
  Image     offImage;
  Graphics  offGraphics;

  // Data for the screen font.

  Font font      = new Font("monospaced", Font.PLAIN, 14);
  FontMetrics fm = getFontMetrics(font);
  int fontWidth  = fm.getMaxAdvance();
  int fontHeight = fm.getHeight();

  public String getAppletInfo() {

    // Return copyright information.

    return(copyText);
  }

  // Perform all necessary one-time instantiation:
  public void init() {
    
    menuInput = "";
    
    // Initialize menu info:
    menuInfo.add("Braitenberg Pilots");
    menuInfo.add("");
    menuInfo.add("Enter a two-character key and press 'Enter' to see that vehicle play the game:");
    menuInfo.add("");
    menuInfo.add("Braitenberg-Vehicles   |   Fit-Vehicles");
    menuInfo.add("Key   Pilot            |   Key   Pilot");
    menuInfo.add("-----------------------|--------------------");
    menuInfo.add("1R    1  Radius        |   1F    Ray Eye");
    menuInfo.add("1L    1  Laser         |   2F");
    menuInfo.add("2A    2A Triangle      |   3F");
    menuInfo.add("2B    2B Triangle      |   4F");
    menuInfo.add("3A    3A Triangle      |   5F");
    menuInfo.add("3B    3B Triangle      |   6F");
    menuInfo.add("4A    4A Triangle      |   7F");
    menuInfo.add("");
    menuInfo.add("Press 'P' at any time to Pause.");
    menuInfo.add("Press 'K' at any time to End the current game.");
    menuInfo.add("Press 'Enter' to begin or clear current input.");
    menuInfo.add("");
    menuInfo.add("Input:");
    menuInfo.add("");
    menuInfo.add("");
    menuInfo.add("Original Engine:");
    menuInfo.add(copyName);
    menuInfo.add(copyVers);
    menuInfo.add(copyInfo);
    menuInfo.add(copyLink);

    // Set up key event handling and set focus to applet window.

    addKeyListener(this);
    requestFocus();
    resizeScreen();

    // Create shape for the ship sprite.

    ship = new AsteroidsSprite();
    Coordinate[] shipCoords = {
      new Coordinate(0, -10),
      new Coordinate(7, 10),
      new Coordinate(-7, 10),
      new Coordinate(0, -10)
    };
    ship.shape = new PolyWrapper(SingleGeometryFactory.getFactory().createPolygon(shipCoords));

    // Create shapes for the ship thrusters.

    fwdThruster = new AsteroidsSprite();
    Coordinate[] fwdCoords = {
      new Coordinate(0, 12),
      new Coordinate(-3, 16),
      new Coordinate(0, 26),
      new Coordinate(3, 16),
      new Coordinate(0, 12)
    };
    fwdThruster.shape = new PolyWrapper(SingleGeometryFactory.getFactory().createPolygon(fwdCoords));
    
    revThruster = new AsteroidsSprite();
    Coordinate[] revCoords = {
      new Coordinate(-2, 12),
      new Coordinate(-4, 14),
      new Coordinate(-2, 20),
      new Coordinate(0, 14),
      new Coordinate(2, 12),
      new Coordinate(4, 14),
      new Coordinate(2, 20),
      new Coordinate(0, 14),
      new Coordinate(-2, 12)
    };
    revThruster.shape = new PolyWrapper(SingleGeometryFactory.getFactory().createPolygon(revCoords));
    

    // Create shape for each photon sprites.

    for (int i = 0; i < MAX_SHOTS; i++) {
      photons[i] = new AsteroidsSprite();
      Coordinate[] photonCoord = {
        new Coordinate(1, 1),
        new Coordinate(1, -1),
        new Coordinate(-1, -1),
        new Coordinate(-1, 1),
        new Coordinate(1, 1)
      };
      
      photons[i].shape = new PolyWrapper(SingleGeometryFactory.getFactory().createPolygon(photonCoord));
    }

    // Create shape for the flying saucer.
    ufo = new AsteroidsSprite();
    Coordinate[] ufoCoords = {
        new Coordinate(-15, 0),
        new Coordinate(-10, -5),
        new Coordinate(-5, -5),
        new Coordinate(-5, -8),
        new Coordinate(5, -8),
        new Coordinate(5, -5),
        new Coordinate(10, -5),
        new Coordinate(15, 0),
        new Coordinate(10, 5),
        new Coordinate(-10, 5),
        new Coordinate(-15, 0)
      };
    ufo.shape = new PolyWrapper(SingleGeometryFactory.getFactory().createPolygon(ufoCoords));

    // Create shape for the guided missile.

    missle = new AsteroidsSprite();
    
    Coordinate[] missleCoords = {
        new Coordinate(0, -4),
        new Coordinate(1, -3),
        new Coordinate(1, 3),
        new Coordinate(2, 4),
        new Coordinate(-2, 4),
        new Coordinate(-1, 3),
        new Coordinate(-1, -3),
        new Coordinate(0, -4)
      };
    missle.shape = new PolyWrapper(SingleGeometryFactory.getFactory().createPolygon(missleCoords));

    // Create asteroid sprites.

    for (int i = 0; i < MAX_ROCKS; i++)
      asteroids[i] = new AsteroidsSprite();

    // Create explosion sprites.

    for (int i = 0; i < MAX_SCRAP; i++)
      explosions[i] = new AsteroidsSprite();

    // Initialize game data and put us in 'game over' mode.

    currentState = new GameState( ship,
                                  photons,
                                  asteroids,
                                  missle,
                                  ufo);
    
    factory = new BRVFactory(currentState);

    highScore = 0;
    detail = true;
    initGame();
    endGame();
  }
  
  // Reset the game bounds (display and active-play area) to
  // the current applet-window bounds:
  public void resizeScreen() {
    // Save the screen size.
    
    Dimension d = getSize();
    AsteroidsSprite.width = d.width;
    AsteroidsSprite.height = d.height;
    
    // Adjust the number of rocks based on 
    // the screen size but never exceed the arrays allocated by MAX_ROCKS:
    maxSpawnableRocks = Math.min(
                          (int) Math.pow(d.height * d.width / 10000, .56), 
                          MAX_ROCKS);

    // Generate the starry background.

    numStars = AsteroidsSprite.width * AsteroidsSprite.height / 5000;
    stars = new Point[numStars];
    for (int i = 0; i < numStars; i++) {
      stars[i] = SingleGeometryFactory.getFactory().createPoint(
          new Coordinate(
              Math.random() * AsteroidsSprite.width, 
              (Math.random() * AsteroidsSprite.height))
          );
    }
  }

  // Reset for a new game:
  public void initGame() {
    
    resizeScreen();

    // Initialize game data and sprites.

    score = 0;
    shipsLeft = MAX_SHIPS;
    asteroidsSpeed = MIN_ROCK_SPEED;
    newShipScore = NEW_SHIP_POINTS;
    newUfoScore = NEW_UFO_POINTS;
    initShip();
    initPhotons();
    stopUfo();
    stopMissle();
    initAsteroids();
    initExplosions();
    playing = true;
    paused = false;
    photonTime = System.currentTimeMillis();
    
    lifeData.clear(); // Reset for a new vehicle.
  }

  // Cleanup after a game:
  public void endGame() {

    // Record statistics for the current game:
    writeLifeData();
    
    // Stop ship, flying saucer, guided missile and associated sounds.

    playing = false;
    stopShip();
    stopUfo();
    stopMissle();
  }

  public void start() {
	  loopThread = new Thread(this);
	  loopThread.start();
  }
  
  public void stop() {
    loopThread = null;
  }

  // Main execution loop:
  public void run() {
    long startTime;
    Thread thisThread = Thread.currentThread();

    // Lower this thread's priority and get the current time.

    startTime = System.currentTimeMillis();

    // This is the main loop.
    while (thisThread == loopThread) {
  		try {
  		    startTime += DELAY;
  		    Thread.sleep(Math.max(0, startTime - System.currentTimeMillis()));
  		  }
  		catch (InterruptedException e) {
  			break;
  		}
  
      debugInfo.clear();
      
      // Update the game:
      if (!paused) {
  
        // Move and process all sprites.
        updateShip();
        updatePhotons();
        updateUfo();
        updateMissle();
        updateAsteroids();
        updateExplosions();
  
        // Check the score and advance high score, add a new ship or start the
        // flying saucer as necessary.
  
        if (score > highScore) {
          highScore = score;
        }
        
        if (score > newShipScore) {
          newShipScore += NEW_SHIP_POINTS;
          shipsLeft++;
        }
  
        if (playing && score > newUfoScore && !ufo.active) {
          newUfoScore += NEW_UFO_POINTS;
          ufoPassesLeft = UFO_PASSES;
          initUfo();
        }
  
        // If all asteroids have been destroyed create a new batch.
  
        if (asteroidsLeft <= 0) {
            if (--asteroidsCounter <= 0) {
              initAsteroids();
            }
        }
  
        // Only Update the CurrentState && Pilot while the game is playing:
        if (playing && ship.active) {
          
          updateControlSignals(pilot);
          pilot.update();
          
          // TODO: CONFIGURE THE FOLLOWING FOR THE CURRENT TEST:
          // Record data every nth frame:
          if (pilot.lifetime % STATISTICS_POLLRATE == 0) {
            logStatistic(Stat.FRAME);
            logStatistic(Stat.SENSOR0SIGNAL);
          }
        }
  
        // Update the screen and set the timer for the next loop.
        repaint();
  
      }
    }
  }
  
  // Record some statistics:
  // Only for testing (inefficient).
  public void logStatistic(Stat stat) {
    
    switch(stat) {
    case FRAME:
      lifeData.add("F:" + String.valueOf(pilot.lifetime) + ",");
      break;
      
    case SENSOR0SIGNAL:
      lifeData.add("S0S:" + String.valueOf(pilot.hardpoints.get(0).getOutput()) + ",");
      break;
    
    
    }
    
  }
  
  // Write debug information to the debug buffer:
  public void debug() {
    
    // Record Ship lifetime
    debugInfo.add(String.format("Lifetime: %,10d", pilot.getLifetime()));
    
    // Record Ship Control Signals:
    debugInfo.add(String.format(
        "Ship L: % -4.2f R: % -4.2f F: % -4.2f B: % -4.2f Fire: %b Hyper: %b",
        pilot.getTurnLeft(), 
        pilot.getTurnRight(), 
        pilot.getAccForward(), 
        pilot.getAccBackward(), 
        pilot.getFirePhoton() >= 1,
        pilot.getFireHyper()  >= 1) );
    
    // Record ship location:
    if (true) {
      Point shipLoc = SingleGeometryFactory.getFactory().createPoint(new Coordinate(ship.x, ship.y));
      double shipTheta = ship.angle;
      Point shipHardLoc = pilot.hardpoints.get(0).getWorldLocation(shipLoc, shipTheta);
      

      debugInfo.add(String.format(
          "Ship R  % 4.0f X % 4.0f Y % 4.0f", 
          shipTheta, 
          ship.x, 
          ship.y));
      
      debugInfo.add(String.format(
          "Hardpoint R % 4.0f X % 4.0f Y % 4.0f",
          shipTheta,
          shipHardLoc.getX(), 
          shipHardLoc.getY()) );
    }
    
    // Record asteroid locations:
    for (int h = 0; h != MAX_ROCKS; h++) {
      if (asteroids[h].active) {
        // Display the corresponding collider.
        
        double aTheta = asteroids[h].angle;

        debugInfo.add(String.format(
            "Asteroid % -5d R % 4.1f X % 4.0f Y % 4.0f", 
            h,
            aTheta, 
            asteroids[h].x, 
            asteroids[h].y));
        }
    }

  }

  // Reset the ship for a new life:
  public void initShip() {

    // Reset the ship sprite at the center of the screen.

    ship.active = true;
    ship.angle = 0.0;
    ship.deltaAngle = 0.0;
    ship.x = 0.0;
    ship.y = 0.0;
    ship.deltaX = 0.0;
    ship.deltaY = 0.0;
    ship.render();

    // Initialize thruster sprites.

    fwdThruster.x = ship.x;
    fwdThruster.y = ship.y;
    fwdThruster.angle = ship.angle;
    fwdThruster.render();
    revThruster.x = ship.x;
    revThruster.y = ship.y;
    revThruster.angle = ship.angle;
    revThruster.render();

    hyperCounter = 0;
  }

  // Interpret control-variables into <ship>-actions:
  public void updateShip() {

    double dx, dy, speed;

    if (!playing) {
      return;
    }

    // Rotate the ship if a turn signal is active:
    ship.angle += turnLeft * SHIP_ANGLE_STEP;

    if (ship.angle > 2 * Math.PI) {
      ship.angle -= 2 * Math.PI;
    }

    ship.angle -= turnRight * SHIP_ANGLE_STEP;
    
    if (ship.angle < 0) {
      ship.angle += 2 * Math.PI;
    }


    // Fire thrusters if an accelerate signal is active:
    dx = SHIP_SPEED_STEP * -Math.sin(ship.angle);
    dy = SHIP_SPEED_STEP *  Math.cos(ship.angle);

    ship.deltaX *= FRICTION;
    ship.deltaY *= FRICTION;

    ship.deltaX += accForward * dx;
    ship.deltaY += accForward * dy;

    ship.deltaX -= accBackward * dx;
    ship.deltaY -= accBackward * dy;


    // Don't let ship go past the speed limit.
    speed = Math.sqrt(ship.deltaX * ship.deltaX + ship.deltaY * ship.deltaY);
    if (speed > MAX_SHIP_SPEED) {
      dx = MAX_SHIP_SPEED * -Math.sin(ship.angle);
      dy = MAX_SHIP_SPEED *  Math.cos(ship.angle);
      ship.deltaX = dx;
      ship.deltaY = dy;
    }


    // Fire photon if a fire signal is active:
    if (firePhoton && ship.active) {
      photonTime = System.currentTimeMillis();
      photonIndex++;

      if (photonIndex >= MAX_SHOTS) {
        photonIndex = 0;
      }

      photons[photonIndex].active = true;
      photons[photonIndex].x = ship.x;
      photons[photonIndex].y = ship.y;
      photons[photonIndex].deltaX = 2 * MAX_ROCK_SPEED * -Math.sin(ship.angle);
      photons[photonIndex].deltaY = 2 * MAX_ROCK_SPEED *  Math.cos(ship.angle);
    }


    // Warp ship into hyperspace by moving to a random location and
    // starting counter if the hyper signal is active:
    if (fireHyper && ship.active && hyperCounter <= 0) {
      ship.x = Math.random() * AsteroidsSprite.width;
      ship.y = Math.random() * AsteroidsSprite.height;
      hyperCounter = HYPER_COUNT;
    }

    // Move the ship. If it is currently in hyperspace, advance the countdown.

    if (ship.active) {
      ship.advance();
      ship.render();
      if (hyperCounter > 0)
        hyperCounter--;

      // Update the thruster sprites to match the ship sprite.

      fwdThruster.x = ship.x;
      fwdThruster.y = ship.y;
      fwdThruster.angle = ship.angle;
      fwdThruster.render();
      revThruster.x = ship.x;
      revThruster.y = ship.y;
      revThruster.angle = ship.angle;
      revThruster.render();
    }

    // Ship is exploding, advance the countdown or create a new ship if it is
    // done exploding. The new ship is added as though it were in hyperspace.
    // (This gives the player time to move the ship if it is in imminent
    // danger.) If that was the last ship, end the game.

    else
      if (--shipCounter <= 0)
        if (shipsLeft > 0) {
          initShip();
          hyperCounter = HYPER_COUNT;
        }
        else
          endGame();
  }

  public void stopShip() {

    // Record data for an ended life:
    if (playing == true && pilot.getLifetime() > 0) {
      String lifeRecord = String.format(
        "class:%s,lifetime:%d,score:%d,maxRocks:%d\n", 
        pilot.getClass().getSimpleName(), 
        pilot.expire(),
        this.score,
        maxSpawnableRocks);

      lifeData.add(lifeRecord);
    }
    
    ship.active = false;
    shipCounter = SCRAP_COUNT;
    if (shipsLeft > 0)
      shipsLeft--;
  }

  public void initPhotons() {

    int i;

    for (i = 0; i < MAX_SHOTS; i++)
      photons[i].active = false;
    photonIndex = 0;
  }

  public void updatePhotons() {

    int i;

    // Move any active photons. Stop it when its counter has expired.

    for (i = 0; i < MAX_SHOTS; i++)
      if (photons[i].active) {
        if (!photons[i].advance())
          photons[i].render();
        else
          photons[i].active = false;
      }
  }

  public void initUfo() {

    double angle, speed;

    // Randomly set flying saucer at left or right edge of the screen.

    ufo.active = true;
    ufo.x = -AsteroidsSprite.width / 2;
    ufo.y = Math.random() * 2 * AsteroidsSprite.height - AsteroidsSprite.height;
    angle = Math.random() * Math.PI / 4 - Math.PI / 2;
    speed = MAX_ROCK_SPEED / 2 + Math.random() * (MAX_ROCK_SPEED / 2);
    ufo.deltaX = speed * -Math.sin(angle);
    ufo.deltaY = speed *  Math.cos(angle);
    if (Math.random() < 0.5) {
      ufo.x = AsteroidsSprite.width / 2;
      ufo.deltaX = -ufo.deltaX;
    }
    if (ufo.y > 0)
      ufo.deltaY = ufo.deltaY;
    ufo.render();
    ufoCounter = (int) Math.abs(AsteroidsSprite.width / ufo.deltaX);
  }

  public void updateUfo() {

    int i, d;

    // Move the flying saucer and check for collision with a photon. Stop it
    // when its counter has expired.

    if (ufo.active) {
      if (--ufoCounter <= 0) {
        if (--ufoPassesLeft > 0)
          initUfo();
        else
          stopUfo();
      }
      if (ufo.active) {
        ufo.advance();
        ufo.render();
        for (i = 0; i < MAX_SHOTS; i++)
          if (photons[i].active && ufo.isColliding(photons[i])) {
            explode(ufo);
            stopUfo();
            score += UFO_POINTS;
          }

          // On occasion, fire a missile at the ship if the saucer is not too
          // close to it.

          d = (int) Math.max(Math.abs(ufo.x - ship.x), Math.abs(ufo.y - ship.y));
          if (ship.active && hyperCounter <= 0 &&
              ufo.active && !missle.active &&
              d > MAX_ROCK_SPEED * FPS / 2 &&
              Math.random() < MISSLE_PROBABILITY)
            initMissle();
       }
    }
  }

  public void stopUfo() {

    ufo.active = false;
    ufoCounter = 0;
    ufoPassesLeft = 0;
  }

  public void initMissle() {

    missle.active = true;
    missle.angle = 0.0;
    missle.deltaAngle = 0.0;
    missle.x = ufo.x;
    missle.y = ufo.y;
    missle.deltaX = 0.0;
    missle.deltaY = 0.0;
    missle.render();
    missleCounter = MISSLE_COUNT;
  }

  public void updateMissle() {

    int i;

    // Move the guided missle and check for collision with ship or photon. Stop
    // it when its counter has expired.

    if (missle.active) {
      if (--missleCounter <= 0)
        stopMissle();
      else {
        guideMissle();
        missle.advance();
        missle.render();
        for (i = 0; i < MAX_SHOTS; i++)
          if (photons[i].active && missle.isColliding(photons[i])) {
            explode(missle);
            stopMissle();
            score += MISSLE_POINTS;
          }
        if (missle.active && ship.active &&
            hyperCounter <= 0 && ship.isColliding(missle)) {
          explode(ship);
          stopShip();
          stopUfo();
          stopMissle();
        }
      }
    }
  }

  public void guideMissle() {

    double dx, dy, angle;

    if (!ship.active || hyperCounter > 0)
      return;

    // Find the angle needed to hit the ship.

    dx = ship.x - missle.x;
    dy = ship.y - missle.y;
    if (dx == 0 && dy == 0)
      angle = 0;
    if (dx == 0) {
      if (dy < 0)
        angle = -Math.PI / 2;
      else
        angle = Math.PI / 2;
    }
    else {
      angle = Math.atan(Math.abs(dy / dx));
      if (dy > 0)
        angle = -angle;
      if (dx < 0)
        angle = Math.PI - angle;
    }

    // Adjust angle for screen coordinates.

    missle.angle = angle - Math.PI / 2;

    // Change the missle's angle so that it points toward the ship.

    missle.deltaX = 0.75 * MAX_ROCK_SPEED * -Math.sin(missle.angle);
    missle.deltaY = 0.75 * MAX_ROCK_SPEED *  Math.cos(missle.angle);
  }

  public void stopMissle() {

    missle.active = false;
    missleCounter = 0;
  }

  public void initAsteroids() {

    int i, j;
    int s;
    double theta, r;
    int x, y;

    // Create random shapes, positions and movements for each asteroid.

    for (i = 0; i < maxSpawnableRocks; i++) {

      // Create a jagged shape for the asteroid and give it a random rotation.

      
      s = MIN_ROCK_SIDES + (int) (Math.random() * (MAX_ROCK_SIDES - MIN_ROCK_SIDES));
      Coordinate[] rockCoords = new Coordinate[s+1];
      for (j = 0; j < s; j ++) {
        theta = 2 * Math.PI / s * j;
        r = MIN_ROCK_SIZE + (int) (Math.random() * (MAX_ROCK_SIZE - MIN_ROCK_SIZE));
        x = (int) -Math.round(r * Math.sin(theta));
        y = (int)  Math.round(r * Math.cos(theta));
        rockCoords[j] = new Coordinate(x, y);
      }
      rockCoords[s] = rockCoords[0];
      asteroids[i].shape = new PolyWrapper(SingleGeometryFactory.getFactory().createPolygon(rockCoords));
      
      asteroids[i].active = true;
      asteroids[i].angle = 0.0;
      asteroids[i].deltaAngle = Math.random() * 2 * MAX_ROCK_SPIN - MAX_ROCK_SPIN;

      // Place the asteroid at one edge of the screen.

      if (Math.random() < 0.5) {
        asteroids[i].x = -AsteroidsSprite.width / 2;
        if (Math.random() < 0.5)
          asteroids[i].x = AsteroidsSprite.width / 2;
        asteroids[i].y = Math.random() * AsteroidsSprite.height;
      }
      else {
        asteroids[i].x = Math.random() * AsteroidsSprite.width;
        asteroids[i].y = -AsteroidsSprite.height / 2;
        if (Math.random() < 0.5)
          asteroids[i].y = AsteroidsSprite.height / 2;
      }

      // Set a random motion for the asteroid.

      asteroids[i].deltaX = Math.random() * asteroidsSpeed;
      if (Math.random() < 0.5)
        asteroids[i].deltaX = -asteroids[i].deltaX;
      asteroids[i].deltaY = Math.random() * asteroidsSpeed;
      if (Math.random() < 0.5)
        asteroids[i].deltaY = -asteroids[i].deltaY;

      asteroids[i].render();
      asteroidIsSmall[i] = false;
    }

    asteroidsCounter = STORM_PAUSE;
    asteroidsLeft = maxSpawnableRocks;
    if (asteroidsSpeed < MAX_ROCK_SPEED)
      asteroidsSpeed += 0.5;
  }

  public void initSmallAsteroids(int n) {

    int count;
    int i, j;
    int s;
    double tempX, tempY;
    double theta, r;
    int x, y;

    // Create one or two smaller asteroids from a larger one using inactive
    // asteroids. The new asteroids will be placed in the same position as the
    // old one but will have a new, smaller shape and new, randomly generated
    // movements.

    count = 0;
    i = 0;
    tempX = asteroids[n].x;
    tempY = asteroids[n].y;
    do {
      if (!asteroids[i].active) {        
        s = MIN_ROCK_SIDES + (int) (Math.random() * (MAX_ROCK_SIDES - MIN_ROCK_SIDES));
        Coordinate[] sRockCoords = new Coordinate[s+1];
        for (j = 0; j < s; j ++) {
          theta = 2 * Math.PI / s * j;
          r = (MIN_ROCK_SIZE + (int) (Math.random() * (MAX_ROCK_SIZE - MIN_ROCK_SIZE))) / 2;
          x = (int) -Math.round(r * Math.sin(theta));
          y = (int)  Math.round(r * Math.cos(theta));
          sRockCoords[j] = new Coordinate(x, y);
        }
        sRockCoords[s] = new Coordinate(sRockCoords[0]);
        
        asteroids[i].shape = new PolyWrapper(SingleGeometryFactory.getFactory().createPolygon(sRockCoords));
        
        asteroids[i].active = true;
        asteroids[i].angle = 0.0;
        asteroids[i].deltaAngle = Math.random() * 2 * MAX_ROCK_SPIN - MAX_ROCK_SPIN;
        asteroids[i].x = tempX;
        asteroids[i].y = tempY;
        asteroids[i].deltaX = Math.random() * 2 * asteroidsSpeed - asteroidsSpeed;
        asteroids[i].deltaY = Math.random() * 2 * asteroidsSpeed - asteroidsSpeed;
        asteroids[i].render();
        asteroidIsSmall[i] = true;
        count++;
        asteroidsLeft++;
      }
      i++;
    } while (i < maxSpawnableRocks && count < 2);
  }

  public void updateAsteroids() {

    int i, j;

    // Move any active asteroids and check for collisions.

    for (i = 0; i < MAX_ROCKS; i++)
      if (asteroids[i].active) {
        asteroids[i].advance();
        asteroids[i].render();

        // If hit by photon, kill asteroid and advance score. If asteroid is
        // large, make some smaller ones to replace it.

        for (j = 0; j < MAX_SHOTS; j++)
          if (photons[j].active && asteroids[i].active && asteroids[i].isColliding(photons[j])) {
            asteroidsLeft--;
            asteroids[i].active = false;
            photons[j].active = false;
            explode(asteroids[i]);
            if (!asteroidIsSmall[i]) {
              score += BIG_POINTS;
              initSmallAsteroids(i);
            }
            else
              score += SMALL_POINTS;
          }

        // If the ship is not in hyperspace, see if it is hit.

        if (ship.active && hyperCounter <= 0 &&
            asteroids[i].active && asteroids[i].isColliding(ship)) {
          explode(ship);
          stopShip();
          stopUfo();
          stopMissle();
        }
    }
  }

  public void initExplosions() {

    int i;

    for (i = 0; i < MAX_SCRAP; i++) {
      explosions[i].shape = new PolyWrapper(
          SingleGeometryFactory.getFactory().createPolygon(
            new Coordinate[] {
                new Coordinate(0,0),
                new Coordinate(0,1),
                new Coordinate(1,1),
                new Coordinate(0,0)})
            );
      explosions[i].active = false;
      explosionCounter[i] = 0;
    }
    explosionIndex = 0;
  }

  public void explode(AsteroidsSprite s) {
    // TODO: Explosions interfere with sensors.
    //       Leaving them out seems to be the best option.
    /*
    int c, i, j;
    int cx, cy;

    // Create sprites for explosion animation. The each individual line segment
    // of the given sprite is used to create a new sprite that will move
    // outward  from the sprite's original position with a random rotation.

    s.render();
    c = 2;
    if (detail || s.sprite.getNumPoints() < 6)
      c = 1;
    for (i = 0; i < s.sprite.getNumPoints(); i += c) {
      explosionIndex++;
      if (explosionIndex >= MAX_SCRAP) { explosionIndex = 0; }
      
      explosions[explosionIndex].active = true;
      
      j = i + 1;
      if (j >= s.sprite.getNumPoints()) {
        j -= s.sprite.getNumPoints();
      }
      cx = 
      cx = (int) ((s.shape.xpoints[i] + s.shape.xpoints[j]) / 2);
      cy = (int) ((s.shape.ypoints[i] + s.shape.ypoints[j]) / 2);
      
      Coordinate[] explosionCoords = new Coordinate[2];
      
      explosions[explosionIndex].shape.addPoint(
        s.shape.xpoints[i] - cx,
        s.shape.ypoints[i] - cy);
      explosions[explosionIndex].shape.addPoint(
        s.shape.xpoints[j] - cx,
        s.shape.ypoints[j] - cy);
      
      explosions[explosionIndex].shape = new Polygon();
      
      explosions[explosionIndex].x = s.x + cx;
      explosions[explosionIndex].y = s.y + cy;
      explosions[explosionIndex].angle = s.angle;
      explosions[explosionIndex].deltaAngle = 4 * (Math.random() * 2 * MAX_ROCK_SPIN - MAX_ROCK_SPIN);
      explosions[explosionIndex].deltaX = (Math.random() * 2 * MAX_ROCK_SPEED - MAX_ROCK_SPEED + s.deltaX) / 2;
      explosions[explosionIndex].deltaY = (Math.random() * 2 * MAX_ROCK_SPEED - MAX_ROCK_SPEED + s.deltaY) / 2;
      explosionCounter[explosionIndex] = SCRAP_COUNT;
      
    }*/
  }

  public void updateExplosions() {

    int i;

    // Move any active explosion debris. Stop explosion when its counter has
    // expired.

    for (i = 0; i < MAX_SCRAP; i++)
      if (explosions[i].active) {
        explosions[i].advance();
        explosions[i].render();
        if (--explosionCounter[i] < 0)
          explosions[i].active = false;
      }
  }

  // Read <pilot>'s control-signals to update ship control-variables:
  public void updateControlSignals(BraitenbergVehicle pilot) {

    // Read <pilot>'s brain for control signals:
    turnLeft =    CONTROL_SCALING * pilot.getTurnLeft();
    turnRight =   CONTROL_SCALING * pilot.getTurnRight();
    accForward =  CONTROL_SCALING * pilot.getAccForward();
    accBackward = CONTROL_SCALING * pilot.getAccBackward();
    firePhoton =  (pilot.getFirePhoton() >= 1); // Convert the signal to a bool.
    fireHyper =   (pilot.getFireHyper() >= 1);  // Convert the signal to a bool.
  }

  // Interpret Keyboard commands as utility functions:
  public void keyPressed(KeyEvent e) {
    
    char c = Character.toUpperCase(e.getKeyChar());

    menuInput = menuInput.concat(Character.toString(c));

    // '/' Toggle debug display:
    if (e.getKeyCode() == KeyEvent.VK_SLASH) {
      debugging = !debugging;
    }
    
    // '.' Toggle sensor display:
    if (e.getKeyCode() == KeyEvent.VK_PERIOD) {
      sensorsShown = !sensorsShown;
    }

    // 'P' key: toggle pause mode.
    if (e.getKeyCode() == KeyEvent.VK_P) {
      paused = !paused;
    }

    // 'D' key: toggle graphics detail on or off.
    if (e.getKeyCode() == KeyEvent.VK_D) {
      detail = !detail;
    }
    
    // End the game prematurely:
    if (e.getKeyCode() == KeyEvent.VK_K) {
      endGame();
      menuInput = "";
    }
    
    // 'Enter' key: start the game, if not already in progress.
    if (e.getKeyCode() == KeyEvent.VK_ENTER && !playing) {
      
      if (menuInput.contains("1R")) {
        pilot = factory.makeVehicleOneRadius();
      }
      else if (menuInput.contains("1L")) {
        pilot = factory.makeVehicleOneRay();
      }
      else if (menuInput.contains("2A")) {
        pilot = factory.makeVehicleTwoATriangle();
      }
      else if (menuInput.contains("2B")) {
        pilot = factory.makeVehicleTwoBTriangle();
      }
      else if (menuInput.contains("3A")) {
        pilot = factory.makeVehicleThreeATriangle();
      }
      else if (menuInput.contains("3B")) {
        pilot = factory.makeVehicleThreeBTriangle();
      }
      else if (menuInput.contains("4A")) {
        pilot = factory.makeVehicleFourATriangle();
      }
      else if (menuInput.contains("1F")) {
        pilot = factory.makeVehicleRayEye();
      } 
      else {
        menuInput = "";
      }
      
      // If menuInput was a valid code:
      if (menuInput != "") {
        initGame();
      }
      menuInput = "";
    }
  }

  public void keyReleased(KeyEvent e) {}
  public void keyTyped(KeyEvent e) {}
  public void update(Graphics g) { paint(g); }

  // Write the data in <lifeData> to the console and clear <lifeStatistics>:
  private void writeLifeData() {
    
    ArrayList<String> copy = new ArrayList<String>(lifeData);
    // Write the current vehicles life data on one line:
    for (String str : copy) {
      System.out.print(str);
    }
    
    System.out.print("\n\n"); // Start a new line for the next vehicle.
  }

  public void paint(Graphics g) {

    Dimension d = getSize();
    int i;
    int c;
    String s;

    // Create the off screen graphics context, if no good one exists.

    if (offGraphics == null || d.width != offDimension.width ||
       d.height != offDimension.height) {
      offDimension = d;
      offImage = createImage(d.width, d.height);
      offGraphics = offImage.getGraphics();
    }

    // Fill in background and stars.

    offGraphics.setColor(Color.black);
    offGraphics.fillRect(0, 0, d.width, d.height);
    if (detail) {
      offGraphics.setColor(Color.white);
      for (i = 0; i < numStars; i++) {
        offGraphics.drawLine(
            (int)stars[i].getCoordinate().x, 
            (int)stars[i].getCoordinate().y, 
            (int)stars[i].getCoordinate().x, 
            (int)stars[i].getCoordinate().y);
      }
    }

    // Draw photon bullets.

    offGraphics.setColor(Color.white);
    for (i = 0; i < MAX_SHOTS; i++)
      if (photons[i].active)
        offGraphics.drawPolygon(
            photons[i].sprite.xPoints(), 
            photons[i].sprite.yPoints(), 
            photons[i].sprite.getNumPoints());

    // Draw the guided missle, counter is used to quickly fade color to black
    // when near expiration.

    c = Math.min(missleCounter * 24, 255);
    offGraphics.setColor(new Color(c, c, c));
    if (missle.active) {
      offGraphics.drawPolygon(
          missle.sprite.xPoints(), 
          missle.sprite.yPoints(), 
          missle.sprite.getNumPoints());
      
      offGraphics.drawLine(
        missle.sprite.xPoints()[missle.sprite.getNumPoints() - 1], 
        missle.sprite.yPoints()[missle.sprite.getNumPoints() - 1],
        missle.sprite.xPoints()[0], 
        missle.sprite.yPoints()[0]);
    }

    // Draw the asteroids.

    for (i = 0; i < MAX_ROCKS; i++)
      if (asteroids[i].active) {
        if (detail) {
          offGraphics.setColor(Color.black);
          offGraphics.fillPolygon(
              asteroids[i].sprite.xPoints(),
              asteroids[i].sprite.yPoints(),
              asteroids[i].sprite.getNumPoints());
        }
        
        offGraphics.setColor(Color.white);
        offGraphics.drawPolygon(
            asteroids[i].sprite.xPoints(),
            asteroids[i].sprite.yPoints(),
            asteroids[i].sprite.getNumPoints());
        
        offGraphics.drawLine(
          asteroids[i].sprite.xPoints()[asteroids[i].sprite.getNumPoints() - 1],
          asteroids[i].sprite.yPoints()[asteroids[i].sprite.getNumPoints() - 1],
          asteroids[i].sprite.xPoints()[0], 
          asteroids[i].sprite.yPoints()[0]);
      }

    // Draw the flying saucer.

    if (ufo.active) {
      if (detail) {
        offGraphics.setColor(Color.black);
        offGraphics.fillPolygon(
            ufo.sprite.xPoints(),
            ufo.sprite.yPoints(),
            ufo.sprite.getNumPoints());
      }
      
      offGraphics.setColor(Color.white);
      offGraphics.drawPolygon(
          ufo.sprite.xPoints(),
          ufo.sprite.yPoints(),
          ufo.sprite.getNumPoints());
      
      offGraphics.drawLine( ufo.sprite.xPoints()[ufo.sprite.getNumPoints() - 1], 
                            ufo.sprite.yPoints()[ufo.sprite.getNumPoints() - 1],
                            ufo.sprite.xPoints()[0],
                            ufo.sprite.yPoints()[0]);
    }

    // Draw the ship, counter is used to fade color to white on hyperspace.

    c = 255 - (255 / HYPER_COUNT) * hyperCounter;
    if (ship.active) {
      if (detail && hyperCounter == 0) {
        offGraphics.setColor(Color.black);
        offGraphics.fillPolygon(
            ship.sprite.xPoints(),
            ship.sprite.yPoints(),
            ship.sprite.getNumPoints());
      }
      offGraphics.setColor(new Color(c, c, c));
      offGraphics.drawPolygon(
          ship.sprite.xPoints(),
          ship.sprite.yPoints(),
          ship.sprite.getNumPoints());
      
      offGraphics.drawLine( ship.sprite.xPoints()[ship.sprite.getNumPoints() - 1], 
                            ship.sprite.yPoints()[ship.sprite.getNumPoints() - 1],
                            ship.sprite.xPoints()[0], 
                            ship.sprite.yPoints()[0]);

      // Draw the ship's sensors:
      if (sensorsShown) {
        /* Fill the sensor:
        offGraphics.setColor(Color.red);
        offGraphics.fillPolygon(
            pilot.hardpoints.get(0).sensor.getShape().xPoints(),
            pilot.hardpoints.get(0).sensor.getShape().yPoints(),
            pilot.hardpoints.get(0).sensor.getShape().getNumPoints());
         */
        
        offGraphics.setColor(Color.red);
        offGraphics.drawPolygon(
            pilot.hardpoints.get(0).sensor.getShape().xPoints(),
            pilot.hardpoints.get(0).sensor.getShape().yPoints(),
            pilot.hardpoints.get(0).sensor.getShape().getNumPoints());
      }
      
      
      
      // Draw thruster exhaust if thrusters are on. Do it randomly to get a
      // flicker effect.

      if (!paused && detail && Math.random() < 0.5) {
        if (accForward > 0) {
          offGraphics.drawPolygon(
              fwdThruster.sprite.xPoints(),
              fwdThruster.sprite.yPoints(),
              fwdThruster.sprite.getNumPoints());
          
          offGraphics.drawLine( fwdThruster.sprite.xPoints()[fwdThruster.sprite.getNumPoints() - 1], 
                                fwdThruster.sprite.yPoints()[fwdThruster.sprite.getNumPoints() - 1],
                                fwdThruster.sprite.xPoints()[0], 
                                fwdThruster.sprite.yPoints()[0]);
        }
        if (accBackward > 0) {
          offGraphics.drawPolygon(
              revThruster.sprite.xPoints(),
              revThruster.sprite.xPoints(),
              revThruster.sprite.getNumPoints());
          
          offGraphics.drawLine( revThruster.sprite.xPoints()[revThruster.sprite.getNumPoints() - 1], 
                                revThruster.sprite.yPoints()[revThruster.sprite.getNumPoints() - 1],
                                revThruster.sprite.xPoints()[0],
                                revThruster.sprite.yPoints()[0]);
        }
      }
    }

    // Draw any explosion debris, counters are used to fade color to black.

    for (i = 0; i < MAX_SCRAP; i++)
      if (explosions[i].active) {
        c = (255 / SCRAP_COUNT) * explosionCounter [i];
        offGraphics.setColor(new Color(c, c, c));
        offGraphics.drawPolygon(
            explosions[i].sprite.xPoints(),
            explosions[i].sprite.xPoints(),
            explosions[i].sprite.getNumPoints());
      }

    // Display status and messages.

    offGraphics.setFont(font);
    offGraphics.setColor(Color.white);

    offGraphics.drawString("Score: " + score, fontWidth, fontHeight);
    offGraphics.drawString("Ships: " + shipsLeft, fontWidth, d.height - fontHeight);
    s = "High: " + highScore;
    offGraphics.drawString(s, d.width - (fontWidth + fm.stringWidth(s)), fontHeight);

    // Display debug info:
    if (debugging) {
      debug(); // Collect debug info

      int j = 0; // Index of the current item in the iteration.
      ArrayList<String> debugCopy = new ArrayList<String>(debugInfo);
      for(String cur : debugCopy) {
        s = cur;
        offGraphics.drawString(
          s, 
          d.width - (fontWidth + fm.stringWidth(s)),
          fontHeight*(j+2));

        j++;
      }
    }
    
    // Display Menu:
    if (!playing) {
      
      // Display menu input:
      s = menuInput;
      offGraphics.drawString(
        s, 
        d.width / 4,
        d.height / 8 + fontHeight*(menuInfo.size()-7));
      
      // Display menu info:
      int k = 0; // Index of the current line being displayed.
      ArrayList<String> menuCopy = new ArrayList<String>(menuInfo);
      for(String cur : menuCopy) {
        s = cur;
        offGraphics.drawString(
          s, 
          d.width / 4,
          d.height / 8 + fontHeight*(k));

        k++;
      }
    }
    else if (paused) {
      s = "Game Paused";
      offGraphics.drawString(s, (d.width - fm.stringWidth(s)) / 2, d.height / 4);
    }

    // Copy the off screen buffer to the screen.

    g.drawImage(offImage, 0, 0, this);
  }
}
