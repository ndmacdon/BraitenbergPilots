package braitenbergPilots;
/****
 *
 * Project:    Flotsam Braitenberg Vehicles
 * 
 * Author:      Nicholas Macdonald
 *
 * Date:        January 16, 2014
 * 
 * File:        BRVUtils.java
 *
 * Description: A series of robots implemented inside of Asteroids. 
 *              The robots are modeled after those described in Valentino 
 *              Braitenberg's Vehicles. They are equipped with; sensors which 
 *              are excited by various qualities of the environment, Modulator 
 *              devices which modulate the raw output of sensors, and 'motors' 
 *              which have various effects on the robot and its environment.
 ****/

import java.awt.*;
import java.util.*;
import java.util.List;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.LinearRing;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.util.GeometricShapeFactory;


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


//A singleton used to generate Geometry objects in JTS:
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
  
  // Return the AsteroidsSprites which intersect <p>:
  public List<AsteroidsSprite> intersects(Polygon p) {
    // List of the colliders we inspect:
    List<AsteroidsSprite> activeColliders = this.getActiveColliders();

    // List of colliders intersecting <p>:
    List<AsteroidsSprite> intersected = new ArrayList<AsteroidsSprite>();

    // Check for intersection between <p> and every active-collider:
    for (AsteroidsSprite cur : activeColliders) {
        if (p.intersects(cur.sprite)) {
          intersected.add(cur);
        }
    }

    return intersected;
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
