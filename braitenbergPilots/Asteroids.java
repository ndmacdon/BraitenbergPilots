package braitenbergPilots;
/******************************************************************************
  
  Original Engine Code:
  
  Asteroids, Version 1.3

  Copyright 1998-2001 by Mike Hall.
  Please see http://www.brainjar.com for terms of use.

******************************************************************************/

/****
 *
 * Project:    Flotsam Braitenberg Vehicles
 * 
 * Author:      Nicholas Macdonald
 *
 * Date:        January 16, 2014
 * 
 * File:        Asteroids.java
 * 
 * Usage:       <embed  type="application/x-java-applet;version=1.6"
 *                      width="w" height="h" 
 *                      archive="braitenoids_selection.jar,lib/jts-1.13.jar"
 *                      code="braitenbergPilots.Asteroids.class"
 *                      pluginspage="http://java.com/download/" />
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
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.operation.distance.DistanceOp;

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
  
  static final int STATISTICS_POLLRATE = 1; // Record statistics every n frames.
  
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
  
  // Tourney Information.
  
  boolean tourneyMode = false;
  int gamesPerParticipant = 30;
  int participants = 7;
  int currentGame = 0;
  int currentParticipant = 0;
  
  
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

  static  int FRAME_LENGTH = 500; //1000    // Milliseconds in one frame.
  static final int DELAY = 7;     //15      // Milliseconds between screen and
  static final int FPS   =                  // the resulting frame rate.
    Math.round(FRAME_LENGTH / DELAY);

  static final int MAX_SHOTS =  12;         // Maximum number of sprites
  static final int MAX_ROCKS =  4;         // for photons, asteroids and
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
    menuInfo.add("Enter a two-character key and press 'Enter'");
    menuInfo.add("to see that vehicle play the game:");
    menuInfo.add("");
    menuInfo.add("Braitenberg-Vehicles   |   Fit-Vehicles");
    menuInfo.add("Key   Pilot            |   Key   Pilot");
    menuInfo.add("-----------------------|--------------------");
    menuInfo.add("1R    D.N.O.-Radius    |   1F    Ray Eye");
    menuInfo.add("1L    D.N.O.-Laser     |   2F    Polar Hopper");
    menuInfo.add("2A    Coward           |   3F");
    menuInfo.add("2B    Angry            |   4F");
    menuInfo.add("3A    Admirer          |   5F");
    menuInfo.add("3B    Explorer         |   6F");
    menuInfo.add("3C    Anxious-Explorer |   6F");
    menuInfo.add("4A    Dancer           |   7F");
    menuInfo.add("4B    Orbiter          |   7F");
    menuInfo.add("");
    menuInfo.add("Press 'P' at any time to Pause.");
    menuInfo.add("Press 'K' at any time to End the current game.");
    menuInfo.add("Press 'Enter' to begin or clear current input.");
    menuInfo.add("Press '.' to toggle sensor display.");
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
        
        // Execute the tournament:
        if (tourneyMode && !playing) {
          
          if (currentParticipant < participants) {
            if (currentGame < gamesPerParticipant) {
                pilot = factory.makeVehicleRayEye();
                currentGame++;
                initGame();
            }
            else {
              // Select next participant...
              currentParticipant++;http://stackoverflow.com/questions/11224217/iterating-over-all-methods-that-have-name-starting-with-get-comparing-object
            }
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
            //logStatistic(Stat.SENSOR0SIGNAL);
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
      while (lifeData.size() < 1) { lifeData.add("F:"); }
      
      lifeData.set(0, lifeData.get(0).concat(String.valueOf(pilot.lifetime) + ",")); 
      break;
      
    case SENSOR0SIGNAL:
      while (lifeData.size() < 2) { lifeData.add("S0S:"); }
      
      lifeData.set(1, lifeData.get(1).concat(String.valueOf(pilot.hardpoints.get(0).getOutput()) + ","));
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
        "class:%s,lifetime:%d,score:%d,maxRocks:%d", 
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
      else if (menuInput.contains("1C")) {
        pilot = factory.makeVehicleOneCone();
      }
      else if (menuInput.contains("2A")) {
        pilot = factory.makeVehicleTwoACone();
      }
      else if (menuInput.contains("2B")) {
        pilot = factory.makeVehicleTwoBCone();
      }
      else if (menuInput.contains("3A")) {
        pilot = factory.makeVehicleThreeACone();
      }
      else if (menuInput.contains("3B")) {
        pilot = factory.makeVehicleThreeBRadius();
      }
      else if (menuInput.contains("3C")) {
        pilot = factory.makeVehicleThreeBCone();
      }
      else if (menuInput.contains("4A")) {
        pilot = factory.makeVehicleFourARadius();
      }
      else if (menuInput.contains("4B")) {
        pilot = factory.makeVehicleFourBRadius();
      }
      else if (menuInput.contains("1F")) {
        pilot = factory.makeVehicleRayEye();
      }
      else if (menuInput.contains("2F")) {
        pilot = factory.makeVehiclePolarRegions();
      }
      else if (menuInput.contains("1T")) {
        pilot = factory.makeVehicleSensorTestRadius();
      }
      else if (menuInput.contains("TOURNEY")) {
        tourneyMode = true;
        menuInput = "";
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
      System.out.println(str);
    }
    
    System.out.print("\n"); // Start a new line for the next vehicle.
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
        for (Hardpoint h : pilot.hardpoints) {
          offGraphics.setColor(Color.red);
          offGraphics.drawPolygon(
              h.sensor.getWorldShape().xPoints(),
              h.sensor.getWorldShape().yPoints(),
              h.sensor.getWorldShape().getNumPoints());
        }
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
              revThruster.sprite.yPoints(),
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
            explosions[i].sprite.yPoints(),
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
