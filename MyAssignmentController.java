// File: MyAssignmentController.java
// Description: Controller file for Pioneer3-DX 
// ==============================================================================================
// The aim of the assignment is to move the robot around the arena in such a way
// as to generate an occupancy grid map of the arena itself. 
// Creation of motion state machine with FORWARD or ROTATE options which use a given array
// of waypoints to calculate distance and rotation change between current pose and next waypoint, 
// wheel rotation is then used to realise this change in pose.
// ==============================================================================================


import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;

public class MyAssignmentController {

  // Dimensions of the Robot
  // ---------------------------------------------------------------------------
  private final static double ROBOT_RADIUS = 0.215;  // in meters
  public static double wheelRadius = 0.0975;         // in meters
  public static double axelLength = 0.31;            // Distance (in m) between the two wheels
  public static int MAX_NUM_SENSORS = 16;            // Number of sensors on the robot


  // Assignment Parameters
  // ---------------------------------------------------------------------------
  private final static int NUMBER_OF_ROWCELLS = 100;   // How many cells across the occupancy grid
  private final static int NUMBER_OF_COLCELLS = 100;   // How many cells down the occupancy grid
  
  // This is the frequency that the map is updated 
  private final static int GRID_UPDATE_FREQUENCY = 2;  // How frequently do we sample the world 
  
  // This boolean switches on (or off) the generation of the occupancy grid
  private final static boolean GENERATE_OCCUPANCY_GRID = true;


  // Robot instance
  // ---------------------------------------------------------------------------
  public static Supervisor robot;
  public static Node robotNode;


  // Static Methods  
  // ==================================================================================
  
  //   returns the real position of the robot without the need for localisation through particle filters etc
  public static Pose getLocalisedPos() {
    double[] realPos = robotNode.getPosition();
    double[] rot = robotNode.getOrientation(); // 3x3 Rotation matrix as vector of length 9
    double theta1 = Math.atan2(rot[2], rot[8]);
    double halfPi = Math.PI/2;
    double theta2 = theta1 + halfPi;
    if (theta1 > halfPi)
        theta2 = -(3*halfPi)+theta1;
    
    return new Pose(realPos[0], -realPos[2], theta2);
  }
  
  // new enum to model state machine for moving forward or rotating
  public enum MoveState { FORWARD, ROTATE };
  
 
  // Main Methods 
  // ==================================================================================  
  public static void main(String[] args) {

    // Define Robot Parameters
    long loopCounter = 0; // Used to count the number of main loop iterations
                                  
    // create the Supervised Robot instance.
    // ---------------------------------------------------------------------------
    robot = new Supervisor();
    robotNode = robot.getSelf(); // Get a handle to the Robot Node in supervisor mode
    
    // get the time step of the current world.
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // time elapsed variable initialised to zero
    int timeElapsed = 0;    
    
    // Set up motor devices
    // ---------------------------------------------------------------------------
    Motor leftMotor = robot.getMotor("left wheel");
    Motor rightMotor = robot.getMotor("right wheel");
    
    // Set the target positions of the motors
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);

    // Initialise motor velocity
    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);
    
    // angular velocity initialised to 2 pi rads per second
    double av = 2 * Math.PI;
    
    // start moving forward
    leftMotor.setVelocity(av);
    rightMotor.setVelocity(av);
    
    boolean inMotion = true;  // used to determine if Pioneer3-DX is in motion or stationary
    double targetDistance;    // used to determine distance between current pose and next waypoint
    double targetRotation;    // used to determine difference in angle between current pose and desired heading
    double linearVelocity = 0.3;   // linear velocity in m/s initialised to 0.3
    double targetTimeDistance = 0;   // time taken to travel targetDistance initialised to zero
    double targetTimeRotation = 0;   // time taken to rotate tagetRotation initialsed to zero
    double distanceToWaypoint;       // used when calculating cartesian distance between current pose and next waypoint 
    double differenceInAngle = 0;    // used when calculating difference in angle between current pose and next heading
    
    // set initial moveState to rotation
    MoveState state = MoveState.ROTATE;

    // set up proximity detectors
    // ---------------------------------------------------------------------------
    DistanceSensor[] ps = new DistanceSensor[MAX_NUM_SENSORS];
    String[] psNames = {
      "so0", "so1", "so2", "so3", "so4", "so5", "so6", "so7",
      "so8", "so9", "so10", "so11", "so12", "so13", "so14", "so15",     
    };
    
    // The following array determines the orientation of each sensor, based on the details of the Pioneer Robot Stat sheet
    double[] psAngleDeg = { 90, 50, 30, 10, -10, -30, -50, -90,
                            -90, -130, -150, -170, 170, 150, 130, 90};

    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
      ps[i] = robot.getDistanceSensor(psNames[i]);
      ps[i].enable(timeStep);
    }
    
    double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0};

    // Set up occupancy grid
    // ---------------------------------------------------------------------------
    OccupancyGrid grid;                              // Instantiate to generate an occupancy grid
    if (GENERATE_OCCUPANCY_GRID == true) {
      grid = new OccupancyGrid(5.0, 5.0,             // Size of the arena
                                NUMBER_OF_ROWCELLS,  // Number of cells along the x-axis
                                NUMBER_OF_COLCELLS,  // Number of cells along the y-axis
                                ps,                  // Array of distance sensors
                                psAngleDeg,          // Orientation of each sensor (array)
                                ROBOT_RADIUS);       // Radius of the robot body (assumes cylindrical)
    } else {
      grid=null;                                     // No occupancy grid will be generated
    }
  
  
    // Set up display devices
    // ---------------------------------------------------------------------------
    
    // The sensor view from the Labs is included to assist in debugging
    Display sensorDisplay = robot.getDisplay("sensorDisplay");
    SensorView sensorView = new SensorView(sensorDisplay, ps, psAngleDeg, ROBOT_RADIUS);

    // A variant of the Arena view is used to show the robot position in a map
    // The current display is configured as a 500x500 display attached to the robot   
    Display occupancyGridDisplay = robot.getDisplay("occupancyGridDisplay");
    ArenaView gridView = new ArenaView(occupancyGridDisplay, getLocalisedPos(), grid, 5.0, 5.0, ROBOT_RADIUS);
    
    // waypoints array consisting of a number of given poses
    Pose[] wayPoints = new Pose[50];
    
    // populate wayPoints array with multiple poses 
    wayPoints[0] = new Pose(0.55, 2.128, 4.71);
    wayPoints[1] = new Pose(0.55, -0.461, 3.14);
    wayPoints[2] = new Pose(-0.309, -0.461, 1.57);
    wayPoints[3] = new Pose(-0.309, 1.23, 3.14);
    wayPoints[4] = new Pose(-2.11, 1.20, 4.71);
    wayPoints[5] = new Pose(-2.11, 0.49, 0);
    wayPoints[6] = new Pose(-1.17, 0.49, 4.71);
    wayPoints[7] = new Pose(-1.17, -0.67, 3.14);
    wayPoints[8] = new Pose(-2.08, -0.67, 4.71);
    wayPoints[9] = new Pose(-2.08, -1.34, 5.9);
    wayPoints[10] = new Pose(0.019, -2.12, 0.38);
    wayPoints[11] = new Pose(2.09, -1.26, 1.57);
    wayPoints[12] = new Pose(2.09, -0.696, 3.14);
    wayPoints[13] = new Pose(0.99, -0.696, 1.57);
    wayPoints[14] = new Pose(0.99, 1.17, 0);
    wayPoints[15] = new Pose(2.12, 1.17, 4.71);
    wayPoints[16] = new Pose(2.12, -0.696, 3.14);
    wayPoints[17] = new Pose(0.79, -0.696, 1.57);
    wayPoints[18] = new Pose(0.79, 2.21, 3.14);
    wayPoints[19] = new Pose(-2.16, 2.21, 4.71);
    wayPoints[20] = new Pose(-2.16, 1.26, 0);
    wayPoints[21] = new Pose(-0.306, 1.26, 4.71);
    wayPoints[22] = new Pose(-0.304, -0.74, 0);
    wayPoints[23] = new Pose(2.1, -0.74, 4.71);
    wayPoints[24] = new Pose(2.1, -1.260, 3.5);
    wayPoints[25] = new Pose(-0.4, -2.1, 2.8);
    wayPoints[26] = new Pose(-2.13, -1.37, 1.57);
    wayPoints[27] = new Pose(-2.13, -0.74, 0);
    wayPoints[28] = new Pose(-1.2, -0.74, 1.57);
    wayPoints[29] = new Pose(-1.2, 0.39, 3.14);
    wayPoints[30] = new Pose(-2.13, 0.39, 1.57);
    wayPoints[31] = new Pose(-2.15, 2.1, 0);
    wayPoints[32] = new Pose(1.13, 2.1, 4.3);
    wayPoints[33] = new Pose(0.55, 0.338, 5.2);
    wayPoints[34] = new Pose(1.19, -0.70, 0);
    wayPoints[35] = new Pose(2.05, -0.789, 1.57);
    wayPoints[36] = new Pose(2.05, 1.3, 3.14);
    wayPoints[37] = new Pose(0.76, 1.27, 1.57);
    wayPoints[38] = new Pose(0.76, 2.06, 3.14);
    wayPoints[39] = new Pose(-2.08, 2.09, 4.71);
    wayPoints[40] = new Pose(-2.08, 1.23, 0);
    wayPoints[41] = new Pose(-0.297, 1.23, 4.71);
    wayPoints[42] = new Pose(-0.297, -0.297, 0);
    wayPoints[43] = new Pose(0.561, -0.297, 1.57);
    wayPoints[44] = new Pose(0.561, 2.1, 0);
    wayPoints[45] = new Pose(2.1, 2.1, 0);
    wayPoints[46] = new Pose(2.1, 2.1, 0);
    
    int i = 0;   // used as a counter to move to next waypoint

    // Main loop
    // ---------------------------------------------------------------------------
    
    // perform simulation steps until Webots is stopping the controller
    while (robot.step(timeStep) != -1) {
      
      // Get current pose of the robot        
      // ---------------------------------------------------------------------------
      
      Pose p = getLocalisedPos();
            
      // Update the grid map and arena display
      // ---------------------------------------------------------------------------
      
      if (loopCounter++ % GRID_UPDATE_FREQUENCY == 0) {
        if (GENERATE_OCCUPANCY_GRID == true) {
          grid.occupancy_grid_mapping(p);
        }
        gridView.setPose(p);
        gridView.paintView();
      }

      // Update the sensor display
      // ---------------------------------------------------------------------------
      
      sensorView.setPose(p);
      sensorView.paintView();
      
      // Move robot
      // ---------------------------------------------------------------------------
      
      // switch statement to select FORWARD or ROTATE block of code
      switch (state) {
        case FORWARD:
             
          if (inMotion == true) {
            // Pioneer3-DX is in motion
            if (timeElapsed > targetTimeDistance) {
              // Need to stop Pioneer3-DX moving
              leftMotor.setVelocity(0.0);
              rightMotor.setVelocity(0.0);
              inMotion = false;
              // Calculate the next difference in angle to rotate by before initiating ROTATE case
              differenceInAngle = wayPoints[i].getTheta() - p.getTheta(); 
              // Normalise difference in current pose heading and desired heading (ideally call the getDeltaTheta() method) 
              if (differenceInAngle > Math.PI) {
                differenceInAngle = -(2*Math.PI) + differenceInAngle;
              } else if (differenceInAngle < -Math.PI) {
                  differenceInAngle = (2*Math.PI) + differenceInAngle;
                }
              // if angle is less than zero or not calculate time to rotate
              if (differenceInAngle < 0) {
                targetRotation = -1 * Math.PI * axelLength * (differenceInAngle / (2*Math.PI));
                targetTimeRotation = 1000.0 * (targetRotation/linearVelocity);
              } else {
                  targetRotation = Math.PI * axelLength * (differenceInAngle / (2*Math.PI));
                  targetTimeRotation = 1000.0 * (targetRotation/linearVelocity);
                }
              // increment i to set next waypoint from wayPoint array and set state to ROTATE
              i++;
              state = MoveState.ROTATE;
            } else {
                timeElapsed += timeStep;   // increment time
              }
           } else {
               // Pioneer3-DX is not in motion, first check there is a wayPoint to travel to then start moving forward
               if (i < 46) {
                 av = linearVelocity/wheelRadius;
                 leftMotor.setVelocity(av);
                 rightMotor.setVelocity(av);
                 // reset time elapsed and set inMotion status to true
                 timeElapsed = 0;
                 inMotion = true;
                } else {
                   // if no further waypoints stop moving
                   leftMotor.setVelocity(0);
                   rightMotor.setVelocity(0);
                 }
            }
            break;
          
          case ROTATE:
          
            if (inMotion == true) {
              // Pioneer3-DX is in motion
              if (timeElapsed > targetTimeRotation) {
                // Need to stop Pioneer3-DX moving
                leftMotor.setVelocity(0.0);
                rightMotor.setVelocity(0.0);
                inMotion = false;
                // calculate the difference in distanc eto travel by before initiating the FORWARD case
                distanceToWaypoint = Math.sqrt(Math.pow(p.getX()-wayPoints[i].getX(),2)+Math.pow(p.getY()-wayPoints[i].getY(),2));
                targetDistance = distanceToWaypoint;
                // calculate time to travel 
                targetTimeDistance = 1000.0 * (targetDistance/linearVelocity);
                // set state to FORWARD
                state = MoveState.FORWARD;
              } else {
                  timeElapsed += timeStep;   // increment time
                }
             } else {
                 // Pioneer3-DX is not in motion, rotate using angular velocity
                 av = (linearVelocity/wheelRadius);
                 // first determine if the turn is clockwise or anticlockwise
                 if (differenceInAngle < 0) {
                    leftMotor.setVelocity(av);
                    rightMotor.setVelocity(-av);
                  } else {
                      leftMotor.setVelocity(-av);
                      rightMotor.setVelocity(av);
                    }
                  // reset time elapsed and set inMotion status to true
                  timeElapsed = 0;
                  inMotion = true;
                }
           break;
        }
    };
    // Enter here exit cleanup code.
  }
}
