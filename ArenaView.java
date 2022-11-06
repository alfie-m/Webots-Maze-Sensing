 /* This class displays the robot's internal model of the environment as a set of
 * constant sized cells.
 * =================================================================================
 */

import java.util.ArrayList;
import com.cyberbotics.webots.controller.Display;

public class ArenaView {
  private Display display;    // reference to the display device on the robot
  private OccupancyGrid grid; // Occupancy Grid used to display map (could be NULL for no map)
  private Pose robotPose;     // Pose of the robot
  
  private int deviceWidth;    // width of the display device
  private int deviceHeight;   // height of the display device
  private int numCellsInRow;	 // number of cells across
  private int numCellsInCol;	 // number of cells up
  private int cellWidth;      // width of a single cell in display ccordinates
  private int cellHeight;     // height of a single cell in display ccordinates
  private double radius;      // radius of the robot (assume it is round)

  private double scaleFactor; // Scale factor to scale rendered map to the maximal dimension on the display
	
  // Colours used by the display
  private final static int DARKGREY = 0x3C3C3C;
  private final static int GREY = 0x787878;
  private final static int BLACK = 0x000000;
  private final static int WHITE = 0xFFFFFF;

  // Constructor
  // ==================================================================================
  
  public ArenaView(Display d, Pose p, OccupancyGrid grid, double arenaWidth, double arenaHeight, double radius) {
    this.display = d;
    this.grid = grid;
    this.robotPose = new Pose(p);    // Create a new object with the pose information
    this.radius = radius;    
    this.deviceWidth = this.display.getWidth();
    this.deviceHeight = this.display.getHeight();
    this.numCellsInRow = (grid==null?0:grid.getNumCellsInRow());
    this.numCellsInCol = (grid==null?0:grid.getNumCellsInCol());
    this.cellWidth = (int) (this.deviceWidth / (double) this.numCellsInRow);
    this.cellHeight = (int) (this.deviceHeight / (double) this.numCellsInCol);

   
    // Determine the rendering scale factor
    double wsf = ((double) this.deviceWidth) / arenaWidth;
    double hsf = ((double) this.deviceHeight) / arenaHeight;
    this.scaleFactor = Math.min(wsf, hsf);   
  }

  // Getters / Setters  
  // ================================================================================== 
  
  public void setPose(Pose p) {
    // Sets the pose of the robot
    this.robotPose.setPosition(p);
  }

  // Internal Methods  
  // ================================================================================== 
  
  // Map the real coordinates to screen coordinates assuming the origin is in the center and y axis is inverted
  private int scale(double l) {
    return (int) (this.scaleFactor * l);
  }
  private int mapX(double x) {
    return (int) ((this.deviceWidth/2.0) + scale(x));
  }
  private int mapY(double y) {
    return (int) ((this.deviceHeight/2.0) - scale(y));
  }

  
  // External Methods  
  // ==================================================================================
  
  // If arity=0, then call the arity 1 method with a non 0..1 value
  public void paintView() {
    this.paintView(-1);
  }
  
  // If the value of threshold is in the inclusive range 0..1, then a binary grid will
  // be painted with the threshold used to determine occupancy
  public void paintView(double threshold) { 
  
    // draw a background
    this.display.setColor(0xF0F0F0);     // Off White
    this.display.fillRectangle(0, 0, this.deviceWidth, this.deviceHeight);

    // Handle occupancy grid
    if (grid!=null) {
    
      double cellProb;          // probabilty of occuupancy for each cell
      for (int i=0; i< grid.getGridSize(); i++) {
        cellProb = grid.getCellProbability(i);
        int x = cellWidth * (i%numCellsInRow);
        int y = cellHeight * (i/numCellsInRow);
                
        if ((threshold < 0) || (threshold >1)) {
          // Determine colour
          if (cellProb < 0.1) this.display.setColor(WHITE);
          else if (cellProb < 0.2) this.display.setColor(0xDDDDDD);
          else if (cellProb < 0.3) this.display.setColor(0xBBBBBB);
          else if (cellProb < 0.4) this.display.setColor(0x999999);
          else if (cellProb > 0.9) this.display.setColor(BLACK);
          else if (cellProb > 0.8) this.display.setColor(0x222222);
          else if (cellProb > 0.7) this.display.setColor(0x444444);
          else if (cellProb > 0.6) this.display.setColor(0x666666);
          else this.display.setColor(GREY);
        } else {
          // use the threshold to determine a binary colour
          if (cellProb < threshold) this.display.setColor(WHITE);
          else this.display.setColor(BLACK);
        }

        this.display.fillRectangle(x, y, cellWidth, cellHeight);
      }
           
      // draw lines for the model
      this.display.setColor(GREY);     // Dark Grey
      
      // Draw Vertical Lines
      for (int i=0, x=0; i <= numCellsInRow; i++, x+= cellWidth) {    // Draw extra border after last cell
        this.display.drawLine(x, 0, x, this.deviceHeight);
      }           
      
      // Draw Horizontal Lines
      for (int i=0, y=0; i <= numCellsInCol; i++, y+= cellHeight) {    // Draw extra border after last cell
        this.display.drawLine(0, y, this.deviceWidth, y);
      }           
    }
   
    // Draw Robot Body          
    this.display.setColor(WHITE);               // White
    this.display.fillOval(mapX(this.robotPose.getX()), mapY(this.robotPose.getY()), scale(this.radius), scale(this.radius));
    this.display.setColor(DARKGREY);            // Dark Grey
    this.display.drawOval(mapX(this.robotPose.getX()), mapY(this.robotPose.getY()), scale(this.radius), scale(this.radius));

    // Need to indicate heading          
    double headingX = this.robotPose.getX() + Math.cos(this.robotPose.getTheta()) * this.radius;
    double headingY = this.robotPose.getY() + Math.sin(this.robotPose.getTheta()) * this.radius;
    this.display.setColor(DARKGREY);                      // Dark Grey
    this.display.drawLine(mapX(this.robotPose.getX()), mapY(this.robotPose.getY()), mapX(headingX), mapY(headingY));
                              
  }
}