 /**
 **********************************************************************************************************************
 * @file       Haptic_Physics_Template.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V3.0.0
 * @date       27-September-2018
 * @brief      Base project template for use with pantograph 2-DOF device and 2-D physics engine
 *             creates a blank world ready for creation
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */
 
 
 
 /* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 3;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           rendering_force                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           pos_ee                              = new PVector(0, 0);
PVector           f_ee                                = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 30.0;  
float             worldHeight                         = 20.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

PVector offset = new PVector(-2, -3.46);
double sens = 5;

boolean start = false;

boolean loadedMap = false;

boolean hit = false;
int corner = 0;
int[] n = new int[6];

float score = 0;

boolean firstStart = true;
int margin = 100;  // Distance from the edge
/* Initialization of virtual tool */
HVirtualCoupling  s;

/* Wall Setup */
int seconds = 0;

int secondsStart = 0;
  
FBox[] mazeWalls = new FBox[12];

float[] wallX = {worldWidth/2+3, worldWidth/2-3, worldWidth/2+6, worldWidth/2-6, worldWidth/2+9, worldWidth/2-9};

//worldWidth/2+4
/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1200, 800);
  
  
  
  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, Serial.list()[3], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  println(Serial.list());
  
  widgetOne.set_mechanism(pantograph);
  

 
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(2); 
  s.h_avatar.setFill(255,0,0); 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), (300.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
  
  
  
  
}


/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  background(255);
  world.draw(); 
  
  if(firstStart == false){
    seconds = millis() / 1000 - secondsStart;
  }
  
  fill(0,0,255);
  textSize(32);
  text("Timer: " + seconds + "s", width-200, 100);
  text("Score: " + score, width-200, 200);
  text("Pts/Minute: " + (score/seconds)*60, width-300, 300);
  
  
  if (start == false){
      
      fill(0, 255, 0); 
      textSize(32); 
      text("Go to green circle", 200, 100); // Draw text at (x=50, y=100)
     
      fill(0, 255, 0); 
      noStroke(); // Remove the outline (optional)
      circle(width/2, height/2, 100); // Draw a circle at the center
      if(loadedMap == true){
        eraseMap();
        loadedMap = false;
      }
      
      
  }else{
      
      if(firstStart){
        secondsStart = millis() / 1000;
        firstStart = false;
      }
      
      fill(255, 0, 0); 
      textSize(32); 
      text("Go to red circle", 200, 100); // Draw text at (x=50, y=100)
      
      if(hit){
        corner = (int)random(4);  // Random value: 0, 1, 2, or 3
        for(int i = 0; i<=5; i++){
          n[i] = (int)random(-6, 7);
        }
        hit = false;
      }
      
      fill(255, 0, 0);  // Set fill color to red
        noStroke();       // Remove the outline (optional)
        
        
        
        
        float x = 0;
        float y = 0;
        
        if (corner == 0) {          // Top-left
          x = margin;
          y = margin;
        } else if (corner == 1) {   // Top-right
          x = width - margin;
          y = margin;
        } else if (corner == 2) {   // Bottom-left
          x = margin;
          y = height - margin;
        } else if (corner == 3) {   // Bottom-right
          x = width - margin;
          y = height - margin;
        }
      
        circle(x, y, 100);  // Draw the circle
        
      
      offset = new PVector(-2, -5);
      if(loadedMap == false){
        loadMap();
        loadedMap = true;
      }
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    rendering_force = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      pos_ee.set(widgetOne.get_device_position(angles.array()));
      pos_ee.set(pos_ee.copy().mult(200));  
      //println(angles.array());
      //println(pos_ee.copy().mult(200));
    }
    
    pos_ee.x += offset.x;
    pos_ee.y += offset.y;
    pos_ee.x *= sens;
    pos_ee.y *= sens;
    
    float toolX = edgeTopLeftX+worldWidth/2-(pos_ee).x;
    float toolY = edgeTopLeftY+(pos_ee).y;
    
    if(start == false){
      if(sqrt(pow((toolX*pixelsPerCentimeter-width/2),2)+pow((toolY*pixelsPerCentimeter-height/2),2)) < 50){
        start = true;
        score++;
        hit = true;
      }
    }
   
   if(corner == 0){
     if(sqrt(pow((toolX*pixelsPerCentimeter-margin),2)+pow((toolY*pixelsPerCentimeter-margin),2)) < 50){
        start = false;
    }
   }else if(corner == 1){
     if(sqrt(pow((toolX*pixelsPerCentimeter-(width-margin)),2)+pow((toolY*pixelsPerCentimeter-margin),2)) < 50){
        start = false;
    }
   }else if(corner == 2){
     if(sqrt(pow((toolX*pixelsPerCentimeter-100),2)+pow((toolY*pixelsPerCentimeter-(height-margin)),2)) < 50){
        start = false;
    }
   }else if(corner == 3){
     if(sqrt(pow((toolX*pixelsPerCentimeter-(width-margin)),2)+pow((toolY*pixelsPerCentimeter-(height-margin)),2)) < 50){
        start = false;
    }
   }
   
   if(sqrt(pow((toolX*pixelsPerCentimeter-100),2)+pow((toolY*pixelsPerCentimeter-100),2)) < 50){
        start = false;
    }
      
      
     s.setToolPosition(toolX, toolY); 
    //println((pos_ee).x, (pos_ee).y);
    //println(toolX*pixelsPerCentimeter, toolY*pixelsPerCentimeter);
    s.updateCouplingForce();
    f_ee.set(-s.getVCforceX(), s.getVCforceY());
    f_ee.div(20000); //
    
    torques.set(widgetOne.set_device_torques(f_ee.array()));
    widgetOne.device_write_torques();
  
    world.step(1.0f/1000.0f);
  
    rendering_force = false;
    
    
  }
}
/* end simulation section **********************************************************************************************/

void loadMap(){
  
  
  
  for(int i = 0; i <= 5; i++){
    mazeWalls[i] = new FBox(1,15);
    mazeWalls[i].setPosition(wallX[i], n[i]);
    mazeWalls[i].setFill(0,0,0);
    mazeWalls[i].setStatic(true);
    world.add(mazeWalls[i]);
  }
  
  for(int i = 6; i <= 11; i++){
    mazeWalls[i] = new FBox(1,15);
    mazeWalls[i].setPosition(wallX[i-6], n[i-6]+17);
    mazeWalls[i].setFill(0,0,0);
    mazeWalls[i].setStatic(true);
    world.add(mazeWalls[i]);
  }
  
  
}

void eraseMap(){
  
  for(int i = 0; i <= 11; i++){
    world.remove(mazeWalls[i]);
  }
}


/* helper functions section, place helper functions here ***************************************************************/

/* end helper functions section ****************************************************************************************/
