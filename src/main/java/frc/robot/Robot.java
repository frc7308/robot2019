/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import com.mach.LightDrive.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.cscore.UsbCamera;

import java.sql.Struct;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;
import java.util.ArrayList;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.LoopMaster;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Boom;
import frc.robot.subsystems.Drivetrain;
import frc.robot.auto.StraightToHoldAuto;

public class Robot extends TimedRobot {
  /*public static Elevator elevator;
  public static Intake intake;
  public static Boom boom;
  public static Drivetrain drivetrain;
  private Compressor compressor;
  private LoopMaster loopMaster;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  Joystick joy1 = new Joystick(0);
  Joystick joy2 = new Joystick(1);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  private SendableChooser autChooser;
  private CommandGroup StraightToHold;*/

  private UsbCamera testCamera;
  WheelColor _lastRecognizedColor;

	//LightDriveCAN ledController;

  @Override
  public void robotInit() {

    //this.elevator = new Elevator();
    /*this.intake = new Intake();
    this.drivetrain = new Drivetrain();
    //this.boom = new Boom();

    this.loopMaster = new LoopMaster();
    //loopMaster.addLoop(elevator.controlLoop);
    loopMaster.addLoop(intake.controlLoop);
    loopMaster.addLoop(drivetrain.controlLoop);
    //loopMaster.addLoop(boom.controlLoop);
    loopMaster.start();

    this.compressor = new Compressor();
    compressor.start();

    UsbCamera backCam = CameraServer.getInstance().startAutomaticCapture();
    backCam.setResolution(320, 240);
    backCam.setFPS(30);

    /*ledController = new LightDriveCAN();
    ledController.SetColor(1, java.awt.Color.BLUE);
    ledController.SetColor(2, java.awt.Color.BLUE, 1.0);
    ledController.Update();*/

    /*StraightToHold = new StraightToHoldAuto();
    NetworkTableEntry ledMode = table.getEntry("ledMode");
    ledMode.setNumber(1);*/

    CameraServer.getInstance().startAutomaticCapture();
    _lastRecognizedColor = WheelColor.UNKNOWN;

  }

  public enum WheelColor  {
  BLUE, GREEN, RED, YELLOW, UNKNOWN
  }

  public String wheelColorDescription (WheelColor wheelColor) {
    String description = "Unknown";
    
    switch(wheelColor) {

      case BLUE:
        description = "Blue";
        break;
      case RED:
        description = "Red";
        break;
      case GREEN:
        description = "Green";
        break;
      case YELLOW:
        description = "Yellow";
        break;
      default:
        break;
    }
    return description;
  }

  @Override
  public void robotPeriodic() {

    // Creates the CvSink and connects it to the UsbCamera
    var cvSink = CameraServer.getInstance().getVideo();
  
    // create a matrix that corresponds to the pixels of the image
    Mat imageMatrix = new Mat();
    cvSink.grabFrame(imageMatrix);

    /*  TODO: Refactor the next bit of code into it's own function
        that can take a pixel value in and use that value to generate a box 
        around the middle of the screen and grab the average color of all the edges
    */
    // Grab the values from the center of the image print
    double[] pixelA = imageMatrix.get((int)(Math.floor(imageMatrix.rows() /  2)), (int)(Math.floor(imageMatrix.cols() / 2)));
    double[] pixelB = imageMatrix.get((int)(Math.ceil(imageMatrix.rows() /  2)), (int)(Math.floor(imageMatrix.cols() / 2)));
    double[] pixelC = imageMatrix.get((int)(Math.floor(imageMatrix.rows() /  2)), (int)(Math.ceil(imageMatrix.cols() / 2)));
    double[] pixelD = imageMatrix.get((int)(Math.ceil(imageMatrix.rows() /  2)), (int)(Math.ceil(imageMatrix.cols() / 2)));

    var testThing = imageMatrix.get(0,0);

    if (testThing != null) {
      double cyanSum = 0;
      double magentaSum = 0; 
      double yellowSum = 0;
      double kSum = 0;

      double[] pixel1 = (double[])imageMatrix.get((int)(Math.floor(imageMatrix.rows() /  2)), (int)(Math.floor(imageMatrix.cols() / 2)));
      cyanSum += pixel1[0];
      magentaSum += pixel1[1];
      yellowSum += pixel1[2];
      // kSum += pixel1[3];

      double[] pixel2 = (double[])imageMatrix.get((int)(Math.ceil(imageMatrix.rows() /  2)), (int)(Math.floor(imageMatrix.cols() / 2)));
      cyanSum += pixel2[0];
      magentaSum += pixel2[1];
      yellowSum += pixel2[2];
      // kSum += pixel2[3];

      double[] pixel3 = (double[])imageMatrix.get((int)(Math.floor(imageMatrix.rows() /  2)), (int)(Math.ceil(imageMatrix.cols() / 2)));
      cyanSum +=  pixel3[0];
      magentaSum += pixel3[1];
      yellowSum += pixel3[2];
      // kSum += pixel3[3];

      double[] pixel4 = (double[])imageMatrix.get((int)(Math.ceil(imageMatrix.rows() /  2)), (int)(Math.ceil(imageMatrix.cols() / 2)));
      cyanSum += pixel4[0];
      magentaSum += pixel4[1];
      yellowSum += pixel4[2];
      // kSum += pixel4[3];

      double cyanAverage  = cyanSum / 4.0;
      double magentaAverage = magentaSum / 4.0;
      double yellowAverage = yellowSum / 4.0;
      // double kAverage = kSum / 4.0;

      WheelColor newRecognizedColor = WheelColor.UNKNOWN;

      // TODO: Should retest this values with the better camera
      if (cyanAverage < 100 && cyanAverage > 60 && magentaAverage > 170 && yellowAverage > 140  && yellowAverage < 150) {
        // Yellow
        // TODO: Needs more tuning to get this color right. 
        newRecognizedColor = WheelColor.YELLOW;
      }
      else if (cyanAverage < 160 && cyanAverage > 60 && magentaAverage > 90 && magentaAverage < 110 && yellowAverage > 220){
        // Red 
        newRecognizedColor = WheelColor.RED;
      }
      else if (cyanAverage > 200 && magentaAverage < 160 && magentaAverage > 140 && yellowAverage < 50){
        // Blue
        newRecognizedColor = WheelColor.BLUE; 
      }
      else if (cyanAverage > 120 && cyanAverage < 150 && magentaAverage < 175 && magentaAverage > 150 && yellowAverage > 80 && yellowAverage < 100){
        // Green
        newRecognizedColor = WheelColor.GREEN;
      }
      else {
        // Unknown
        newRecognizedColor = WheelColor.UNKNOWN;
      }


      if (newRecognizedColor != _lastRecognizedColor) {
        // only apply this if the code has changed. 
        _lastRecognizedColor = newRecognizedColor;
      }

      System.out.println("Current Color is: " + wheelColorDescription(_lastRecognizedColor));
      if (_lastRecognizedColor == WheelColor.UNKNOWN) {
        // We don't know what the color is, print out the raw values for tuning. 
        System.out.println(cyanAverage + " " + magentaAverage + " " + yellowAverage + " "/* + kAverage*/);
      }
  }

 

  // test to see if testThing or something like that is null
  // if not, then print out the RBG numberss
  // put colors up to camera and figure out range of RBG numbers for each color

  

  // METHOD 2: Full image, color percentages
  Mat threshMatrix = new Mat();
  Core.inRange(imageMatrix, new Scalar(0,0,0), new Scalar(0,0,255), threshMatrix);

  // Count pixels 


    /*NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    System.out.println(table.getKeys());

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);*/

    /*if (joy1.getTrigger()) {
      sol.set(Value.kForward);
    } else if (joy2.getTrigger()) {
      sol.set(Value.kReverse);
    } else {
      sol.set(Value.kOff);
    }*/
  }

  @Override
  public void autonomousInit() {
    //loopMaster.setGameState("Autonomous");

    //SmartDashboard.putString("Mode", "Autonomous");

    // Run the appropriate auto based on game condition and driver station selection.

    //String selectedAuto = (String) autoChooser.getSelected();
    //StraightToHold.start();
    // Run autos based on selection
  }

  @Override
  public void autonomousPeriodic() {
    //Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    //loopMaster.setGameState("Teleop");
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() {
    //loopMaster.setGameState("Disabled");
  }
}
