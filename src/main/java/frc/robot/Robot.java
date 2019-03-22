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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.LoopMaster;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Boom;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {
  public static Elevator elevator;
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

	//LightDriveCAN ledController;

  @Override
  public void robotInit() {
    this.elevator = new Elevator();
    this.intake = new Intake();
    this.drivetrain = new Drivetrain();
    this.boom = new Boom();

    this.loopMaster = new LoopMaster();
    loopMaster.addLoop(elevator.controlLoop);
    loopMaster.addLoop(intake.controlLoop);
    loopMaster.addLoop(drivetrain.controlLoop);
    loopMaster.addLoop(boom.controlLoop);
    loopMaster.start();

    this.compressor = new Compressor();
    compressor.start();
    
    /*ledController = new LightDriveCAN();
    ledController.SetColor(1, java.awt.Color.BLUE);
    ledController.SetColor(2, java.awt.Color.BLUE, 1.0);
    ledController.Update();*/
  }

  @Override
  public void robotPeriodic() {
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
    loopMaster.setGameState("Autonomous");
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    loopMaster.setGameState("Teleop");
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() {
    loopMaster.setGameState("Disabled");
  }
}
