package frc.robot.subsystems;

import frc.robot.ControlLoop;
import frc.robot.RobotInput;
import frc.robot.subsystems.Subsystem;
import frc.robot.PIDController;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import jaci.pathfinder.followers.EncoderFollower;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import java.io.File;

public class Drivetrain extends Subsystem {
    private RobotInput input = RobotInput.getInstance();

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private TalonSRX m_leftController1;
    private TalonSRX m_leftController2;
    private TalonSRX m_rightController1;
    private TalonSRX m_rightController2;

    private RobotInput driverStation;

    private Encoder leftEncoder;
    private Encoder rightEncoder;

    private static double mQuickStopAccumulator;
    private static final double kStickDeadband = 0.1;
    private static final double kWheelDeadband = 0.1;
    private static final double kTurnSensitivity = 1.0;

    private double k_P = 0.01;
    private double k_I = 0.00005;
    private double k_D = 0.0;

    private double k_pathP = 0.0;
    private double k_pathI = 0.0;
    private double k_pathD = 0.0;
    private double k_wheelBaseWidth = 0.5;

    public static final double kDefaultQuickStopThreshold = 0.2;
    public static final double kDefaultQuickStopAlpha = 0.1;

    private ADXRS450_Gyro m_gyro;

    private double m_quickStopThreshold = kDefaultQuickStopThreshold;
    private double m_quickStopAlpha = kDefaultQuickStopAlpha;
    private double m_quickStopAccumulator;
    private double m_rightSideInvertMultiplier = -1.0;

    //private Joystick joy0 = new Joystick(0);
    //private Joystick joy1 = new Joystick(1);

    private boolean aligning = false;
    private double m_gyroOffset;
    private double m_angleSetpoint;

    private double k_motionMagicAcceptableError;

    private PIDController m_PIDController;

    Waypoint[] points = new Waypoint[] {
        new Waypoint(-1, 0, 0),
        new Waypoint(0, 0, 0)
    };

    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
    Trajectory trajectory = Pathfinder.generate(points, config);

    //File myFile = new File("myfile.csv");
    //Pathfinder.writeToCSV(myFile, trajectory);

    // Wheelbase Width = 0.5m
    TankModifier modifier = new TankModifier(trajectory).modify(0.5);

    // Do something with the new Trajectories...
    Trajectory left = modifier.getLeftTrajectory();
    Trajectory right = modifier.getRightTrajectory();

    EncoderFollower leftEnc = new EncoderFollower(modifier.getLeftTrajectory());
    EncoderFollower rightEnc = new EncoderFollower(modifier.getRightTrajectory());

    public static boolean m_followingAuto = false;

    public Drivetrain() {
        m_leftController1 = new TalonSRX(1);
        m_leftController2 = new TalonSRX(2);
        m_rightController1 = new TalonSRX(3);
        m_rightController2 = new TalonSRX(4);

        this.driverStation = RobotInput.getInstance();

        this.m_PIDController = new PIDController(this.k_P, this.k_I, this.k_D);

        m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
        m_gyro.calibrate();

        leftEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
        rightEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        leftEncoder.setReverseDirection(true);

        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void zero() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public final ControlLoop controlLoop = new ControlLoop() {
        @Override
        public void loopPeriodic() {

            // Drive under operator control during teleop.
            //if (gameState.equals("Teleop")) {
                double[] driveSpeed = {0.0, 0.0};
                //driveSpeed = CurvatureDrive(input.throttleStick.getY(), input.wheel.getX(), input.quickTurnButton.get());
                /*if (input.autoAlignButton.get()) {
                    NetworkTableEntry ledMode = table.getEntry("ledMode");
                    ledMode.setNumber(3);
                    aligning = true;
                    NetworkTableEntry tx = table.getEntry("tx");
                    m_PIDController.setSetpoint(tx.getDouble(0.0));
                    m_gyroOffset = m_gyro.getAngle();
                }*/
                
                if (aligning) {
                    if (Math.abs(m_gyro.getAngle() - m_gyroOffset - m_PIDController.setpoint) < 5) {
                        aligning = false;
                    } else {
                        double turn = 0.75 * m_PIDController.calculate(m_gyro.getAngle() - m_gyroOffset, deltaTime);

                        //System.out.println(turn);
                        driveSpeed = ArcadeDrive(input.switchController.getY(), -turn);
                    }
                } else {
                    NetworkTableEntry ledMode = table.getEntry("ledMode");
                    ledMode.setNumber(1);

                    driveSpeed = CurvatureDrive(input.switchController.getY(), input.switchController.getX(), (input.switchController.getZ() > 0.8));
                }

                double currHeight = Elevator.m_middleStageHeight * 2 + Elevator.m_innerStageHeight;
                double maxSpeed = (Math.pow(currHeight, 2) / -7000.0) + 1.1;

                m_leftController1.set(ControlMode.PercentOutput, driveSpeed[0]);
                m_leftController2.set(ControlMode.PercentOutput, driveSpeed[0]);
                m_rightController1.set(ControlMode.PercentOutput, driveSpeed[1]);
                m_rightController2.set(ControlMode.PercentOutput, driveSpeed[1]);
            //} else if (gameState.equals("Autonomous")) {
                /*if (m_followingAuto) {
                    followTrajectory(leftEnc, rightEnc);
                } else {
                    m_leftController1.set(ControlMode.PercentOutput, 0.0);
                    m_leftController2.set(ControlMode.PercentOutput, 0.0);
                    m_rightController1.set(ControlMode.PercentOutput, 0.0);
                    m_rightController2.set(ControlMode.PercentOutput, 0.0);
                }*/
            //}

            //System.out.println("L: " + leftEncoder.get());
            //System.out.println("R: " + rightEncoder.get() + "\n");
        }
    };

    public void followTrajectory(EncoderFollower left, EncoderFollower right) {
        left.configureEncoder(leftEncoder.get(), 1000, 0.1524);
        left.configurePIDVA(1.0, 0.0, 0.0, 1 / 1.7, 0);

        left.configureEncoder(rightEncoder.get(), 1000, 0.1524);
        left.configurePIDVA(1.0, 0.0, 0.0, 1 / 1.7, 0);

        double l = left.calculate(leftEncoder.get());
        double r = right.calculate(rightEncoder.get());

        double gyro_heading = m_gyro.getAngle();    // Assuming the gyro is giving a value in degrees
        double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees

        // This allows the angle difference to respect 'wrapping', where 360 and 0 are the same value
        double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
        angleDifference = angleDifference % 360.0;
        if (Math.abs(angleDifference) > 180.0) {
            angleDifference = (angleDifference > 0) ? angleDifference - 360 : angleDifference + 360;
        } 

        double turn = 0.8 * (-1.0/80.0) * angleDifference;

        m_leftController1.set(ControlMode.PercentOutput, l + turn);
        m_leftController2.set(ControlMode.PercentOutput, l + turn);
        m_rightController1.set(ControlMode.PercentOutput, r - turn);
        m_rightController2.set(ControlMode.PercentOutput, r - turn);
    }

    public double[] ArcadeDrive(double throttle, double rotation) {
        double left = throttle - rotation;
        double right = throttle + rotation;
        double[] speed = {left, -right};
        return speed;
    }

    public void AutoDrive(double throttle, double rotation) {
        double left = throttle - rotation;
        double right = throttle + rotation;
        m_leftController1.set(ControlMode.PercentOutput, left);
        m_leftController2.set(ControlMode.PercentOutput, left);
        m_rightController1.set(ControlMode.PercentOutput, -right);
        m_rightController2.set(ControlMode.PercentOutput, -right);
    }

    // Curvature Drive adapted from 254 and 971's drive code.
    public double[] CurvatureDrive(double throttle, double rotation, boolean quickTurn) {
        double xSpeed = applyDeadzone(throttle, kStickDeadband);
        double zRotation = applyDeadzone(rotation, kWheelDeadband);

        double angularPower;
        boolean overPower;

        if (quickTurn) {
            if (Math.abs(xSpeed) < m_quickStopThreshold) {
                m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
                    + m_quickStopAlpha * zRotation * 2;
            }
            overPower = true;
            angularPower = zRotation;
        } else {
            overPower = false;
            angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

            if (m_quickStopAccumulator > 1) {
                m_quickStopAccumulator -= 1;
            } else if (m_quickStopAccumulator < -1) {
                m_quickStopAccumulator += 1;
            } else {
                m_quickStopAccumulator = 0.0;
            }
        }

        double leftMotorOutput = xSpeed + angularPower;
        double rightMotorOutput = xSpeed - angularPower;

        if (overPower) {
            if (leftMotorOutput > 1.0) {
                rightMotorOutput -= leftMotorOutput - 1.0;
                leftMotorOutput = 1.0;
            } else if (rightMotorOutput > 1.0) {
                leftMotorOutput -= rightMotorOutput - 1.0;
                rightMotorOutput = 1.0;
            } else if (leftMotorOutput < -1.0) {
                rightMotorOutput -= leftMotorOutput + 1.0;
                leftMotorOutput = -1.0;
            } else if (rightMotorOutput < -1.0) {
                leftMotorOutput -= rightMotorOutput + 1.0;
                rightMotorOutput = -1.0;
            }
        }
/*
        if (Elevator.m_encoder.get() >= 611) {
            double speedLimit = (550.0 / Elevator.m_encoder.get()) + 0.1;
            leftMotorOutput = clamp(leftMotorOutput, -speedLimit, speedLimit);
            rightMotorOutput = clamp(rightMotorOutput, -speedLimit, speedLimit);
        }
*/
        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude;
            rightMotorOutput /= maxMagnitude;
        }

        double leftSpeed = leftMotorOutput;
        double rightSpeed = rightMotorOutput * m_rightSideInvertMultiplier;

        double[] speed = {leftSpeed, rightSpeed};

        return speed;
    }

    public double applyDeadzone(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}