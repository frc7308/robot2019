package frc.robot.subsystems;

import frc.robot.ControlLoop;
import frc.robot.RobotInput;
import frc.robot.subsystems.Subsystem;
import frc.robot.PIDController;
import frc.robot.VelocityTracker;
import frc.robot.subsystems.Elevator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Boom extends Subsystem {
    private RobotInput input = RobotInput.getInstance();

    public static TalonSRX m_boomController;

    private double k_encoderTicksPerRevolutions = 4096;

    private double k_arbitraryFeedforward = -0.475;
    private double k_F = 0.0;
    private double k_P = 0.55;
    private double k_I = 0.01;
    private double k_D = 20.0;

    /*private double k_feedforward = 0.0;
    private double k_P = 0.4;
    private double k_I = 0.0003;
    private double k_D = 0.0;

    private double k_P_DOWN = 0.00005;
    private double k_I_DOWN = 0.00031;
    private double k_D_DOWN = 10.0;*/

    public static double m_setpoint = 0;
    public static double k_acceptableRange = 0.2;
    public static double k_acceptableRangeTicks = 145;

    private double k_gravityCoefficient = -0.3;

    private int k_timeoutMs = 100;

    private PIDController m_boomPIDController;

    private DoubleSolenoid brakeSolenoid;

    private int m_goalPositionID;
    private int m_prevPositionID;
    private double m_brakeThreshold;

    private int m_boomState = 0; // 0 = safe, 1 = cargo delivery, 2 = forward, 3 = cargo pick-up

    public Boom() {
        m_boomController = new TalonSRX(7);

        m_boomController.setNeutralMode(NeutralMode.Brake);
        m_boomController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);

        m_boomController.setSensorPhase(false);
        m_boomController.setInverted(false);

        m_boomPIDController = new PIDController(k_P, k_I, k_D) {
            @Override
            public double getFeedforward(double currentPosition) {
                double angle = currentPosition;
                //System.out.println(setpoint - angle);
                return (Math.sin(angle) * k_gravityCoefficient);
            }
        };

        brakeSolenoid = new DoubleSolenoid(4, 5);

        m_boomController.configMotionCruiseVelocity(150);
        m_boomController.configMotionAcceleration(50);

        m_boomController.configNominalOutputForward(0, k_timeoutMs);
        m_boomController.configNominalOutputReverse(0, k_timeoutMs);
		m_boomController.configPeakOutputForward(0.85, k_timeoutMs);
        m_boomController.configPeakOutputReverse(-0.85, k_timeoutMs);

        //m_boomController.selectProfileSlot(0, k_timeoutMs);
		m_boomController.config_kF(0, k_F, k_timeoutMs);
		m_boomController.config_kP(0, k_P, k_timeoutMs);
		m_boomController.config_kI(0, k_I, k_timeoutMs);
        m_boomController.config_kD(0, k_D, k_timeoutMs);

        m_boomController.configAllowableClosedloopError(0, 50, k_timeoutMs);

        //m_boomController.configMaxIntegralAccumulator(0, iaccum);
        m_boomController.config_IntegralZone(0, 300);

        zero();

        //m_boomController.setSelectedSensorPosition(0, 0, k_timeoutMs);
    }

    public void zero() {
        m_boomController.setSelectedSensorPosition(0, 0, k_timeoutMs);
    }

    public final ControlLoop controlLoop = new ControlLoop() {
        @Override
        public void loopPeriodic() {
            //brakeSolenoid.set(DoubleSolenoid.Value.kReverse);

            //System.out.println("Test");
            
            double boomPosition = encoderTicksToRadians(m_boomController.getSelectedSensorPosition());
            double boomVelocity = (m_boomController.getSelectedSensorVelocity() * 10);

            //System.out.println(boomVelocity + " rads/sec");

            //m_boomPIDController.setSetpoint(-1.57);
            //m_boomController.set(ControlMode.PercentOutput, m_boomPIDController.calculate(boomVelocity, this.deltaTime));

            //m_boomController.config_kF(0, calculateFeedForward(, ), k_timeoutMs);

            if (input.extraButton.get()) {
                zero();
            }

            //if (gameState.equals("Teleop")) {
            if (input.safePositionButton.get()) {
                m_boomState = 0;
            }
            if ((input.cargoDeliverButton.get())) {
                m_boomState = 1;
            }
            if (((input.lowDeliverButton.get() || input.midDeliverButton.get() || input.highDeliverButton.get()) && m_boomState == 0 && m_boomController.getClosedLoopError() < 150) && input.robotDirection == true) {
                m_boomState = 2;
            }
            if (((input.lowDeliverButton.get() || input.midDeliverButton.get() || input.highDeliverButton.get()) && m_boomState == 0 && m_boomController.getClosedLoopError() < 150) && input.robotDirection == false) {
                m_boomState = 3;
            }
            if (input.cargoPickupButton.get() && m_boomState == 0 && m_boomController.getClosedLoopError() < 150) {
                System.out.println("HI");
                input.robotMode = false;
                m_boomState = 4;
            }
            if (input.backModeButton.get()) {
                input.robotDirection = false;
            }
            if (input.frontModeButton.get()) {
                input.robotDirection = true;
            }
            if (input.backModeButton.get()) {
                input.robotDirection = false;
            }

                
            if (m_boomState == 0) {
                m_setpoint = 0;
            }
            if (m_boomState == 1) {
                if (m_setpoint != 1500 || m_setpoint != -1500) {
                    if (input.robotMode == false) {
                        if (input.robotDirection == true) {
                            m_setpoint = 800;
                        } else {
                            m_setpoint = 800;
                        }
                    }
                }
            }
            if (m_boomState == 2) {
                if (m_setpoint != 1500 || m_setpoint != -1500) {
                    if (input.robotMode == true) {
                        m_setpoint = 1100;
                    } else {
                        m_setpoint = 800;
                    }
                }
            }
            if (m_boomState == 3) {
                if (m_setpoint != 1500 || m_setpoint != -1500) {
                    if (input.robotMode == true) {
                        m_setpoint = -1100;
                    } else {
                        m_setpoint = -800;
                    }
                }
            }
            if (m_boomState == 4) {
                if (m_setpoint != 1500 || m_setpoint != -1500) {
                    if (input.robotDirection == true) {
                        m_setpoint = 1500;
                    } else {
                        m_setpoint = -1500;
                    }
                }
            }

            //}

            double error = boomPosition - encoderTicksToRadians(m_setpoint);
            double targetVelocity = 0;
            if (Math.abs(error) < k_acceptableRange) {
                brakeSolenoid.set(DoubleSolenoid.Value.kReverse);
            } else {
                brakeSolenoid.set(DoubleSolenoid.Value.kForward);
            }
            //m_boomController.set(ControlMode.MotionMagic, m_setpoint, DemandType.ArbitraryFeedForward, calculateFeedForward(boomPosition));
            m_boomController.set(ControlMode.Position, m_setpoint, DemandType.ArbitraryFeedForward, calculateFeedForward(boomPosition));
            
            //m_boomController.set(ControlMode.PercentOutput, calculateFeedForward(boomPosition));

            //radiansToEncoderTicks(speed) / 50;

            SmartDashboard.putNumber("Target Velocity", encoderTicksToRadians(m_setpoint));
            SmartDashboard.putNumber("Actual Velocity", boomPosition);
            SmartDashboard.putNumber("Error", error);

            System.out.println(m_setpoint);//m_boomController.getClosedLoopError());

            //System.out.println(calculateFeedForward(boomPosition));

            /*if (bruhJoy.getTrigger()) {
                m_boomController.setSelectedSensorPosition(0, 0, k_timeoutMs);
                brakeSolenoid.set(DoubleSolenoid.Value.kForward);
            }*/

            /*double targetPos = Math.PI / 2.0;
            m_boomPIDController.setSetpoint(targetPos);
            if (bruhJoy.getTrigger()) {
                //m_boomController.setSelectedSensorPosition(0, 0, k_timeoutMs);
                amDoingStuff = true;
            }

            if (bruhJoy2.getTrigger()) {
                m_boomController.setSelectedSensorPosition(0, 0, k_timeoutMs);
                amDoingStuff = false;
                System.out.println("yo");
            }

            if (m_goalPositionID == 0) {
                if (m_prevPositionID == -2) {
                    m_brakeThreshold = 0.18;
                } else if (m_prevPositionID == 2) {
                    m_brakeThreshold = 0.25;
                }
            }

            //System.out.println(m_boomController.getMotorOutputPercent());

            //System.out.println(m_boomController.getMotorOutputPercent());

            //System.out.println(m_boomController.getSelectedSensorPosition());

            //System.out.println(m_boomController.getMotorOutputPercent() + " -- " + m_boomController.getSelectedSensorPosition());
            
            //m_boomController.set(ControlMode.PercentOutput, -0.1);
            if (Math.abs(m_boomPIDController.setpoint - boomPosition) < 0.35 && amDoingStuff) { // front 0.18 back 0.26 // front boom 0.3
                brakeSolenoid.set(DoubleSolenoid.Value.kForward);
                m_boomController.set(ControlMode.PercentOutput, 0);
            } else {
                brakeSolenoid.set(DoubleSolenoid.Value.kReverse);
                if (amDoingStuff) {
                    m_boomController.set(ControlMode.PercentOutput, clamp(m_boomPIDController.calculate(boomPosition, this.deltaTime), -1, 1));
                }
            }*/
        }
    };

    public double encoderTicksToRadians(double ticks) {
        return ticks * 2.0 * Math.PI / k_encoderTicksPerRevolutions;
    }

    public double radiansToEncoderTicks(double radians) {
        return radians * k_encoderTicksPerRevolutions / 2.0 * Math.PI;
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private double calculateFeedForward(double angle) {
        return (Math.sin(angle) * k_arbitraryFeedforward);
    }
}