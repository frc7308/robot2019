package frc.robot.subsystems;

import java.util.ArrayList; 

import frc.robot.ControlLoop;
import frc.robot.RobotInput;
import frc.robot.subsystems.Subsystem;
import frc.robot.PIDController;
import frc.robot.Robot;
import frc.robot.AnalogEncoder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class Elevator extends Subsystem {
    private RobotInput input = RobotInput.getInstance();

    public static TalonSRX m_innerStageController;
    public static TalonSRX m_middleStageController;
    private static AnalogEncoder m_innerStageEncoder;
    private static AnalogEncoder m_middleStageEncoder;

    private static PIDController m_innerStagePIDController;
    private static PIDController m_middleStagePIDController;

    private int k_encoderTicksPerRevolution = 4096;
    private double k_pulleyDiameter = 2.58;
    private double k_encoderGearRatio = 1.75;
    private double k_encoderTicksPerInch = 884.358630135;

    private double k_innerStageFeedforward = 0.25;
    private double k_innerStageP = 0.8;
    private double k_innerStageI = 0.0001;
    private double k_innerStageD = 10.0;

    private double k_middleStageFeedforward = 0.325;
    private double k_middleStageP = 0.2;
    private double k_middleStageI = 0.0001;
    private double k_middleStageD = 10.0;

    public static double k_acceptableEncoderError = 1.0; // In Inches

    private double m_heightState = 0; // 0 = Low, 1 = Mid, 2 = High, 3 = High->Low, 4 = Ball Pickup, 5 = Cargo Hold
    private double m_movementState = 0; // 0 = In place, 1 = Moving
    private double m_motionStage = 0;
    private boolean m_moving = false;

    private int m_maxVal;

    private double prev_error;
    private ArrayList<Double> integral = new ArrayList<Double>();

    private boolean m_innerStageInPID = false;

    public static double m_innerStageHeight;
    public static double m_middleStageHeight;

    public Elevator() {
        this.m_innerStageController = new TalonSRX(5);
        this.m_middleStageController = new TalonSRX(6);

        this.m_innerStageEncoder = new AnalogEncoder(1, this.k_encoderTicksPerRevolution, this.k_encoderTicksPerInch, false);
        this.m_innerStageEncoder.zero();

        this.m_middleStageEncoder = new AnalogEncoder(0, this.k_encoderTicksPerRevolution, this.k_encoderTicksPerInch, false);
        this.m_middleStageEncoder.zero();

        m_innerStageController.setNeutralMode(NeutralMode.Brake);
        m_middleStageController.setNeutralMode(NeutralMode.Brake);

        m_innerStagePIDController = new PIDController(k_innerStageP, k_innerStageI, k_innerStageD) {
            @Override
            public double getFeedforward(double currentPosition) {
                return k_innerStageFeedforward;
            }
        };

        m_middleStagePIDController = new PIDController(k_middleStageP, k_middleStageI, k_middleStageD) {
            @Override
            public double getFeedforward(double currentPosition) {
                return k_middleStageFeedforward;
            }
        };
    }

    public void zero() {
        m_innerStageEncoder.zero();
        m_middleStageEncoder.zero();
    }

    public final ControlLoop controlLoop = new ControlLoop() {
        @Override
        public void loopPeriodic() {
            m_innerStageEncoder.update();
            m_middleStageEncoder.update();
            m_innerStageHeight = m_innerStageEncoder.getPosition();
            m_middleStageHeight = m_middleStageEncoder.getPosition();

            if (input.extraButton.get()) {
                m_innerStageEncoder.zero();
                m_middleStageEncoder.zero();
                m_innerStageController.set(ControlMode.PercentOutput, 0.3);
                m_middleStageController.set(ControlMode.PercentOutput, 0.3);
            } else {
                if (m_heightState == 0) {
                    m_innerStagePIDController.setSetpoint(0);
                    m_middleStagePIDController.setSetpoint(0);
                    if (m_innerStageHeight > 1.5) {
                        m_innerStageController.set(ControlMode.PercentOutput, clamp(m_innerStagePIDController.calculate(m_innerStageHeight, this.deltaTime), -0.6, 1.0));
                    } else {
                        m_innerStageController.set(ControlMode.PercentOutput, 0);
                    }
                    if (Math.abs(m_innerStageHeight - (m_innerStagePIDController.setpoint)) < k_acceptableEncoderError) {
                        m_moving = false;
                    }

                    if (input.midDeliverButton.get() && !m_moving) {
                        m_moving = true;
                        m_heightState = 1;
                        m_motionStage = 0;
                    }
                    if (input.highDeliverButton.get() && !m_moving) {
                        m_moving = true;
                        m_heightState = 2;
                        m_motionStage = 0;
                    }
                    if (input.cargoPickupButton.get() && !m_moving && input.robotMode == false) {
                        m_moving = true;
                        m_heightState = 4;
                        m_motionStage = 0;
                    }
                    if (input.cargoDeliverButton.get() && !m_moving && input.robotMode == false) {
                        m_moving = true;
                        m_heightState = 5;
                        m_motionStage = 0;
                    }

                }
                if (m_heightState == 1) {
                    m_innerStagePIDController.setSetpoint(28);
                    m_middleStagePIDController.setSetpoint(0);
                    m_innerStageController.set(ControlMode.PercentOutput, clamp(m_innerStagePIDController.calculate(m_innerStageHeight, this.deltaTime), -0.6, 1.0));
                    m_middleStageController.set(ControlMode.PercentOutput, 0);
                    if (Math.abs(m_innerStageHeight - (m_innerStagePIDController.setpoint)) < k_acceptableEncoderError) {
                        m_moving = false;
                    }
                    if ((input.lowDeliverButton.get() || input.safePositionButton.get()) && !m_moving) {
                        m_moving = true;
                        m_heightState = 0;
                        m_motionStage = 0;
                    }
                    if (input.cargoPickupButton.get() && !m_moving && input.robotMode == false) {
                        m_moving = true;
                        m_heightState = 4;
                        m_motionStage = 0;
                    }
                } else if (m_heightState == 2) {
                    if (m_motionStage == 0) {
                        m_innerStagePIDController.setSetpoint(28);
                        m_innerStageController.set(ControlMode.PercentOutput, clamp(m_innerStagePIDController.calculate(m_innerStageHeight, this.deltaTime), -0.6, 1.0));
                        m_middleStageController.set(ControlMode.PercentOutput, 0.0);
                        if (Math.abs(m_innerStageHeight - m_innerStagePIDController.setpoint) < k_acceptableEncoderError) {
                            m_innerStagePIDController.setSetpoint(0);
                            m_middleStagePIDController.setSetpoint(28);
                            m_motionStage = 1;
                        }
                    } else if (m_motionStage == 1) {
                        m_innerStageController.set(ControlMode.PercentOutput, clamp(m_innerStagePIDController.calculate(m_innerStageHeight, this.deltaTime), -1.0, 1.0) * 0.1625 * 0.6);
                        m_middleStageController.set(ControlMode.PercentOutput, clamp(m_middleStagePIDController.calculate(m_middleStageHeight, this.deltaTime), -0.6, 1.0));
                        if (Math.abs(m_innerStageHeight - (m_innerStagePIDController.setpoint)) < k_acceptableEncoderError && Math.abs(m_middleStageHeight - (m_middleStagePIDController.setpoint)) < k_acceptableEncoderError) {
                            m_motionStage = 2;
                        }
                    } else if (m_motionStage == 2) {
                        m_innerStageController.set(ControlMode.PercentOutput, clamp(m_innerStagePIDController.calculate(m_innerStageHeight, this.deltaTime), -0.6, 1.0));
                        m_middleStageController.set(ControlMode.PercentOutput, clamp(m_middleStagePIDController.calculate(m_middleStageHeight, this.deltaTime), -0.6, 1.0));
                        m_moving = false;
                    }

                    if ((input.lowDeliverButton.get() || input.safePositionButton.get()) && !m_moving) {
                        m_moving = true;
                        m_heightState = 3;
                        m_motionStage = 0;
                    }
                    if (input.cargoPickupButton.get() && !m_moving && input.robotMode == false) {
                        m_moving = true;
                        m_heightState = 4;
                        m_motionStage = 0;
                    }
                } else if (m_heightState == 3) {
                    if (m_motionStage == 0) {
                        m_innerStagePIDController.setSetpoint(28);
                        m_middleStagePIDController.setSetpoint(0);
                        m_innerStageController.set(ControlMode.PercentOutput, clamp(m_innerStagePIDController.calculate(m_innerStageHeight, this.deltaTime), -1.0, 1.0) * 1.15 * 0.6);
                        if (m_middleStageHeight > 1.5) {
                            m_middleStageController.set(ControlMode.PercentOutput, clamp(m_middleStagePIDController.calculate(m_middleStageHeight, this.deltaTime), -0.6, 1.0));
                        } else {
                            m_middleStageController.set(ControlMode.PercentOutput, 0);
                        }
                        if (Math.abs(m_innerStageHeight - m_innerStagePIDController.setpoint) < k_acceptableEncoderError && Math.abs(m_middleStageHeight - (m_middleStagePIDController.setpoint)) < k_acceptableEncoderError) {
                            m_innerStagePIDController.setSetpoint(0);
                            m_motionStage = 1;
                        }
                    } else if (m_motionStage == 1) {
                        if (m_innerStageHeight > 1.5) {
                            m_innerStageController.set(ControlMode.PercentOutput, clamp(m_innerStagePIDController.calculate(m_innerStageHeight, this.deltaTime), -0.6, 1.0));
                        } else {
                            m_innerStageController.set(ControlMode.PercentOutput, 0);
                        }
                        if (Math.abs(m_innerStageHeight - (m_innerStagePIDController.setpoint)) < k_acceptableEncoderError) {
                            m_moving = false;
                            m_heightState = 0;
                            m_motionStage = 0;
                        }
                    }
                } else if (m_heightState == 4) {
                    m_innerStagePIDController.setSetpoint(4.5);
                    m_middleStagePIDController.setSetpoint(0);
                    m_innerStageController.set(ControlMode.PercentOutput, clamp(m_innerStagePIDController.calculate(m_innerStageHeight, this.deltaTime), -0.6, 1.0));
                    m_middleStageController.set(ControlMode.PercentOutput, 0);
                    if (Math.abs(m_innerStageHeight - (m_innerStagePIDController.setpoint)) < k_acceptableEncoderError) {
                        m_moving = false;
                    }
                    if ((input.lowDeliverButton.get() || input.safePositionButton.get()) && !m_moving) {
                        m_moving = true;
                        m_heightState = 0;
                        m_motionStage = 0;
                    }
                } else if (m_heightState == 5) {
                    m_innerStagePIDController.setSetpoint(12);
                    m_middleStagePIDController.setSetpoint(0);
                    m_innerStageController.set(ControlMode.PercentOutput, clamp(m_innerStagePIDController.calculate(m_innerStageHeight, this.deltaTime), -0.6, 1.0));
                    m_middleStageController.set(ControlMode.PercentOutput, 0);
                    if (Math.abs(m_innerStageHeight - (m_innerStagePIDController.setpoint)) < k_acceptableEncoderError) {
                        m_moving = false;
                    }
                    if ((input.lowDeliverButton.get() || input.safePositionButton.get()) && !m_moving) {
                        m_moving = true;
                        m_heightState = 0;
                        m_motionStage = 0;
                    }
                }

                //System.out.println("A: " + (m_innerStageEncoder.getPosition() - m_innerStagePIDController.setpoint));
                //System.out.println(m_innerStageController.getMotorOutputPercent());
            }
        }
    };

    // 0 = Low, 1 = Mid, 2 = High, 3 = Cargo
    /*private setState(int state) {

    }
*/
    private double encoderTicksToHeight(double revolutions, double encoderValue, double encoderOffset) {
        return (((revolutions * this.k_encoderTicksPerRevolution + encoderValue) - encoderOffset) * k_pulleyDiameter * Math.PI) / (k_encoderTicksPerRevolution * k_encoderGearRatio);
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}