package frc.robot.subsystems;

import frc.robot.ControlLoop;
import frc.robot.RobotInput;
import frc.robot.subsystems.Subsystem;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Intake extends Subsystem {
    private RobotInput input = RobotInput.getInstance();

    private VictorSPX m_topIntakeController;
    private VictorSPX m_bottomIntakeController;
    public static DoubleSolenoid releaseSolenoid = new DoubleSolenoid(0, 1);
    public static DoubleSolenoid outInSolenoid = new DoubleSolenoid(2, 3);

    /*private JoystickButton velcroOutButton = new JoystickButton(joy, 3);
    private JoystickButton velcroInButton = new JoystickButton(joy, 2);
    private JoystickButton ejectButton = new JoystickButton(joy, 4);*/

    private boolean velcroState = false;
    private boolean spitting = false;
    private boolean pickingUp = false;
    private int rollerMode = 0; // 0 = none, 1 = in, 3 = hold

    private boolean out = false;

    public Intake() {
        m_topIntakeController = new VictorSPX(8);
        m_bottomIntakeController = new VictorSPX(9);
    }

    public final ControlLoop controlLoop = new ControlLoop() {
        @Override
        public void loopPeriodic() {
            if (input.switchController.getThrottle() > 0.8) {
                releaseSolenoid.set(DoubleSolenoid.Value.kReverse);
            } else {
                releaseSolenoid.set(DoubleSolenoid.Value.kForward);
            }
            if (input.outButton.get()) {
                out = true;
            } else if (input.inButton.get()) {
                out = false;
            }

            if (out) {
                outInSolenoid.set(DoubleSolenoid.Value.kReverse);
            } else {
                outInSolenoid.set(DoubleSolenoid.Value.kForward);
            }

            //if (gameState.equals("Teleop")) {
                /*if (input.hatchModeButton.get()) {
                    input.robotMode = true;
                }
                if (input.cargoModeButton.get()) {
                    input.robotMode = false;
                }

                if (input.switchController.getThrottle() > 0.8) {
                    spitting = true;
                } else {
                    spitting = false;
                }
                if (input.cargoPickupButton.get()) {
                    pickingUp = true;
                }
                if (input.safePositionButton.get() || input.cargoDeliverButton.get() || input.lowDeliverButton.get() || input.midDeliverButton.get() || input.highDeliverButton.get()) {
                    pickingUp = false;
                }

                if (spitting) {
                    if (input.robotMode == true) {
                        ejectorSolenoid.set(DoubleSolenoid.Value.kForward);
                        velcroSolenoid.set(DoubleSolenoid.Value.kReverse);
                    }
                    if (input.robotMode == false) {
                        m_topIntakeController.set(ControlMode.PercentOutput, -1.0);
                        m_bottomIntakeController.set(ControlMode.PercentOutput, 1.0);
                    }
                    //System.out.println("\n\n");
                } else {
                    if (input.robotMode == true) {
                        velcroSolenoid.set(DoubleSolenoid.Value.kForward);
                        m_topIntakeController.set(ControlMode.PercentOutput, 0.0);
                        m_bottomIntakeController.set(ControlMode.PercentOutput, 0.0);
                    } else {
                        velcroSolenoid.set(DoubleSolenoid.Value.kReverse);
                        if (pickingUp) {
                            m_topIntakeController.set(ControlMode.PercentOutput, 1.0);
                            m_bottomIntakeController.set(ControlMode.PercentOutput, -1.0);
                        } else {
                            m_topIntakeController.set(ControlMode.PercentOutput, 0.4);
                            m_bottomIntakeController.set(ControlMode.PercentOutput, -0.4);
                        }
                    }
                    ejectorSolenoid.set(DoubleSolenoid.Value.kReverse);
                }
            //}

            /*if (input.intakeButton.get()) {
                m_topIntakeController.set(ControlMode.PercentOutput, 1.0);
                m_bottomIntakeController.set(ControlMode.PercentOutput, -1.0);
            } else if (input.intakeButton.get()) {
                m_topIntakeController.set(ControlMode.PercentOutput, -1.0);
                m_bottomIntakeController.set(ControlMode.PercentOutput, 1.0);
            } else {
                m_topIntakeController.set(ControlMode.PercentOutput, 0.0);
                m_bottomIntakeController.set(ControlMode.PercentOutput, 0.0);
            }*/

            /*if (velcroOutButton.get()) {
                velcroState = true;                
            } else if (velcroInButton.get()) {
                velcroState = false;
            }

            if (velcroState == true) {
                velcroSolenoid.set(DoubleSolenoid.Value.kForward);
            } else {
                velcroSolenoid.set(DoubleSolenoid.Value.kReverse);
            }

            if (ejectButton.get()) {
                ejectorSolenoid.set(DoubleSolenoid.Value.kForward);
            } else {
                ejectorSolenoid.set(DoubleSolenoid.Value.kReverse);
            }*/
        }
    };
}