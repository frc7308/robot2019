package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

// Maps driver station inputs to functions to increase readability and consistency in code.
public class RobotInput {
    private static RobotInput m_instance = null;

    public static RobotInput getInstance() {
        if (m_instance == null) {
            m_instance = new RobotInput();
        }
        return m_instance;
    }

    public Joystick wheel;
    public Joystick throttleStick;
    public Joystick buttonBoard;
    public Joystick switchController;

    public JoystickButton quickTurnButton;
    public JoystickButton autoAlignButton;
    public JoystickButton spitbutton;

    public JoystickButton highDeliverButton;
    public JoystickButton midDeliverButton;
    public JoystickButton lowDeliverButton;
    public JoystickButton safePositionButton;
    public JoystickButton cargoDeliverButton;
    public JoystickButton cargoPickupButton;
    public JoystickButton extraButton;
    public JoystickButton hatchModeButton;
    public JoystickButton cargoModeButton;
    public JoystickButton frontModeButton;
    public JoystickButton backModeButton;

    public boolean elevatorState;
    public boolean robotMode;
    public boolean robotDirection;

    public RobotInput() {
        //this.wheel = new Joystick(0);
        //this.throttleStick = new Joystick(1);
        this.switchController = new Joystick(0);
        this.buttonBoard = new Joystick(1);

        this.switchController.setYChannel(5);
        this.switchController.setXChannel(0);
        this.switchController.setZChannel(2);
        this.switchController.setThrottleChannel(3);
    
        //this.quickTurnButton = new JoystickButton(switchController, 7);
        this.autoAlignButton = new JoystickButton(switchController, 6);
        //this.spitbutton = new JoystickButton(switchController, 6);

        this.highDeliverButton = new JoystickButton(this.buttonBoard, 1);
        this.midDeliverButton = new JoystickButton(this.buttonBoard, 2);
        this.lowDeliverButton = new JoystickButton(this.buttonBoard, 3);
        this.safePositionButton = new JoystickButton(this.buttonBoard, 4);
        this.cargoDeliverButton = new JoystickButton(this.buttonBoard, 5);
        this.cargoPickupButton = new JoystickButton(this.buttonBoard, 7);
        this.extraButton = new JoystickButton(this.buttonBoard, 6);
        this.hatchModeButton = new JoystickButton(this.buttonBoard, 8);
        this.cargoModeButton = new JoystickButton(this.buttonBoard, 9);
        this.frontModeButton = new JoystickButton(this.buttonBoard, 10);
        this.backModeButton = new JoystickButton(this.buttonBoard, 11);
    
        this.robotMode = true;
        this.robotDirection = true;
    }
}