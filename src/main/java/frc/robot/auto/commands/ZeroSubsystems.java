package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.command.Command;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Robot;

// Drives the robot for a specified amount of time using tank drive. t depends on the specified time.
public class ZeroSubsystems extends Command {

    public ZeroSubsystems() {
    }

    protected void initialize() {
        Robot.boom.zero();
        //Robot.elevator.m_innerStageController.set(ControlMode.PercentOutput, -0.35);
        //Robot.elevator.m_middleStageController.set(ControlMode.PercentOutput, 0.35);
        setTimeout(0.4);
    }

    protected void execute() {
    }

    protected boolean isFinished() {
        return isTimedOut();
    }

    protected void end() {
        //Robot.elevator.m_innerStageController.set(ControlMode.PercentOutput, 0.0);
        //Robot.elevator.m_middleStageController.set(ControlMode.PercentOutput, 0.0);
        Robot.elevator.zero();
    }

    protected void interrupted() {
        //Robot.elevator.m_innerStageController.set(ControlMode.PercentOutput, 0.0);
        //Robot.elevator.m_middleStageController.set(ControlMode.PercentOutput, 0.0);
    }
}