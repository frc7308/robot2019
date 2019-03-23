package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

// Drives the robot for a specified amount of time using tank drive. t depends on the specified time.
public class ScootBack extends Command {
    private double speed;
    private double time;

    public ScootBack(double speed, double time) {
        this.speed = speed;
        this.time = time;
    }

    protected void initialize() {
        setTimeout(this.time);
        Robot.drivetrain.AutoDrive(this.speed, 0.0);
    }

    protected void execute() {
    }

    protected boolean isFinished() {
        return isTimedOut();
    }

    protected void end() {
        Robot.drivetrain.AutoDrive(0.0, 0.0);
    }

    protected void interrupted() {
        Robot.drivetrain.AutoDrive(0.0, 0.0);
    }
}