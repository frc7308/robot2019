package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import frc.robot.Robot;

public class EjectHatch extends Command {
    private double setpoint;

    public EjectHatch() {
    }

    protected void initialize() {
        Robot.intake.ejectorSolenoid.set(DoubleSolenoid.Value.kForward);
        Robot.intake.velcroSolenoid.set(DoubleSolenoid.Value.kReverse);
        setTimeout(0.5);
    }

    protected void execute() {
    }

    protected boolean isFinished() {
        return isTimedOut();
    }

    protected void end() {
        Robot.intake.ejectorSolenoid.set(DoubleSolenoid.Value.kReverse);
        Robot.intake.velcroSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    protected void interrupted() {
        Robot.intake.ejectorSolenoid.set(DoubleSolenoid.Value.kReverse);
        Robot.intake.velcroSolenoid.set(DoubleSolenoid.Value.kForward);
    }
}