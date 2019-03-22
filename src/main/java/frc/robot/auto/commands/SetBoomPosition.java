package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;

public class SetBoomPosition extends Command {
    private double setpoint;

    public SetBoomPosition(double setpoint) {
        this.setpoint = setpoint;
    }

    protected void initialize() {
        Robot.boom.m_setpoint = setpoint;
    }

    protected void execute() {
    }

    protected boolean isFinished() {
        return Robot.boom.m_boomController.getClosedLoopError() < Robot.boom.k_acceptableRangeTicks;
    }

    protected void end() {
    }

    protected void interrupted() {
        Robot.boom.m_setpoint = 0;
    }
}