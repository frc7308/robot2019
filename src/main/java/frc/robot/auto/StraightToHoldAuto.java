package frc.robot.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

import frc.robot.auto.commands.EjectHatch;
import frc.robot.auto.commands.SetBoomPosition;
import frc.robot.auto.commands.Sleep;
import frc.robot.auto.commands.ZeroSubsystems;
import frc.robot.auto.commands.DriveForwardToHold;
import frc.robot.auto.commands.ScootBack;

// Auto Routine: Straight To Hold
public class StraightToHoldAuto extends CommandGroup {
    public StraightToHoldAuto() {
        System.out.println("=====\nAUTO: Straight To Hold Auto\n=====");
        addParallel(new ZeroSubsystems());
        addParallel(new DriveForwardToHold(0.5, 2.0));
        addSequential(new SetBoomPosition(1024));
        addParallel(new EjectHatch());
        addParallel(new ScootBack(0.6, 0.5));
    }
}