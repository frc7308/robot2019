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
        addSequential(new ZeroSubsystems());
        addParallel(new DriveForwardToHold(0.5, 2.0));
        addParallel(new SetBoomPosition(900));
        addSequential(new Sleep(2.5));
        addParallel(new EjectHatch());
        addParallel(new ScootBack(0.5, 0.5));
    }
}