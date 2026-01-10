package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.JointSubsystem;

public class StopIntakeCommand extends DynamicCommand {

    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final JointSubsystem jointSubsystem;

    public StopIntakeCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, JointSubsystem jointSubsystem) {
        
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.jointSubsystem = jointSubsystem;

        addRequirements(intakeSubsystem, shooterSubsystem, jointSubsystem);
    }

    @Override
    protected Command getCommand() {
        // if(!jointSubsystem.getZeroed()) {
        //     return Commands.sequence(
        //         jointSubsystem.zeroJointCommand()
        //     );
        // }
        // if(intakeSubsystem.getPieceLoaded()) {
        return new SequentialCommandGroup(
            new WaitCommand(0.2),
            Commands.runOnce(() -> intakeSubsystem.holdIntake())
        );
        // } else {
        //     return null;
        // }
    }
}