package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.JointSubsystem;

public class PreloadCommand extends DynamicCommand {

    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final JointSubsystem jointSubsystem;
    private final DrivetrainSubsystem drivetrainSubsystem;

    public PreloadCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, JointSubsystem jointSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
        
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.jointSubsystem = jointSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(intakeSubsystem, shooterSubsystem, jointSubsystem, drivetrainSubsystem);
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
            shooterSubsystem.rampUpShooter(),
            new WaitCommand(3.0),
            jointSubsystem.shootingPositionCommand(),
            new WaitCommand(0.2),
            Commands.runOnce(() -> intakeSubsystem.setIntakeDutyCycle(0.5)),
            Commands.waitSeconds(1.2),
            Commands.runOnce(() -> intakeSubsystem.holdIntake()),
            new WaitCommand(0.1),
            jointSubsystem.stowPositionCommand(),
            jointSubsystem.zeroJointCommand(),
            intakeSubsystem.intake(),
            Commands.runOnce(() -> drivetrainSubsystem.zeroGyro())
        );
        // } else {
        //     return null;
        // }
    }
}