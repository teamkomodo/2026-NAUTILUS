// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.*;


public class RobotContainer {  
    private final Field2d field2d = new Field2d();

    //Inputs Devices
    public final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT); 
    
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(field2d);
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();


    public RobotContainer() {
        configureBindings();
        registerNamedCommands();
    } 
    
    private void configureBindings() {

        //drivetrain
		Trigger leftBumperDriver = driverController.leftBumper();
		leftBumperDriver.onTrue(Commands.runOnce(() -> {drivetrainSubsystem.zeroGyro();}));

		// deadbands are applied in command
		drivetrainSubsystem.setDefaultCommand(drivetrainSubsystem.joystickDriveCommand(
				() -> -driverController.getLeftY(), // -Y (up) on joystick is +X (forward) on robot
				() -> -driverController.getLeftX(), // -X (left) on joystick is +Y (left) on robot
				() -> driverController.getRightX() // -X (left) on joystick is +Theta (counter-clockwise) on robot
		));

        Trigger lefTrigger = driverController.leftTrigger();
        Trigger rightTrigger = driverController.rightTrigger();

        lefTrigger.whileTrue(intakeSubsystem.runIntakeCommand());
        rightTrigger.whileTrue(shooterSubsystem.runShooterCommand());
    }

    public void teleopInit() {
        Commands.runOnce(() -> {drivetrainSubsystem.zeroGyro();});
    }

    public Command getAutonomousCommand() {
        return null; //AutoBuilder.followPath(null);
    }

    private void registerNamedCommands() {

    }
}
