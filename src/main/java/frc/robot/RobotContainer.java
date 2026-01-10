// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.util.BlinkinPattern;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
//import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.*;

import java.lang.ModuleLayer.Controller;

import frc.robot.LimelightHelpers;


public class RobotContainer {  
    private final Field2d field2d = new Field2d();

    //Inputs Devices
    public final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT); 
    
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(field2d);
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();


    public RobotContainer() {
        configureBindings();
        registerNamedCommands();
        detectAprilTag(driverController);
    } 

    private Command xboxRumbleCommand(CommandXboxController controller, double time) {
            return Commands.runEnd(() -> {
                controller.setRumble(RumbleType.kLeftRumble, 1);
                controller.setRumble(RumbleType.kRightRumble, 1);
            }, () -> {
                controller.setRumble(RumbleType.kLeftRumble, 0);
                controller.setRumble(RumbleType.kRightRumble, 0);
            }).withTimeout(time);
    }


    private void detectAprilTag(CommandXboxController controller){
        boolean tv = LimelightHelpers.getTV("limelight");

        if(tv){
            controller.setRumble(RumbleType.kLeftRumble, 1);
            System.out.println("RUMBBLEEE");
        } else {
            controller.setRumble(RumbleType.kLeftRumble, 0);
        }
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

        Trigger driverLeftTrigger = driverController.leftTrigger();
        Trigger driverRightTrigger = driverController.rightTrigger();
        driverLeftTrigger.whileTrue(drivetrainSubsystem.goToBranch(false));
        driverRightTrigger.whileTrue(drivetrainSubsystem.goToBranch(true));

        

        

        Trigger driverYButton = driverController.y();
        driverYButton.whileTrue(xboxRumbleCommand(driverController, 10));

        Trigger driverXButton = driverController.x();
        driverXButton.whileTrue(drivetrainSubsystem.limelightAlignCommand());

        



    }

    public void teleopInit() {
        Commands.runOnce(() -> {drivetrainSubsystem.zeroGyro();});
        Commands.runOnce(() -> ledSubsystem.setFramePatternCommand(BlinkinPattern.COLOR_1_AND_2_PATTERN_SPARKLE_COLOR_2_ON_COLOR_1));
    }


    
    public Command getAutonomousCommand() {
        return null; //AutoBuilder.followPath(null);
    }

    private void registerNamedCommands() {

    }



    

    
}
