// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final boolean TUNING_MODE = false;

    // Controls
    public static final double XBOX_DEADBAND = 0.06;
    public static final int DRIVER_XBOX_PORT = 0;
    public static final int OPERATOR_XBOX_PORT = 1;

	public static final boolean FIELD_RELATIVE_DRIVE = true;
	public static final boolean ALIGNMENT_DRIVE = false;
	public static final double LINEAR_SLOW_MODE_MODIFIER = 0.5;
	public static final double ANGULAR_SLOW_MODE_MODIFIER = 0.2;
	
	public static final double DRIVETRAIN_WIDTH = 0.4864163246;
	public static final double DRIVETRAIN_LENGTH = 0.4079583662;
	
	public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 4;
	public static final int FRONT_LEFT_STEER_MOTOR_ID = 5;
	public static final int FRONT_LEFT_STEER_ENCODER_ID = 10;
	public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(293.1+180)+1.346;
	
	public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 6;
	public static final int FRONT_RIGHT_STEER_MOTOR_ID = 7;
	public static final int FRONT_RIGHT_STEER_ENCODER_ID = 11;
	public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(357.78)+1.422;
	
	public static final int BACK_LEFT_DRIVE_MOTOR_ID = 2;
	public static final int BACK_LEFT_STEER_MOTOR_ID = 3;
	public static final int BACK_LEFT_STEER_ENCODER_ID = 9;
	public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(327.3)+9.235;
	
	public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 0;
	public static final int BACK_RIGHT_STEER_MOTOR_ID = 1;
	public static final int BACK_RIGHT_STEER_ENCODER_ID = 8;
	public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(170.75)+9.259;
	
	public static final double WHEEL_DIAMETER = 0.1016;
	public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
	public static final double STEER_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);

	public static final double LIMELIGHT_X_OFFSET = 0;
	
	public static final double MAX_MODULE_VELOCITY = 4.058; // physical maximum attainable speed of swerve modules
	public static final double MAX_MODULE_ACCEL = 21; // physical maximum attainable accel of swerve modules
	
	public static final double MAX_ANGULAR_VELOCITY = 4.0 * Math.PI; // constraint for angular velocity
	public static final double MAX_ANGULAR_ACCEL = 4.0 * Math.PI; // constraint for angular acceleration
	
	public static final double FALCON_500_NOMINAL_VOLTAGE = 12.0;
	public static final double TALON_FX_TICKS_PER_ROTATION = 2048.0;

    public static final PIDConstants DRIVE_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants STEER_PID = new PIDConstants(5, 0, 0);

	public static PPHolonomicDriveController HOLONOMIC_PATH_FOLLOWER_CONFIG = 
    new PPHolonomicDriveController(
        DRIVE_PID, // Translation Constants
        STEER_PID, // Steering Constants
        Math.sqrt(DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH) / 2
	);

    public static final BooleanSupplier ON_RED_ALLIANCE = () -> {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return true;
        }
        return false;
    };

	public static final int LED_CHANNEL = 0;

    // FRC Field
    public static final double FIELD_WIDTH = 8.21; // approxiamation: Field Length is 26ft. 11 1/8 in wide
    public static final double FIELD_LENGTH = 16.54;
	public static final double APRILTAG_TO_BRANCH_X_DISTANCE = 0.1551; // horizontal distance from april tag to branch in meters
	
	// Vision
	public static final double LIMELIGHT_TO_APRILTAG_Y_DISTANCE = 0.6604;
	public static final double LIMELIGHT_ROBOT_X_OFFSET = 0;
	public static final double ROBOT_ALIGNMENT_SPEED = 1.0;
	public static final double ALIGN_LINEAR_SPEED_FACTOR = 1.1;
	public static final double ALIGN_EXPONENTIAL_SPEED_FACTOR = 0.9;

	public static final double APRILTAG_HEIGHT = 0.5715;
	public static final double LIMELIGHT_ROBOT_Y_OFFSET = -0.0254;
	public static final double LIMELIGHT_HEIGHT = 0.1778;
	public static final double LIMELIGHT_ANGLE_OFFSET = 28.3; // degrees
	public static final double TY_ALIGN_THRESHOLD = -3;
}