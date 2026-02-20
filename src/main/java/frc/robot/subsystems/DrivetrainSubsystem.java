package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
//import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.FalconSwerveModule;
import frc.robot.util.SwerveModule;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.robot.Constants.*;

import java.time.Period;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import javax.naming.PartialResultException;

import org.photonvision.PhotonCamera;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXUpdateRate;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import frc.robot.RobotContainer;

import frc.robot.PoseEstimation;

public class DrivetrainSubsystem implements Subsystem {
    PoseEstimation poseEstimationClass = new PoseEstimation();
    Counter counter = new Counter(Counter.Mode.kPulseLength);
    public static final NetworkTable drivetrainNT = NetworkTableInstance.getDefault().getTable("drivetrain");

    public final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT);

    // Telemetry
    private final StructArrayPublisher<SwerveModuleState> measuredSwerveStatesPublisher = drivetrainNT
            .getStructArrayTopic(
                    "MeasuredSwerveStates",
                    SwerveModuleState.struct)
            .publish();

    private final StructArrayPublisher<SwerveModuleState> desiredSwerveStatesPublisher = drivetrainNT
            .getStructArrayTopic(
                    "DesiredSwerveStates",
                    SwerveModuleState.struct)
            .publish();

    private final StructPublisher<Rotation2d> robotRotationPublisher = drivetrainNT.getStructTopic(
            "RobotRotation",
            Rotation2d.struct).publish();

    private final StructPublisher<Pose2d> robotPosePublisher = drivetrainNT.getStructTopic("RobotPose", Pose2d.struct)
            .publish();

    /*
     * Robot Coordinate System
     * See
     * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/
     * coordinate-systems.html#robot-coordinate-system
     * Forward is x+, Left is y+, counterclockwise is theta+
     */

    // private final SysIdRoutine driveSysIdRoutine = new SysIdRoutine(
    // new SysIdRoutine.Config(),
    // new SysIdRoutine.Mechanism(
    // (voltage) -> runDriveVolts(voltage.in()),
    // null,
    // this
    // )
    // );

    // private final SysIdRoutine steerSysIdRoutine = new SysIdRoutine(
    // new SysIdRoutine.Config(),
    // new SysIdRoutine.Mechanism(
    // (voltage) -> runSteerVolts(voltage.in()),
    // null,
    // this
    // )
    // );

    private final Translation2d frontLeftPosition = new Translation2d(DRIVETRAIN_WIDTH / 2D, DRIVETRAIN_LENGTH / 2D); // All
                                                                                                                      // translations
                                                                                                                      // are
                                                                                                                      // relative
                                                                                                                      // to
                                                                                                                      // center
                                                                                                                      // of
                                                                                                                      // rotation
    private final Translation2d frontRightPosition = new Translation2d(DRIVETRAIN_WIDTH / 2D, -DRIVETRAIN_LENGTH / 2D);
    private final Translation2d backLeftPosition = new Translation2d(-DRIVETRAIN_WIDTH / 2D, DRIVETRAIN_LENGTH / 2D);
    private final Translation2d backRightPosition = new Translation2d(-DRIVETRAIN_WIDTH / 2D, -DRIVETRAIN_LENGTH / 2D);

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftPosition, frontRightPosition,
            backLeftPosition, backRightPosition);
    // private final SwerveDrivePoseEstimator poseEstimator;
    public final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;

    private final HolonomicDriveController driveController = new HolonomicDriveController(
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            new ProfiledPIDController(1, 0, 0,
                    new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCEL)));

    private final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k200Hz);

    private boolean slowMode = false;
    private double rotationOffsetRadians = 0.0;

    public DrivetrainSubsystem(Field2d field) {
        this.field = field;

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        // AutoBuilder.configure(
        // this::getPose,
        // this::resetPose,
        // this::getChassisSpeeds,
        // this::robotRelativeDrive,
        // HOLONOMIC_PATH_FOLLOWER_CONFIG,
        // null,
        // ON_RED_ALLIANCE,
        // this
        // );

        frontLeft = new FalconSwerveModule(
                FRONT_LEFT_DRIVE_MOTOR_ID,
                FRONT_LEFT_STEER_MOTOR_ID,
                FRONT_LEFT_STEER_ENCODER_ID,
                FRONT_LEFT_STEER_OFFSET,
                drivetrainNT.getSubTable("frontleft"));

        frontRight = new FalconSwerveModule(
                FRONT_RIGHT_DRIVE_MOTOR_ID,
                FRONT_RIGHT_STEER_MOTOR_ID,
                FRONT_RIGHT_STEER_ENCODER_ID,
                FRONT_RIGHT_STEER_OFFSET,
                drivetrainNT.getSubTable("frontright"));

        backLeft = new FalconSwerveModule(
                BACK_LEFT_DRIVE_MOTOR_ID,
                BACK_LEFT_STEER_MOTOR_ID,
                BACK_LEFT_STEER_ENCODER_ID,
                BACK_LEFT_STEER_OFFSET,
                drivetrainNT.getSubTable("backleft"));

        backRight = new FalconSwerveModule(
                BACK_RIGHT_DRIVE_MOTOR_ID,
                BACK_RIGHT_STEER_MOTOR_ID,
                BACK_RIGHT_STEER_ENCODER_ID,
                BACK_RIGHT_STEER_OFFSET,
                drivetrainNT.getSubTable("backright"));

        tab.addNumber("Rotation", () -> (getAdjustedRotation().getDegrees()));

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                new Pose2d());
    }

    @Override
    public void periodic() {
        // does not need to use adjusted rotation, odometry handles it.
        poseEstimator.update(navX.getRotation2d(), getSwervePositions());
        field.setRobotPose(getPose());

        visionPosePeriodic();

        // detectAprilTag(driverController);
        // System.out.println(calculateAlignDistance(false));
        updateTelemetry();

        // if(limelightX() == 0){
        // System.out.println("AAAAAA");
        // }

    }

    private void updateTelemetry() {
        // Swerve
        desiredSwerveStatesPublisher.set(new SwerveModuleState[] {
                frontLeft.getDesiredState(),
                frontRight.getDesiredState(),
                backLeft.getDesiredState(),
                backRight.getDesiredState()
        });

        measuredSwerveStatesPublisher.set(new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        });

        robotRotationPublisher.set(getAdjustedRotation());

        frontLeft.updateTelemetry();
        frontRight.updateTelemetry();
        backLeft.updateTelemetry();
        backRight.updateTelemetry();

        robotPosePublisher.set(getPose());
    }
    public void robotRelativeDrive(ChassisSpeeds chassisSpeeds, DriveFeedforwards driveFeedforwards) {
        drive(chassisSpeeds, false);
    }

    public void drive(double xSpeed, double ySpeed, double angularVelocity, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, angularVelocity);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                chassisSpeeds, getAdjustedRotation())
                        : chassisSpeeds);

        // var moduleStates = kinematics.toSwerveModuleStates(
        // ChassisSpeeds.discretize(
        // fieldRelative
        // ? ChassisSpeeds.fromFieldRelativeSpeeds(
        // xSpeed, ySpeed, angularVelocity, getAdjustedRotation())
        // : new ChassisSpeeds(xSpeed, ySpeed, angularVelocity), periodSeconds));

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_MODULE_VELOCITY);
        setModuleStates(moduleStates);
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, fieldRelative);
    }

    public void stopMotion() {
        drive(0, 0, 0, false);
    }

    public void timedDriveCommand(double xSpeed, double ySpeed, double angularVelocity, boolean fieldRelative,
            double driveTime) {
        new SequentialCommandGroup(
                Commands.run(() -> drive(xSpeed, ySpeed, angularVelocity, fieldRelative), this).withTimeout(driveTime),
                Commands.runOnce(() -> stopMotion())).schedule();
    }

    public void zeroGyro() {
        rotationOffsetRadians = -navX.getRotation2d().getRadians() + Math.PI;
        resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
    }

    // Getters

    public SwerveModulePosition[] getSwervePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public AHRS getNavx() {
        return navX;
    }

    public HolonomicDriveController getDriveController() {
        return driveController;
    }

    public Rotation2d getAdjustedRotation() {
        return navX.getRotation2d().plus(Rotation2d.fromRadians(rotationOffsetRadians));
    }

    // Setters
    public void setModuleStates(SwerveModuleState[] moduleStates) {
        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }

    public void resetPose(Pose2d pose) {
        // does not need to be adjusted rotation, odometry handles this
        poseEstimator.resetPosition(getAdjustedRotation(), getSwervePositions(), pose);
    }

    public Pose2d getPoseEstimation() {
        return(poseEstimator.getEstimatedPosition());
    }
    public void setGyro(Rotation2d rotation) {
        rotationOffsetRadians = -navX.getRotation2d().getRadians() + rotation.getRadians();
    }

    // teleop drive

    public Command enableSlowModeCommand() {
        return Commands.runOnce(() -> {
            slowMode = true;
        });
    }

    public Command disableSlowModeCommand() {
        return Commands.runOnce(() -> {
            slowMode = false;
        });
    }

    public Command joystickDriveCommand(DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotAxis) {
        return Commands.run(() -> {

            double xVelocity = MathUtil.applyDeadband(xAxis.getAsDouble(), 0.1) * MAX_MODULE_VELOCITY
                    * (slowMode ? LINEAR_SLOW_MODE_MODIFIER : 1);
            double yVelocity = MathUtil.applyDeadband(yAxis.getAsDouble(), 0.1) * MAX_MODULE_VELOCITY
                    * (slowMode ? LINEAR_SLOW_MODE_MODIFIER : 1);
            double rotVelocity = MathUtil.applyDeadband(rotAxis.getAsDouble(), 0.1) * MAX_ANGULAR_VELOCITY
                    * (slowMode ? ANGULAR_SLOW_MODE_MODIFIER : 1);
            drive(xVelocity, yVelocity, rotVelocity, FIELD_RELATIVE_DRIVE);

        }, this);
    }

    // sysID

    // public Command driveSysIdRoutineCommand(){
    // return Commands.sequence(
    // driveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(7),
    // Commands.waitSeconds(2),
    // driveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(7),
    // Commands.waitSeconds(2),
    // driveSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(2),
    // Commands.waitSeconds(2),
    // driveSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(2),
    // Commands.waitSeconds(2)
    // );
    // }

    // public Command steerSysIdRoutineCommand() {
    // return Commands.sequence(
    // steerSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(7),
    // Commands.waitSeconds(2),
    // steerSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(7),
    // Commands.waitSeconds(2),
    // steerSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(2),
    // Commands.waitSeconds(2),
    // steerSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(2),
    // Commands.waitSeconds(2)
    // );
    // }

    public void runDriveVolts(double voltage) {
        frontLeft.runForward(voltage);
        frontRight.runForward(voltage);
        backLeft.runForward(voltage);
        backRight.runForward(voltage);
    }

    public void runSteerVolts(double voltage) {
        frontLeft.runRotation(voltage);
        frontRight.runRotation(voltage);
        backLeft.runRotation(voltage);
        backRight.runRotation(voltage);
    }

    // vision

    public Command PrintPoseEstimationData() {
        return Commands.runOnce(() -> {
            Pose3d poseEst = poseEstimationClass.GetPose();
            System.out.print("Pos X: ");
            System.out.print(poseEst.getX());
            System.out.print("Pos y: ");
            System.out.print(poseEst.getY());
            /*
             * System.out.print("Rotation");
             * System.out.println(poseEst.getRotation());
             */
            /*
             * System.out.print("Pos X: ");
             * System.out.print(poseEstimator.getEstimatedPosition().getX());
             * System.out.print(", Pos Y: ");
             * System.out.print(poseEstimator.getEstimatedPosition().getY());
             * System.out.print(", Rotation: ");
             * System.out.println(poseEstimator.getEstimatedPosition().getRotation());
             */

        }, this);
    }
}