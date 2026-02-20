package frc.robot;

import static frc.robot.Constants.ON_RED_ALLIANCE;

import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;;
import edu.wpi.first.math.geometry.Rotation2d;

import org.photonvision.*;
import frc.robot.subsystems.*;

public class PoseEstimation {
    private final Field2d field2d = new Field2d();
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0)); // Tune me
    PhotonCamera camera = new PhotonCamera("photonvision");
    PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
    DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(field2d);

    // void GetVision() {
    // List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    // if (!results.isEmpty()) {
    // var result = results.get(results.size() - 1);
    // if (result.hasTargets()) {
    // // At least one AprilTag was seen by the camera
    // for (var target : result.getTargets()) {
    // if (target.getFiducialId() == 7) {
    // // Found Tag 7, record its information
    // targetYaw = target.getYaw();
    // targetVisible = true;
    // }
    // }
    // }
    // }

    public Pose3d GetPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
        }
        return visionEst.get().estimatedPose;
        // visionEst.ifPresent(
        //         est -> {
        //             // Change our trust in the measurement based on the tags we can see
        //             var estStdDevs = getEstimationStdDevs();

        //             estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        //         });
    }

        /**
     * Returns a Rotation2d to the team hub
     * 
     * @return
     */
    public Rotation2d getRotationToHub() {
        Translation2d hubPosMeters;
        if(ON_RED_ALLIANCE.getAsBoolean()) { //FIXME: Replace placeholders with actual hub positions
            hubPosMeters = new Translation2d(0, 0); // Red hub position
        } else { // Blue alliance
            hubPosMeters = new Translation2d(0, 0); // Blue hub position
        }
        Translation2d robotPos = this.GetPose().toPose2d().getTranslation();
        return drivetrainSubsystem.getPoseEstimation().getTranslation().minus(hubPosMeters).getAngle();
    }

    public double getDistanceToHubCenterMeters() {
        Translation2d hubPosMeters;
        if(ON_RED_ALLIANCE.getAsBoolean()) { //FIXME: Replace placeholders with actual hub positions
            hubPosMeters = new Translation2d(16.4592, 8.2296); // Red hub position
        } else {
            hubPosMeters = new Translation2d(0, 0); // Blue hub position
        }
        return drivetrainSubsystem.getPoseEstimation().getTranslation().getDistance(hubPosMeters);
    }
}