// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import javax.swing.text.html.Option;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LUT;

public class ShooterVision extends SubsystemBase {
  private final SwerveDrivePoseEstimator poseEstimator;
  private final PhotonCamera camera;
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  //Forward Camera
  public final Translation2d robotToCam = new Translation2d(Units.inchesToMeters(-11.5), Units.inchesToMeters(-9.5));
  private final Transform3d odoRobotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(-11.5), Units.inchesToMeters(-9.5), 0.22), new Rotation3d(0,Units.degreesToRadians(30),0));

  // Construct PhotonPoseEstimator
  private final PhotonPoseEstimator photonPoseEstimator;

  private double lastAngle = 10;

  public static LUT normalLut = new LUT(new double[][]{
    {0.8, 62.2, 0},
    {1, 57.8, 0},
    {1.2, 53.8, 0},
    {1.4, 50.3, 0},
    {1.6, 47.0, 0},
    {1.8, 45.0, 0},
    {2, 43.8, 0},//Range good
    {2.25, 42.0, 0}, //Range good
    {2.5, 39.5, 0},
    {2.75, 37.0, 0}, // Range good
    {3, 36.0, 0}, //Range good, limelight timed out
    {3.25, 34.0, 0},//good
    {3.5, 32.7, 0},
    {3.75, 32.0, 0},
    {4, 31.2, 0},
    {5, 30.0, 0}
  }
  );

  /** Creates a new ShooterVision. */
  public ShooterVision(SwerveDrivePoseEstimator poseEstimator) {
    this.poseEstimator = poseEstimator;

    camera = new PhotonCamera("Shooter_Vision_Cam");
    camera.setPipelineIndex(0);
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, odoRobotToCam);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Optional<EstimatedRobotPose> estimatedRobotPose = getEstimatedGlobalPose(null)
    // SmartDashboard.putNumber("shooterVision/distanceMeters", getDistanceToSpeakerTag().orElse(9999999999.0));
    // SmartDashboard.putNumber("shooterVision/targetPitch", getTargetPitch().orElse(9999999999.0));
    // SmartDashboard.putNumber("shooterVision/desiredAngle", getTargetAngle().orElse(0.0));
    // Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
    // if(estimatedPose.isPresent()){
    //   poseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), Timer.getFPGATimestamp());
    // }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
      photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      return photonPoseEstimator.update();
  }
  
  public Optional<Double> getDistanceToSpeakerTag(){
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    PhotonPipelineResult results = camera.getLatestResult();
    Optional<Double> returnValue = Optional.empty();
    if(results.hasTargets()){
      for (PhotonTrackedTarget target : results.getTargets()){
        if((target.getFiducialId()==7&&alliance==Alliance.Blue)||(target.getFiducialId()==4&&alliance==Alliance.Red)){
          double targetDistance = PhotonUtils.calculateDistanceToTargetMeters(0.22, 1.45, Units.degreesToRadians(30), Units.degreesToRadians(target.getPitch()));
          returnValue = Optional.of(targetDistance);
        }
      }
    }

    return returnValue;
  }

  public Optional<Rotation2d> getYawToSpeakerTag(){
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    PhotonPipelineResult results = camera.getLatestResult();
    Optional<Rotation2d> returnValue = Optional.empty();
    if(results.hasTargets()){
      for (PhotonTrackedTarget target : results.getTargets()){
        if((target.getFiducialId()==7&&alliance==Alliance.Blue)||(target.getFiducialId()==4&&alliance==Alliance.Red)){
          returnValue = Optional.of(new Rotation2d(Units.degreesToRadians(-target.getYaw())));
        }
      }
    }

    return returnValue;
  }

  public Optional<Double> getTargetPitch(){  
  PhotonPipelineResult results = camera.getLatestResult();
    Optional<Double> returnValue = Optional.empty();
    if(results.hasTargets()){
      for (PhotonTrackedTarget target : results.getTargets()){
        if(target.getFiducialId()==7){
          returnValue = Optional.of(target.getPitch());
        }
      }
    }

    return returnValue;
  }

  public Optional<Double> getTargetAngle(){
    Optional<Double> distance = getDistanceToSpeakerTag();
    if(distance.isPresent()){
      lastAngle = normalLut.get(distance.get())[0];
      return Optional.of(normalLut.get(distance.get())[0]);
    }

    return Optional.of(lastAngle);
  }
  
}
