// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.Optional;
import java.util.OptionalInt;

import javax.swing.text.html.Option;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeVision extends SubsystemBase {
  /** Creates a new IntakeVision. */

  private final Translation3d cameraLocation = new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)); 

  private final PhotonCamera intakeCamera;

  public IntakeVision() {

    intakeCamera = new PhotonCamera("IntakeCamera");
    intakeCamera.setPipelineIndex(0);

  }  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically
    SmartDashboard.putNumber("intakeVision/rotationToTarget", getRotationToNote().orElse(new Rotation2d(12345)).getDegrees());
    SmartDashboard.putBoolean("intakeVision/seesTarget", seesTarget());


  }

  public Optional<Rotation2d> getRotationToNote(){
    PhotonPipelineResult results = intakeCamera.getLatestResult();

    if (results.hasTargets()){
      PhotonTrackedTarget target = results.getBestTarget();
      return Optional.of(new Rotation2d(Units.degreesToRadians(-target.getYaw())));
    }

    return Optional.empty();
  }

  public boolean seesTarget(){
    PhotonPipelineResult results = intakeCamera.getLatestResult();

    if(results.hasTargets()){
      return true;
    }
    return false;
  }
}