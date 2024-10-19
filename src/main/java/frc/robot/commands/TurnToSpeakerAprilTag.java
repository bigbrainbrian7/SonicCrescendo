// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ShooterVision;

public class TurnToSpeakerAprilTag extends Command {
  Chassis chassis;
  ShooterVision shooterVision;

  DoubleSupplier vx;
  DoubleSupplier vy;
  DoubleSupplier omega;
  /** Creates a new TurnToSpeakerAprilTag. */
  public TurnToSpeakerAprilTag(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega, Chassis chassis, ShooterVision shooterVision) {
    this.vx = vx;
    this.vy = vy;
    this.omega = omega;

    this.chassis = chassis;
    this.shooterVision = shooterVision;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<Double> distanceMeters = shooterVision.getDistanceToSpeakerTag();
    Optional<Rotation2d> yaw = shooterVision.getYawToSpeakerTag();
    if(distanceMeters.isPresent() && yaw.isPresent()){
      Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(distanceMeters.get(), yaw.get()).minus(shooterVision.robotToCam);
      Rotation2d offset = translation.getAngle();
      Rotation2d fieldRelativeAngle = offset.plus(chassis.getRotation2d());
      chassis.driveToBearing(vx.getAsDouble()*chassis.kMaxSpeedMetersPerSecond, vy.getAsDouble()*chassis.kMaxSpeedMetersPerSecond, fieldRelativeAngle.getRadians());
    }
    else{
      chassis.setChassisSpeeds(new ChassisSpeeds(vx.getAsDouble()*chassis.kMaxSpeedMetersPerSecond, vy.getAsDouble()*chassis.kMaxSpeedMetersPerSecond, omega.getAsDouble()*chassis.kMaxAngularVelocity), true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
