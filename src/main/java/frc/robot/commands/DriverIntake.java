// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeVision;

public class DriverIntake extends Command {
  private final Chassis chassis;
  private final Intake intake;
  private final IntakeVision intakeVision;

  DoubleSupplier vx;
  DoubleSupplier vy;
  DoubleSupplier omega;

  private double latestNoteAngle;

  private boolean beamBreakBeenPassed = false;

  private double startTime = Double.MAX_VALUE;


  /** Creates a new DriverIntake. */
  public DriverIntake(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega, Chassis chassis, Intake intake, IntakeVision intakeVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vx = vx;
    this.vy = vy;
    this.omega = omega;

    this.chassis = chassis;
    this.intake = intake;
    this.intakeVision = intakeVision;

    addRequirements(chassis, intake, intakeVision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<Rotation2d> rotationToNote = intakeVision.getRotationToNote();
    if (rotationToNote.isPresent()){
      double error = rotationToNote.get().plus(chassis.getRotation2d()).getRadians();
      chassis.driveToBearing(vx.getAsDouble()*chassis.kMaxSpeedMetersPerSecond, vy.getAsDouble()*chassis.kMaxSpeedMetersPerSecond, error);
    }
    else{
      chassis.setChassisSpeeds(new ChassisSpeeds(vx.getAsDouble()*chassis.kMaxSpeedMetersPerSecond, vy.getAsDouble()*chassis.kMaxSpeedMetersPerSecond, omega.getAsDouble()*chassis.kMaxAngularVelocity), true);
    }

    intake.setVelocity(Units.InchesPerSecond.of(45));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getBeamBreakIsBlocked();
  }
}
