// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Clamp;
import frc.robot.subsystems.Shooter;

public class SetShooterStateTrapezoidal extends Command {
  private final double kDt = 0.02;
  private final TrapezoidProfile profile = 
    new TrapezoidProfile(new TrapezoidProfile.Constraints(20.0, 2));
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  Shooter shooter;

  /** Creates a new SetShooterStateTrapezoidal. */
  public SetShooterStateTrapezoidal(double angleDegrees, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    Clamp.clamp(angleDegrees, shooter.kReverseSoftLimit, shooter.kForwardSoftLimit);
    goal = new TrapezoidProfile.State(angleDegrees, 0);

    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setpoint = profile.calculate(kDt, setpoint, goal);
    shooter.setShooterState(setpoint);
    SmartDashboard.putNumber("trapezoid/speed", setpoint.velocity);
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
