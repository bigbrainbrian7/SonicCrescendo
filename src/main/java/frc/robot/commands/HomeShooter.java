// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class HomeShooter extends Command {
  Shooter shooter;
  /** Creates a new HomeShooter. */
  public HomeShooter(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter=shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.unhome();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVoltage(Units.Volts.of(-0.3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setVoltage(Units.Volts.of(0));
    if(interrupted){
      return;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.isAtHomePosition();
  }
}
