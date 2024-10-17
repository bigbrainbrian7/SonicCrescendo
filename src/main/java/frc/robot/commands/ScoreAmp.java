// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ScoreAmp extends Command {
  private final Shooter shooter;
  private final Flywheel flywheel;
  private final Intake intake;
  private DoubleSupplier feed;

  private int intakePassed = 0;

  private boolean lastBlockedState;

  /** Creates a new ScoreAmp. */
  public ScoreAmp(Shooter shooter, Flywheel flywheel, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feed = feed;

    this.shooter = shooter;
    this.flywheel = flywheel;
    this.intake = intake;

    addRequirements(shooter, flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastBlockedState = intake.getBeamBreakIsBlocked();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //  if(intake.getBeamBreakIsBlocked() != lastBlockedState){
   //   if(intake.getBeamBreakIsBlocked()==false){
   //     intakePassed++;
   //     lastBlockedState = false;
   //   }
   //   else{
   //     lastBlockedState = true;
   //   }
   // }
   if(intake.getBeamBreakIsBlocked()==false){
        intakePassed=1;
        lastBlockedState = false;
       }
       else{
        intakePassed=0;
        lastBlockedState = true;
       }

    if(intakePassed < 1){
      shooter.setPosition(Units.Degrees.of(85));
    }
    else{
      if(shooter.getAngle().in(Units.Degrees)<shooter.kForwardSoftLimit-2){
        shooter.setPosition(Units.Degrees.of(100));
      }
    }

    flywheel.setVelocity(Units.InchesPerSecond.of(-175));
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
