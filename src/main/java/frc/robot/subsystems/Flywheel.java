// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Flywheel extends SubsystemBase {
  public CANSparkMax topMotor = new CANSparkMax(11, MotorType.kBrushless);
  public CANSparkMax botMotor = new CANSparkMax(12, MotorType.kBrushless);

  SimpleMotorFeedforward botMotorFeedforward = new SimpleMotorFeedforward(0.27677, 0.013468326, 0.0014590776);
  SimpleMotorFeedforward topMotorFeedforward = new SimpleMotorFeedforward(0.30421, 0.012778232, 0.0011931396);

  private final double kGearing = (36.0/22.0);
  private final double kConversionFactor = kGearing * 2.0 * Math.PI;
  public double targetSetpoint;

  private final SysIdRoutine sysIdRoutine =
  new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(Units.Volts.per(Units.Seconds).of(1.0), Units.Volts.of(9), null),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            setTopMotorVoltage(volts);
            setBotMotorVoltage(volts);
          },
          log -> {
            log.motor("topMotor")
                .voltage(getTopMotorVoltage())
                .linearPosition(getTopMotorPosition())
                .linearVelocity(getTopMotorVelocity());
            log.motor("botMotor")
              .voltage(getBotMotorVoltage())
              .linearPosition(getBotMotorPosition())
              .linearVelocity(getTopMotorVelocity());
          },
          this));


  /** Creates a new Flywheel. */
  public Flywheel() {
    topMotor.setInverted(true);
    botMotor.setInverted(true);
    for(CANSparkMax motor : new CANSparkMax[]{topMotor,botMotor} ){
      motor.restoreFactoryDefaults();
      motor.clearFaults();

      //SAFETY
      motor.setSmartCurrentLimit(60);
      motor.setIdleMode(IdleMode.kCoast);

      //POSITION + VELOCITY
      motor.getEncoder().setPositionConversionFactor(kConversionFactor);
      motor.getEncoder().setVelocityConversionFactor(kConversionFactor/60.0);
      motor.getEncoder().setMeasurementPeriod(8);
      motor.getEncoder().setAverageDepth(2);
      motor.getEncoder().setPosition(0);

      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 20);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

      motor.burnFlash();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("flywheel/topMotorPos", getTopMotorPosition().in(Units.Inches));
    SmartDashboard.putNumber("flywheel/botMotorPos", getBotMotorPosition().in(Units.Inches));
    SmartDashboard.putNumber("flywheel/topMotorVel", getTopMotorVelocity().in(Units.InchesPerSecond));
    SmartDashboard.putNumber("flywheel/botMotorVel", getBotMotorVelocity().in(Units.InchesPerSecond));
  }

  private Measure<Distance> getTopMotorPosition(){
    return Units.Inches.of(topMotor.getEncoder().getPosition());
  }

  private Measure<Distance> getBotMotorPosition(){
    return Units.Inches.of(botMotor.getEncoder().getPosition());
  }

  private Measure<Velocity<Distance>> getTopMotorVelocity(){
    return Units.InchesPerSecond.of(topMotor.getEncoder().getVelocity());
  }

  private Measure<Velocity<Distance>> getBotMotorVelocity(){
    return Units.InchesPerSecond.of(botMotor.getEncoder().getVelocity());
  }

  public Measure<Voltage> getTopMotorVoltage(){
    return Units.Volts.of(topMotor.getAppliedOutput() * topMotor.getBusVoltage());
  }

  public Measure<Voltage> getBotMotorVoltage(){
    return Units.Volts.of(botMotor.getAppliedOutput() * botMotor.getBusVoltage());
  }

  public void setTopMotorVoltage(Measure<Voltage> volts){
    topMotor.setVoltage(volts.in(Units.Volts));
  };

  public void setBotMotorVoltage(Measure<Voltage> volts){
    botMotor.setVoltage(volts.in(Units.Volts));
  };

  public void setVelocity(Measure<Velocity<Distance>> velocity){
    setTopMotorVoltage(Units.Volts.of(topMotorFeedforward.calculate(velocity.in(Units.InchesPerSecond))));
    setBotMotorVoltage(Units.Volts.of(botMotorFeedforward.calculate(velocity.in(Units.InchesPerSecond))));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public Command sysIdRoutineCommand(){
    return new InstantCommand()
                .andThen(this.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(10))
                .andThen(new WaitCommand(5))
                .andThen(this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(10))
                .andThen(new WaitCommand(5))
                .andThen(this.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(5))
                .andThen(new WaitCommand(5))
                .andThen(this.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(5))
                .andThen(new WaitCommand(5));
  }
}
