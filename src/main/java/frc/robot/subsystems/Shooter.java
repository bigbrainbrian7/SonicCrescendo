// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
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
import frc.robot.Clamp;

public class Shooter extends SubsystemBase {
  public CANSparkFlex motor = new CANSparkFlex(13, MotorType.kBrushless);
  private SparkPIDController pidController = motor.getPIDController();
  public double targetSetpoint;

  private final ArmFeedforward feedforward = new ArmFeedforward(0.66, 0.33, 0.106, 0.0); //In units of radians I believe

  private final double kGearing = (1/20.0) * (18.0/36.0);
  private final double kConversionFactor = 30.4/(21.856-0.254);

  private final double angleFromHorizontal = 2.0;

  public final double kForwardSoftLimit = 100;
  public final double kReverseSoftLimit = 5;

  public boolean isHomed = false; 

  private final SysIdRoutine sysIdRoutine =
  new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(Units.Volts.per(Units.Seconds).of(0.1), Units.Volts.of(1.25), null),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            setVoltage(volts);
          },
          log -> {
            log.motor("motor")
                .voltage(getVoltage())
                .angularPosition(getAngle())
                .angularVelocity(getAngularVelocity());
          },
          this));

  /** Creates a new Shooter. */
  public Shooter() {
    //Configuration
    motor.clearFaults();
    motor.restoreFactoryDefaults();

    //Safety
    motor.setSmartCurrentLimit(40);
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) kForwardSoftLimit);
    motor.setSoftLimit(SoftLimitDirection.kReverse, (float) kReverseSoftLimit);
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    motor.setIdleMode(IdleMode.kCoast);

    //Position/Velocity
    motor.setInverted(true);
    pidController.setFeedbackDevice(motor.getEncoder());
    motor.getEncoder().setPositionConversionFactor(kConversionFactor);
    motor.getEncoder().setVelocityConversionFactor(kConversionFactor/60.0);


    //PID
    pidController.setP(30.0/360.0);
    pidController.setOutputRange(-1.0, 1.0);
    motor.burnFlash();

    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 193);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 23);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 26);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 2671);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2333);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 17);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 23);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter/angle", getAngle().in(Units.Degrees));
    SmartDashboard.putNumber("shooter/omega", getAngularVelocity().in(Units.DegreesPerSecond));
    SmartDashboard.putNumber("shooter/voltage", getVoltage().in(Units.Volts));
  }

  public void homeShooter(){
    motor.getEncoder().setPosition(angleFromHorizontal);
    // motor.getEncoder().setPosition(0.0);
    isHomed = true;
  }

  public Measure<Angle> getAngle(){
    return Units.Degrees.of(motor.getEncoder().getPosition());
  }

  public Measure<Velocity<Angle>> getAngularVelocity(){
    return Units.DegreesPerSecond.of(motor.getEncoder().getVelocity());
  }

  public Measure<Voltage> getVoltage(){
    return Units.Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
  }

  public void setVoltage(Measure<Voltage> voltage){
    motor.setVoltage(voltage.in(Units.Volts));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public Command sysIdRoutineCommand(){
    return new InstantCommand()
                .andThen(this.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(14))
                .andThen(new WaitCommand(5))
                .andThen(this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(8))
                .andThen(new WaitCommand(5))
                .andThen(this.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(5))
                .andThen(new WaitCommand(5))
                .andThen(this.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(5))
                .andThen(new WaitCommand(5));
  }

  public void setShooterState(TrapezoidProfile.State state){
    double arbFF = feedforward.calculate(state.position, state.velocity);

    pidController.setReference(state.position, CANSparkBase.ControlType.kPosition, 0, arbFF, ArbFFUnits.kVoltage);

    this.targetSetpoint = state.position;
  }

  public void setPosition(Measure<Angle> angle){
    double targetDegree = Clamp.clamp(angle.in(Units.Degrees), kReverseSoftLimit, kForwardSoftLimit);
    pidController.setReference(targetDegree, ControlType.kPosition, 0, 0.33*Math.cos(getAngle().in(Units.Degrees)), ArbFFUnits.kVoltage);
  }
}
