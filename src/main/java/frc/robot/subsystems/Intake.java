// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Intake extends SubsystemBase {
  //Intake is the front of the robot
  public CANSparkFlex topMotor = new CANSparkFlex(14, MotorType.kBrushless);
  public CANSparkMax botMotor1 = new CANSparkMax(9, MotorType.kBrushless);
  public CANSparkMax botMotor2 = new CANSparkMax(10, MotorType.kBrushless);

  private final DigitalInput beamBreak;

  private final double kbotMotorGearing = (1/5.0);
  private final double kbotMotorConversionFactor = kbotMotorGearing*0.0282*Math.PI;

  private final double ktopMotorGearing = (1/4);
  private final double ktopMotorConversionFactor = ktopMotorGearing*0.0282*Math.PI;


  SimpleMotorFeedforward botMotor1Feedforward = new SimpleMotorFeedforward(0.35016, 3.6774, 0.57346);//p:3.596 //RETUNE
  SimpleMotorFeedforward botMotor2Feedforward = new SimpleMotorFeedforward(0.28074, 3.619, 0.17632);//p:3.596 //RETUNE
  SimpleMotorFeedforward topMotorFeedforward = new SimpleMotorFeedforward(0.14092, 13.75, 0.42175);//p:1.7946 //RETUNE

  private final SysIdRoutine sysIdRoutine =
  new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(Units.Volts.per(Units.Seconds).of(0.5), Units.Volts.of(5), null),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            setbotMotor1Voltage(volts);
            setbotMotor2Voltage(volts);
            settopMotorVoltage(volts);
          },
          log -> {
            log.motor("botMotor1")
                .voltage(getbotMotor1Voltage())
                .linearPosition(getbotMotor1Position())
                .linearVelocity(getbotMotor1Velocity());
            log.motor("botMotor2")
              .voltage(getbotMotor2Voltage())
              .linearPosition(getbotMotor2Position())
              .linearVelocity(getbotMotor2Velocity());
            log.motor("topMotor")
              .voltage(gettopMotorVoltage())
              .linearPosition(gettopMotorPosition())
              .linearVelocity(gettopMotorVelocity());
          },
          this));

  /** Creates a new Intake. */
  public Intake() {
    beamBreak = new DigitalInput(0);

    for(CANSparkMax motor : new CANSparkMax[]{botMotor1,botMotor2} ){
      motor.restoreFactoryDefaults();
      motor.clearFaults();

      //SAFETY
      motor.setSmartCurrentLimit(22);
      motor.setIdleMode(IdleMode.kBrake);
      botMotor1.setInverted(false);
      botMotor2.setInverted(true);

      //POSITION + VELOCITY
      motor.getEncoder().setPositionConversionFactor(kbotMotorConversionFactor);
      motor.getEncoder().setVelocityConversionFactor(kbotMotorConversionFactor/60.0);
      motor.getEncoder().setMeasurementPeriod(8);
      motor.getEncoder().setAverageDepth(2);
      motor.getEncoder().setPosition(0);

      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 151);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 19);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 23);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 26);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 3889);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 3023);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 4111);

     // botMotor2.follow(botMotor1);

      motor.burnFlash();
    }

    topMotor.restoreFactoryDefaults();
    topMotor.clearFaults();

    //SAFETY
    topMotor.setSmartCurrentLimit(60);
    topMotor.setIdleMode(IdleMode.kBrake);
    topMotor.setInverted(false);

    //POSITION + VELOCITY
    topMotor.getEncoder().setPositionConversionFactor(ktopMotorConversionFactor);
    topMotor.getEncoder().setVelocityConversionFactor(ktopMotorConversionFactor/60.0);
    topMotor.getEncoder().setMeasurementPeriod(8);
    topMotor.getEncoder().setAverageDepth(2);
    topMotor.getEncoder().setPosition(0);

    topMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 149);
    topMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 26);
    topMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 24);
    topMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 29);
    topMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1129);
    topMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 2347);
    topMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 3163);

    topMotor.burnFlash();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("intake/botMotor1Pos", getbotMotor1Position().in(Units.Inches));
    // SmartDashboard.putNumber("intake/botMotor2Pos", getbotMotor2Position().in(Units.Inches));
    // SmartDashboard.putNumber("intake/topMotorPos", gettopMotorPosition().in(Units.Inches));

    // SmartDashboard.putNumber("intake/botMotor1Vel", getbotMotor1Velocity().in(Units.InchesPerSecond));
    // SmartDashboard.putNumber("intake/botMotor2Vel", getbotMotor2Velocity().in(Units.InchesPerSecond));
    // SmartDashboard.putNumber("intake/topMotorVel", gettopMotorVelocity().in(Units.InchesPerSecond));

    // SmartDashboard.putBoolean("beamBreak/blocked", getBeamBreakIsBlocked());

  }

  private Measure<Distance> getbotMotor1Position(){
    return Units.Meters.of(botMotor1.getEncoder().getPosition());
  }

  private Measure<Distance> getbotMotor2Position(){
    return Units.Meters.of(botMotor2.getEncoder().getPosition());
  }

  private Measure<Distance> gettopMotorPosition(){
    return Units.Meters.of(topMotor.getEncoder().getPosition());
  }

  private Measure<Velocity<Distance>> getbotMotor1Velocity(){
    return Units.MetersPerSecond.of(botMotor1.getEncoder().getVelocity());
  }

  private Measure<Velocity<Distance>> getbotMotor2Velocity(){
    return Units.MetersPerSecond.of(botMotor2.getEncoder().getVelocity());
  }

  private Measure<Velocity<Distance>> gettopMotorVelocity(){
    return Units.MetersPerSecond.of(topMotor.getEncoder().getVelocity());
  }

  public Measure<Voltage> getbotMotor1Voltage(){
    return Units.Volts.of(botMotor1.getAppliedOutput() * botMotor1.getBusVoltage());
  }

  public Measure<Voltage> getbotMotor2Voltage(){
    return Units.Volts.of(botMotor2.getAppliedOutput() * botMotor2.getBusVoltage());
  }

  public Measure<Voltage> gettopMotorVoltage(){
    return Units.Volts.of(topMotor.getAppliedOutput() * topMotor.getBusVoltage());
  }

  public void setbotMotor1Voltage(Measure<Voltage> volts){
    botMotor1.setVoltage(volts.in(Units.Volts));
  };

  public void setbotMotor2Voltage(Measure<Voltage> volts){
    botMotor2.setVoltage(volts.in(Units.Volts));
  };

  public void settopMotorVoltage(Measure<Voltage> volts){
    topMotor.setVoltage(volts.in(Units.Volts));
  };

  public void setVelocity(Measure<Velocity<Distance>> velocity){
    setbotMotor1Voltage(Units.Volts.of(botMotor1Feedforward.calculate(velocity.in(Units.MetersPerSecond))));
    setbotMotor2Voltage(Units.Volts.of(botMotor2Feedforward.calculate(velocity.in(Units.MetersPerSecond))));
    settopMotorVoltage(Units.Volts.of(topMotorFeedforward.calculate(velocity.in(Units.MetersPerSecond))));
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
                .andThen(new WaitCommand(2))
                .andThen(this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(10))
                .andThen(new WaitCommand(2))
                .andThen(this.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(5))
                .andThen(new WaitCommand(2))
                .andThen(this.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(5))
                .andThen(new WaitCommand(2));
  }

  public boolean getBeamBreakIsBlocked(){
    return !beamBreak.get();
  }
}
