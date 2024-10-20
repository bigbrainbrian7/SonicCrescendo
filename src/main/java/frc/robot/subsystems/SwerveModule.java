// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

/** Add your docs here. */
public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    public final RelativeEncoder driveEncoder;
    public final AbsoluteEncoder turnEncoder;

    private final SparkPIDController drivePIDController;
    private final SparkPIDController turnPIDController;

    private final double kDriveGearing = (14.0/22.0)*(15.0/45.0);
    private final double kWheelDiameter = 0.0762;

    private final SimpleMotorFeedforward driveMotorFeedForward;

    public SwerveModuleState desiredState = new SwerveModuleState();

    public SwerveModule(int driveCANId, int turnCANId, SimpleMotorFeedforward driveMotorFeedForward){
        driveMotor = new CANSparkMax(driveCANId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnCANId, MotorType.kBrushless);
        this.driveMotorFeedForward = driveMotorFeedForward;

        //SETUP
        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();
        
        driveMotor.clearFaults();
        turnMotor.clearFaults();

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getAbsoluteEncoder();

        drivePIDController = driveMotor.getPIDController();
        drivePIDController.setFeedbackDevice(driveEncoder);
        driveMotor.setInverted(false);

        turnPIDController = turnMotor.getPIDController();
        turnPIDController.setFeedbackDevice(turnEncoder);
        turnMotor.setInverted(false);
        turnEncoder.setInverted(true); //Module reverses spin direction of azimuth mot

        //SAFETY
        driveMotor.setSmartCurrentLimit(50);
        turnMotor.setSmartCurrentLimit(20);
        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setOpenLoopRampRate(0.25); 

        //Position + Velocity
        double positionConversionFactor = kDriveGearing * kWheelDiameter*Math.PI;
        driveEncoder.setPositionConversionFactor(positionConversionFactor);
        driveEncoder.setVelocityConversionFactor(positionConversionFactor/60.0);
        driveEncoder.setMeasurementPeriod(8);
        driveEncoder.setAverageDepth(2);
        driveEncoder.setPosition(0);

        turnEncoder.setPositionConversionFactor(2*Math.PI);
        turnEncoder.setVelocityConversionFactor(2*Math.PI/60.0); //hopefully not important

        //PID
        turnPIDController.setPositionPIDWrappingEnabled(true);
        turnPIDController.setPositionPIDWrappingMinInput(0);
        turnPIDController.setPositionPIDWrappingMaxInput(2*Math.PI);

        turnPIDController.setP(1.0);

        drivePIDController.setP(0.06);

        //PERIODIC FRAME STATUSES
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 127);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 11);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 9);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 4421);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 5099);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 6143);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 5449);

        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 89);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 9);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 6551);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 7451);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 12);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 4177);
    }

    public Measure<Distance> getDriveMotorPosition(){
        return Units.Meters.of(driveEncoder.getPosition());
    }

    public Measure<Velocity<Distance>> getDriveMotorVelocity(){
        return Units.MetersPerSecond.of(driveEncoder.getVelocity());
    }

    public Measure<Angle> getTurnMotorPosition(){
        return Units.Radians.of(turnEncoder.getPosition());
    }

    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getTurnMotorPosition().in(Units.Radians)));
    }

    public SwerveModuleState getSwerveModuleState(){
        return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getTurnMotorPosition().in(Units.Radians)));
    }

    public Measure<Voltage> getDriveVoltage(){
        return Units.Volts.of(driveMotor.getAppliedOutput() * driveMotor.getBusVoltage());
    }

    //SUPPLY CURRENT IN AMPS
    public double getDriveCurrent(){
        return driveMotor.getOutputCurrent();
    }

    public void resetDriveEncoder(){
        driveEncoder.setPosition(0.0);
    }

    public void setDriveVoltage(Measure<Voltage> voltage){
        driveMotor.setVoltage(voltage.in(Units.Volts));
    }

    public void setVoltageDrive(Measure<Voltage> voltage){
        setDriveVoltage(voltage);
    
        turnPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    }

    public SwerveModuleState getDesiredSate(){
        return desiredState;
    }

    public void setDesiredState(SwerveModuleState desiredState){
        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState,
            new Rotation2d(turnEncoder.getPosition()));

        var arbFF = 0.0;
        arbFF = driveMotorFeedForward.calculate(optimizedDesiredState.speedMetersPerSecond);
        drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, 0, arbFF, ArbFFUnits.kVoltage);

        turnPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        this.desiredState = desiredState;
    }

    public void setDriveMotorInverted(boolean isInverted){
        driveMotor.setInverted(isInverted);
    }
}
