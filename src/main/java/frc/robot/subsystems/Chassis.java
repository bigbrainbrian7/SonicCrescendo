// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Chassis extends SubsystemBase {

  public AHRS navx;
  public SwerveDriveKinematics swerveDriveKinematics; 
  public SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  public final double kMaxSpeedMetersPerSecond = 4.67; 
  public final double kMaxAngularVelocity = 11.47;

  private final PIDController headingPID = new PIDController(24.0/Math.PI, 0, 0);

  private SwerveModule[] swerveModules = new SwerveModule[4];

  Field2d field = new Field2d();

  private final SysIdRoutine sysIdRoutine =
  new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(Units.Volts.per(Units.Seconds).of(0.1), Units.Volts.of(1.25), null),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            swerveModules[0].setVoltageDrive(volts);
            swerveModules[1].setVoltageDrive(volts);
            swerveModules[2].setVoltageDrive(volts);
            swerveModules[3].setVoltageDrive(volts);
          },
          log -> {
            log.motor("frontRight")
                .voltage(swerveModules[1].getDriveVoltage())
                .linearPosition(swerveModules[1].getDriveMotorPosition())
                .linearVelocity(swerveModules[1].getDriveMotorVelocity());
            log.motor("frontLeft")
              .voltage(swerveModules[0].getDriveVoltage())
              .linearPosition(swerveModules[0].getDriveMotorPosition())
              .linearVelocity(swerveModules[0].getDriveMotorVelocity());
            log.motor("rearRight")
              .voltage(swerveModules[3].getDriveVoltage())
              .linearPosition(swerveModules[3].getDriveMotorPosition())
              .linearVelocity(swerveModules[3].getDriveMotorVelocity());
            log.motor("rearLeft")
              .voltage(swerveModules[2].getDriveVoltage())
              .linearPosition(swerveModules[2].getDriveMotorPosition())
              .linearVelocity(swerveModules[2].getDriveMotorVelocity());
          },
          this));

  /** Creates a new Chassis. */
  public Chassis(AHRS navx, SwerveDriveKinematics swerveDriveKinematics, SwerveDrivePoseEstimator swerveDrivePoseEstimator) {
    this.navx = navx;
    this.swerveDriveKinematics = swerveDriveKinematics; 
    this.swerveDrivePoseEstimator = swerveDrivePoseEstimator;

    swerveModules[0] = new SwerveModule(1, 5, new SimpleMotorFeedforward(0.04, 2.6, 0.36785));
    swerveModules[1] = new SwerveModule(2, 6, new SimpleMotorFeedforward(0.04, 2.6, 0.30547));
    swerveModules[2] = new SwerveModule(3, 7, new SimpleMotorFeedforward(0.04, 2.6, 0.33358));
    swerveModules[3] = new SwerveModule(4, 8, new SimpleMotorFeedforward(0.04, 2.6, 0.321));

    // swerveModules[0].setDriveMotorInverted(false);
    // swerveModules[1].setDriveMotorInverted(false);
    // swerveModules[2].setDriveMotorInverted(false);  
    // swerveModules[3].setDriveMotorInverted(false);  

    navx.reset();
    resetOdometry(new Pose2d(8, 4, new Rotation2d()));

    headingPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    SmartDashboard.putNumber("chassis/navx", navx.getRotation2d().getRadians());
    field.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
    SmartDashboard.putData("field", field);    
    SmartDashboard.putNumber("chassis/xPos", swerveDrivePoseEstimator.getEstimatedPosition().getX());
    for(int i=0; i<4; i++){
      // SmartDashboard.putNumber("chassis/swervemodule"+i+"/drivePosition", swerveModules[i].getSwerveModulePosition().distanceMeters);
      // SmartDashboard.putNumber("chassis/swervemodule"+i+"/driveVel", swerveModules[i].getSwerveModuleState().speedMetersPerSecond);
      // SmartDashboard.putNumber("chassis/swervemodule"+i+"/turnPosition", swerveModules[i].getSwerveModulePosition().angle.getDegrees());
      // SmartDashboard.putNumber("chassis/swervemodule"+i+"/desiredState", swerveModules[i].desiredState.angle.getDegrees());
    }
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean fieldRelative){
    if(fieldRelative){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, navx.getRotation2d());
    }

    SwerveModuleState[] states = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    setModuleStates(states);
  }

  public Command getDriveCommand(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega){
    return new RunCommand(()->setChassisSpeeds(new ChassisSpeeds(vx.getAsDouble()*kMaxSpeedMetersPerSecond, vy.getAsDouble()*kMaxSpeedMetersPerSecond, omega.getAsDouble()*kMaxAngularVelocity), true), this);
  };

  public void driveToBearing(double vx, double vy, double targetRadians){
    double currentRadians = navx.getRotation2d().getRadians(); 
    double correctedRadians = MathUtil.angleModulus(targetRadians);
    double output = headingPID.calculate(currentRadians, correctedRadians);
    // SmartDashboard.putNumber("drivePid/output", output);
    // SmartDashboard.putNumber("drivePid/error", correctedRadians-currentRadians);
    setChassisSpeeds(new ChassisSpeeds(vx*kMaxSpeedMetersPerSecond, vy*kMaxSpeedMetersPerSecond, output), true);
  };

  public void resetOdometry(Pose2d pose) {

    // var rot = new Rotation2d(MathUtil.angleModulus(navx.getRotation2d().getRadians()));
    var rot = navx.getRotation2d(); 

    swerveDrivePoseEstimator.resetPosition(
        navx.getRotation2d(),
        new SwerveModulePosition[] {
            swerveModules[0].getSwerveModulePosition(),
            swerveModules[1].getSwerveModulePosition(),
            swerveModules[2].getSwerveModulePosition(),
            swerveModules[3].getSwerveModulePosition()
        },
        pose);
  }

  /**
   * Sets the swerve ModuleStates. Meters Per Second
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, kMaxSpeedMetersPerSecond);
    swerveModules[0].setDesiredState(desiredStates[0]);
    swerveModules[1].setDesiredState(desiredStates[1]);
    swerveModules[2].setDesiredState(desiredStates[2]);
    swerveModules[3].setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[] {
      swerveModules[0].getSwerveModuleState(),
      swerveModules[1].getSwerveModuleState(),
      swerveModules[2].getSwerveModuleState(),
      swerveModules[3].getSwerveModuleState()
    };
  }

  public void updateOdometry(){
    swerveDrivePoseEstimator.updateWithTime(
        Timer.getFPGATimestamp(),
        navx.getRotation2d(),
        new SwerveModulePosition[] {
            swerveModules[0].getSwerveModulePosition(),
            swerveModules[1].getSwerveModulePosition(),
            swerveModules[2].getSwerveModulePosition(),
            swerveModules[3].getSwerveModulePosition()
        });

    // System.out.println(Timer.getFPGATimestamp());
  }

  public void setFieldCentricOffset(double degrees, BooleanSupplier isBlue){
    navx.reset();
    navx.setAngleAdjustment(isBlue.getAsBoolean() ? degrees : -degrees);
  }

  public void resetOdometryAllianceManaged(Pose2d pose, BooleanSupplier isBlue){
    if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
      pose = new Pose2d(16.542-pose.getX(), pose.getY(), new Rotation2d(Math.PI-pose.getRotation().getRadians())); 
    }

    var rot = navx.getRotation2d(); 

    swerveDrivePoseEstimator.resetPosition(
        navx.getRotation2d(),
        new SwerveModulePosition[] {
            swerveModules[0].getSwerveModulePosition(),
            swerveModules[1].getSwerveModulePosition(),
            swerveModules[2].getSwerveModulePosition(),
            swerveModules[3].getSwerveModulePosition()
        },
        pose);
  }

  public Pose2d getPose(){
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation2d(){
    return navx.getRotation2d();
  }

  public Command sysIdRoutineCommand(){
    return new SequentialCommandGroup(
      sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(5),
      new WaitCommand(5),
      sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(5),
      new WaitCommand(5),
      sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(5),
      new WaitCommand(kMaxSpeedMetersPerSecond),
      sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(5),
      new WaitCommand(5)
    );
  }
}
