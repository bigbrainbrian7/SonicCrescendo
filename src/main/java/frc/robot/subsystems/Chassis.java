// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {

  public AHRS navx;
  public SwerveDriveKinematics swerveDriveKinematics; 
  public SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  public final double kMaxSpeedMetersPerSecond = 4.3; 
  public final double kMaxAngularVelocity = 2.35;

  private SwerveModule[] swerveModules = new SwerveModule[4];

  /** Creates a new Chassis. */
  public Chassis(AHRS navx, SwerveDriveKinematics swerveDriveKinematics, SwerveDrivePoseEstimator swerveDrivePoseEstimator) {
    this.navx = navx;
    this.swerveDriveKinematics = swerveDriveKinematics; 
    this.swerveDrivePoseEstimator = swerveDrivePoseEstimator;

    swerveModules[0] = new SwerveModule(1, 5, new SimpleMotorFeedforward(0.080663, 1.9711, 0.36785));
    swerveModules[1] = new SwerveModule(2, 6, new SimpleMotorFeedforward(0.12682, 1.971, 0.30547));
    swerveModules[2] = new SwerveModule(3, 7, new SimpleMotorFeedforward(0.11553, 1.956, 0.33358));
    swerveModules[3] = new SwerveModule(4, 8, new SimpleMotorFeedforward(0.11616, 1.9571, 0.321));

    // swerveModules[0].setDriveMotorInverted(false);
    // swerveModules[1].setDriveMotorInverted(false);
    // swerveModules[2].setDriveMotorInverted(false);  
    // swerveModules[3].setDriveMotorInverted(false);  

    navx.reset();
    resetOdometry(new Pose2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("chassis/navx", navx.getRotation2d().getRadians());

    for(int i=0; i<4; i++){
      SmartDashboard.putNumber("chassis/swervemodule"+i+"/drivePosition", swerveModules[i].getSwerveModulePosition().distanceMeters);
      SmartDashboard.putNumber("chassis/swervemodule"+i+"/driveVel", swerveModules[i].getSwerveModuleState().speedMetersPerSecond);
      SmartDashboard.putNumber("chassis/swervemodule"+i+"/turnPosition", swerveModules[i].getSwerveModulePosition().angle.getDegrees());
      SmartDashboard.putNumber("chassis/swervemodule"+i+"/desiredState", swerveModules[i].desiredState.angle.getDegrees());
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
}
