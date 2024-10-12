// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SetShooterStateTrapezoidal;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public AHRS navx = new AHRS(Port.kMXP, (byte) 200);

  public SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
    new Translation2d(Units.Meters.convertFrom(18.5/2.0, Units.Inches),Units.Meters.convertFrom(23/2.0, Units.Inches)),
    new Translation2d(Units.Meters.convertFrom(18.5/2.0, Units.Inches),-Units.Meters.convertFrom(23/2.0, Units.Inches)),
    new Translation2d(-Units.Meters.convertFrom(18.5/2.0, Units.Inches),Units.Meters.convertFrom(23/2.0, Units.Inches)),
    new Translation2d(-Units.Meters.convertFrom(18.5/2.0, Units.Inches),-Units.Meters.convertFrom(23/2.0, Units.Inches))
  );

  public SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(swerveDriveKinematics, navx.getRotation2d(), 
    new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    }, 
    new Pose2d(0, 0 , new Rotation2d())
  );

  public final Intake intake = new Intake();
  public final Flywheel flywheel = new Flywheel();
  public final Shooter shooter = new Shooter();
  public final Chassis chassis = new Chassis(navx, swerveDriveKinematics, swerveDrivePoseEstimator);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController driverController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    chassis.setDefaultCommand(chassis.getDriveCommand(()->-1*driverController.getRawAxis(1), ()->-1*driverController.getRawAxis(0), ()->-1*driverController.getRawAxis(4)));
    shooter.setDefaultCommand(new SequentialCommandGroup(
        new WaitCommand(1.0),
        new RunCommand(()->shooter.setPosition(Units.Degrees.of(5)), shooter).withTimeout(0.5),
        new RunCommand(()->{
        shooter.setVoltage(Units.Volts.of(0));
      }, 
      shooter)
      )
    );

    flywheel.setDefaultCommand(new SequentialCommandGroup(
       new WaitCommand(0.5),
        new RunCommand(()->{
        flywheel.setTopMotorVoltage(Units.Volts.of(0));
        flywheel.setBotMotorVoltage(Units.Volts.of(0));
      }, 
      flywheel))
    );

    intake.setDefaultCommand(new SequentialCommandGroup(
        new RunCommand(()->intake.setVelocity(Units.InchesPerSecond.of(-5)), intake).withTimeout(0.5),
        new RunCommand(()->intake.setVelocity(Units.InchesPerSecond.of(0)),intake)
      )
    );

    //intake
    new Trigger(driverController.button(5))
      .whileTrue(new RunCommand(()->intake.setVelocity(Units.InchesPerSecond.of(30)),intake)
      );

    //   //spin up flywheels
    // new Trigger(driverController.button(6))
    //   .whileTrue(new RunCommand(()->{
    //     flywheel.setTopMotorVoltage(Units.Volts.of(-10));
    //     flywheel.setBotMotorVoltage(Units.Volts.of(-10));
    //   }, 
    //   flywheel));

    //   //actually shoot
    // new Trigger(driverController.button(3))
    //   .whileTrue(new RunCommand(()->intake.setVelocity(Units.InchesPerSecond.of(60)),intake)
    //   );

    new Trigger(driverController.button(1))
      .whileTrue(new RunCommand(()->{
        flywheel.setVelocity(Units.InchesPerSecond.of(-driverController.getRawAxis(1)*700));
      }, 
      flywheel)
    );

    // new Trigger(driverController.button(4))
    //   .whileTrue(new SetShooterStateTrapezoidal(45, shooter));

    new Trigger(driverController.button(4))
      .whileTrue(new ParallelCommandGroup(
        new RunCommand(()->shooter.setPosition(Units.Degrees.of(98)), shooter),
        new RunCommand(()->flywheel.setVelocity(Units.InchesPerSecond.of(-150)), flywheel)
      )
    );

      //shoot
      new Trigger(driverController.button(6))
        .whileTrue(new ParallelCommandGroup(
          new RunCommand(()->shooter.setPosition(Units.Degrees.of(30)), shooter),
          new RunCommand(()->flywheel.setVelocity(Units.InchesPerSecond.of(-950)), flywheel)
        )
      );

    new Trigger(()->driverController.getLeftTriggerAxis()>0.25)
    .whileTrue(
      new RunCommand(()->intake.setVelocity(Units.InchesPerSecond.of(30)), flywheel)
    );
    // new Trigger(driverController.button(4))
    // .whileTrue(
    //   new RunCommand(()->shooter.setVoltage(Units.Volts.of(-5*driverController.getRawAxis(1))),shooter)
    // );
    // new Trigger(driverController.button(2))
    //   .whileTrue(new RunCommand(()->{
    //     shooter.setVoltage(Units.Volts.of(4*-1*driverController.getRawAxis(1)));
    //   }, 
    //   shooter)
    // );

    // new Trigger(driverController.button(3))
    //   .whileTrue(
    //     new SetShooterStateTrapezoidal(Units.Degrees.of(-driverController.getRawAxis(0)*9+10), shooter)
    //   );

    new Trigger(driverController.button(8))
      .onTrue(new InstantCommand(()->navx.reset()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    // return flywheel.sysIdRoutineCommand();
    return shooter.sysIdRoutineCommand();
  }
}
