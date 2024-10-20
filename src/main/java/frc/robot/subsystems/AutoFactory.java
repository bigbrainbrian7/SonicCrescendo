// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class AutoFactory {
    RobotContainer robotContainer;

    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public AutoFactory(RobotContainer robotContainer){
        this.robotContainer = robotContainer;
        initPathPlanner();
        initAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void initPathPlanner(){
        Supplier<ChassisSpeeds> getSpeed = ()->{
            return robotContainer.swerveDriveKinematics.toChassisSpeeds(robotContainer.chassis.getModuleStates());
        };

        Consumer<ChassisSpeeds> setSpeed = (chassisSpeeds)->{
            robotContainer.chassis.setChassisSpeeds(chassisSpeeds, false);
        };

        HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(2.5*0), 
            new PIDConstants(0.5), 
            robotContainer.chassis.kMaxSpeedMetersPerSecond, 
            Math.hypot(Units.Meters.convertFrom(18.0/2.0, Units.Inches),Units.Meters.convertFrom(22.5/2.0, Units.Inches)),
            new ReplanningConfig(true, false));

        BooleanSupplier shouldFlipPath = () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followe
            // d to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        };

        AutoBuilder.configureHolonomic(
            robotContainer.chassis::getPose,
            robotContainer.chassis::resetOdometry,
            getSpeed, 
            setSpeed, 
            holonomicPathFollowerConfig, 
            shouldFlipPath, 
            robotContainer.chassis
        );
    }

    private void initAutoChooser(){
        BooleanSupplier isBlue = () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Blue;
            }
            return false;
        };

        autoChooser.setDefaultOption("Please Select Auto", new InstantCommand());

        // autoChooser.addOption("TestChoreo", new SequentialCommandGroup(
        //     new InstantCommand(()->robotContainer.chassis.setFieldCentricOffset(0, isBlue)),
        //     new InstantCommand(()->robotContainer.chassis.resetOdometryAllianceManaged(new Pose2d(0.3,5.65, new Rotation2d()), isBlue)),
        //     followChoreoPath("Test")
        // ));

        // autoChooser.addOption("fannnyyy", new SequentialCommandGroup(
        //     new InstantCommand(()->robotContainer.chassis.setFieldCentricOffset(0, isBlue)),
        //     new InstantCommand(()->robotContainer.chassis.resetOdometryAllianceManaged(new Pose2d(1.47,7.4, new Rotation2d()), isBlue)),
        //     new ParallelCommandGroup(
        //         followChoreoPath("Rush"),
        //         new RunCommand(()->robotContainer.intake.setVelocity(Units.InchesPerSecond.of(60)),robotContainer.intake),
        //         new RunCommand(()->robotContainer.flywheel.setVelocity(Units.InchesPerSecond.of(-200)), robotContainer.flywheel)
        //     )
        // ));

        autoChooser.addOption("fannnyyy2", new SequentialCommandGroup(
            new InstantCommand(()->robotContainer.chassis.setFieldCentricOffset(0, isBlue)),
            new InstantCommand(()->robotContainer.chassis.resetOdometryAllianceManaged(new Pose2d(1.47,3.4, new Rotation2d()), isBlue)),
            new ParallelCommandGroup(
                followChoreoPath("SourceRush"),
                new RunCommand(()->robotContainer.intake.setVelocity(Units.InchesPerSecond.of(60)),robotContainer.intake),
                new RunCommand(()->robotContainer.flywheel.setVelocity(Units.InchesPerSecond.of(-200)), robotContainer.flywheel)
            )
        ));
    }

    private Command followChoreoPath(String pathName){
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
        return AutoBuilder.followPath(path);
    }

    public Command getAuto(){
        return autoChooser.getSelected();
    }
}
