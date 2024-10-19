// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class AutoFactory {
    RobotContainer robotContainer;

    public AutoFactory(RobotContainer robotContainer){
        this.robotContainer = new RobotContainer();
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
            new PIDConstants(5*0.5*0), 
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
}
