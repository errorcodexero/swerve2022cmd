// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.xero1425.XeroRobot;
import org.xero1425.subsystems.XeroOISubsystem;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.OISubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveBaseSubsystem;
import frc.robot.subsystems.TargetTrackerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private XeroRobot robot_ ;
    
    private IntakeSubsystem intake_ ;
    private AgitatorSubsystem agitator_ ;
    private ConveyorSubsystem conveyor_ ;
    private TurretSubsystem turret_ ;
    private ShooterSubsystem shooter_ ;
    private SwerveDriveBaseSubsystem swerve_ ;
    private LimeLightSubsystem limelight_ ;
    private TargetTrackerSubsystem tracker_ ;
    private OISubsystem oi_ ;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(XeroRobot robot) {
        try {
            intake_ = new IntakeSubsystem(robot) ;
        }
        catch(Exception ex) {
            intake_ = null ;
        }

        try {
            agitator_ = new AgitatorSubsystem(robot) ;
        }
        catch(Exception ex) {
            agitator_ = null ;
        }

        try {
            conveyor_ = new ConveyorSubsystem(robot) ;
        }
        catch(Exception ex) {
            conveyor_ = null ;
        }

        try {
            turret_ = new TurretSubsystem(robot) ;
        }
        catch(Exception ex) {
            turret_ = null ;
        }

        try {
            shooter_ = new ShooterSubsystem(robot) ;
        }
        catch(Exception ex) {
            shooter_ = null ;
        }

        try {
            swerve_ = new SwerveDriveBaseSubsystem(robot) ;
        }
        catch(Exception ex) {
            swerve_ = null ;
        }

        try {
            limelight_ = new LimeLightSubsystem(robot) ;
        }
        catch(Exception ex) {
            limelight_ = null ;
        }

        try {
            tracker_ = new TargetTrackerSubsystem(robot, limelight_, turret_) ;
        }
        catch(Exception ex) {
            tracker_ = null ;
        }

        try {
            oi_ = new OISubsystem(robot, XeroOISubsystem.DriverType.Swerve) ;
        }
        catch(Exception ex) {
            oi_ = null ;
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null ;
    }
}
