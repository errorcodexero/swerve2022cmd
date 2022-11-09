package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import org.xero1425.XeroRobot;
import org.xero1425.subsystems.XeroSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDriveBaseSubsystem extends XeroSubsystem {
    private final static String SubsystemName = "swerve";

    private double max_linear_velocity_;
    private double width_;
    private double length_;
    private double maxvoltage_;

    private SwerveDriveKinematics m_kinematics;

    private final AHRS m_navx = new AHRS(Port.kMXP);

    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public SwerveDriveBaseSubsystem(XeroRobot robot) throws Exception {
        super(robot, SubsystemName);

        int drivemotor, steermotor, encoder;
        double steeroffset;

        max_linear_velocity_ = getSetting("physical:maxspeed").getDouble();
        width_ = getSetting("physical:width").getDouble();
        length_ = getSetting("physical:length").getDouble();
        maxvoltage_ = getSetting("electrical:max-voltage").getDouble();

        Translation2d flpos = new Translation2d(width_ / 2.0, length_ / 2.0);
        Translation2d frpos = new Translation2d(width_ / 2.0, -length_ / 2.0);
        Translation2d blpos = new Translation2d(-width_ / 2.0, length_ / 2.0);
        Translation2d brpos = new Translation2d(-width_ / 2.0, -length_ / 2.0);

        m_kinematics = new SwerveDriveKinematics(flpos, frpos, blpos, brpos);
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        drivemotor = getSetting("hw:fl:drive:canid").getInteger();
        steermotor = getSetting("hw:fl:steer:canid").getInteger();
        encoder = getSetting("hw:fl:encoder:canid").getInteger();
        steeroffset = getSetting("hw:fl:encoder:offset").getDouble();
        m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                // This parameter is optional, but will allow you to see the current state of
                // the module on the dashboard.
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2, drivemotor, steermotor, encoder, steeroffset);

        drivemotor = getSetting("hw:fr:drive:canid").getInteger();
        steermotor = getSetting("hw:fr:steer:canid").getInteger();
        encoder = getSetting("hw:fr:encoder:canid").getInteger();
        steeroffset = getSetting("hw:fr:encoder:offset").getDouble();
        m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2, drivemotor, steermotor, encoder, steeroffset);

        drivemotor = getSetting("hw:bl:drive:canid").getInteger();
        steermotor = getSetting("hw:bl:steer:canid").getInteger();
        encoder = getSetting("hw:bl:encoder:canid").getInteger();
        steeroffset = getSetting("hw:bl:encoder:offset").getDouble();
        m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2, drivemotor, steermotor, encoder, steeroffset);

        drivemotor = getSetting("hw:br:drive:canid").getInteger();
        steermotor = getSetting("hw:br:steer:canid").getInteger();
        encoder = getSetting("hw:br:encoder:canid").getInteger();
        steeroffset = getSetting("hw:br:encoder:offset").getDouble();
        m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2, drivemotor, steermotor, encoder, steeroffset);
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        m_navx.zeroYaw();
    }

    public Rotation2d getGyroscopeRotation() {
        if (m_navx.isMagnetometerCalibrated()) {
            return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        }

        return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

        m_frontLeftModule.set(states[0].speedMetersPerSecond / max_linear_velocity_ * maxvoltage_,
                states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / max_linear_velocity_ * maxvoltage_,
                states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / max_linear_velocity_ * maxvoltage_,
                states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / max_linear_velocity_ * maxvoltage_,
                states[3].angle.getRadians());
    }
}
