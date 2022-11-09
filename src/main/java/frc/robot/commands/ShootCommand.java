package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PieceWiseLinear;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveBaseSubsystem;
import frc.robot.subsystems.TargetTrackerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootCommand extends CommandBase {

    private class ShooterParams {
        public final double WheelVelocity ;
        public final double HoodPosition ;
        public final boolean IsValid ;

        public ShooterParams(double wheel, double hood, boolean valid) {
            WheelVelocity = wheel ;
            HoodPosition = hood ;
            IsValid = valid ;
        }
    }

    private final TurretSubsystem turret_ ;
    private final ShooterSubsystem shooter_;
    private final TargetTrackerSubsystem tracker_ ;
    private final ConveyorSubsystem conveyor_ ;
    private final SwerveDriveBaseSubsystem db_ ;
    private boolean shooting_ ;
    private ShooterParams current_params_ ;
    private ShooterParams last_params_ ;
    private ShooterParams default_params_ ;
    private PieceWiseLinear pwl_hood_ ;
    private PieceWiseLinear pwl_velocity_ ;

    public ShootCommand(ShooterSubsystem shooter, ConveyorSubsystem conveyor, TargetTrackerSubsystem tracker, SwerveDriveBaseSubsystem db, TurretSubsystem turret) {
        shooter_ = shooter ;
        tracker_ = tracker ;
        conveyor_ = conveyor ;
        db_ = db ;
        turret_ = turret ;

        // We don't add the drive base and turret to the requirements, we just want it to read data from these
        addRequirements(shooter, tracker, conveyor) ;

        shooting_ = false ;
        default_params_ = new ShooterParams(shooter_.getSetting("default:velocity").getDouble(), shooter_.getSetting("default:hood").getDouble(), false) ;

        pwl_hood_ = new PieceWiseLinear(Arrays.asList(Constants.HoodPwlPoints)) ;
        pwl_velocity_ = new PieceWiseLinear(Arrays.asList(Constants.VelocityPwlPoints)) ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        tracker_.startTracking();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!shooting_) {
            if (tracker_.hasTarget()) {

                current_params_ = getShooterParams() ;
                shooter_.setShootingParams(current_params_.HoodPosition, current_params_.WheelVelocity) ;
                if (isShooterReady() && isTurretReady() && isDriveBaseReady()) {
                    conveyor_.shoot();
                    shooting_ = true ;
                }
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        tracker_.stopTracking();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return conveyor_.getBallCount() == 0 ;
    }

    private double getPercent(double target, double actual) {
        return Math.abs(target - actual) / target * 100.0 ;
    }

    private boolean isShooterReady() {
        boolean ret = false ;

        if (current_params_.IsValid) {
            double hood = getPercent(current_params_.HoodPosition, shooter_.getHood()) ;
            double vel = getPercent(current_params_.WheelVelocity, shooter_.getVelocity()) ;

            if (hood < Constants.ShooterHoodPercentThreshold && vel < Constants.ShooterVelocityPercentThreshold) {
                ret = true ;
            }
        }

        return ret ;
    }

    private boolean isTurretReady() {
        return turret_.isReady() ;
    }

    private boolean isDriveBaseReady () {
        // return db_.getVelocity() < Constants.ShooterDriveBaseMaxVelocity ;
        return true ;
    }

    private ShooterParams getShooterParams() {
        ShooterParams sp ;

        if (tracker_.hasTarget()) {
            sp = computeShooterParams(tracker_.getDistance()) ;
            last_params_ = new ShooterParams(sp.WheelVelocity, sp.HoodPosition, false) ;
        }
        else if (last_params_ != null) {
            sp = last_params_ ;
        }
        else {
            sp = getDefaultShooterParams() ;
        }
        return sp ;
    }

    private ShooterParams computeShooterParams(double dist) {
        double hood = pwl_hood_.getValue(dist) ;
        double wheel = pwl_velocity_.getValue(dist) ;

        return new ShooterParams(wheel, hood, true) ;
    }

    private ShooterParams getDefaultShooterParams() {
        return default_params_ ;
    }
}
