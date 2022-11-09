package org.xero1425.subsystems;

import java.lang.annotation.Target;

import org.xero1425.XeroRobot;
import org.xero1425.motors.BadMotorRequestException;
import org.xero1425.motors.MotorController;
import org.xero1425.motors.MotorRequestFailedException;
import org.xero1425.subsystems.motorsubsystem.XeroMotorEncoderSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import org.xero1425.settings.SettingsValue;

public class Intake2MotorSubsystem extends XeroMotorEncoderSubsystem {
    private MotorController spinner_;
    private double spinner_power_ ;

    private enum State {
        Stowing,
        Deploying,
        Stopped
    }

    private State state_;
    private ProfiledPIDController pid_;
    private Constraints constraints_;

    private double stowed_position_ ;
    private double deployed_position_ ;
    private double target_threshold_ ;

    public Intake2MotorSubsystem(XeroRobot robot, String name) throws Exception {
        super(robot, name, false); // Motor 1, in the base class

        String motorname = "subsystems:" + name + ":hw:spinner:motor" ;
        spinner_ = robot.getMotorFactory().createMotor("intake-spinner", motorname);

        stowed_position_ = getSetting("position:stowed").getDouble() ;
        deployed_position_ = getSetting("position:deployed").getDouble() ;
        target_threshold_ = getSetting("position:threshold").getDouble() ;

        double maxv = getSetting("constraints:max-velocity").getDouble() ;
        double maxa = getSetting("constraints:max-acceleration").getDouble() ;
        constraints_ = new Constraints(maxv, maxa) ;

        double kp = getSetting("pid:kp").getDouble() ;
        double ki = getSetting("pid:ki").getDouble() ;
        double kd = getSetting("pid:kd").getDouble() ;
        pid_ = new ProfiledPIDController(kp, ki, kd, constraints_) ;
        
        state_ = State.Stopped ;
    }

    public void stow() {
        state_ = State.Stowing ;
    }

    public void deploy() {
        state_ = State.Deploying ;
    }

    public void stop() {
        setPower(0.0) ;
        state_ = State.Stopped ;
    }

    @Override
    public void periodic() {
        
        double out = 0.0;
        double pos = getPosition() ;

        switch (state_) {
            case Stowing:
                if (Math.abs(pos - stowed_position_) < target_threshold_) {
                    state_ = State.Stopped;
                } else {
                    out = pid_.calculate(pos, stowed_position_);
                }
                break;
            case Deploying:
                if (Math.abs(pos - deployed_position_) < target_threshold_) {
                    state_ = State.Stopped;
                } else {
                    out = pid_.calculate(pos, deployed_position_);
                }
                break;
            case Stopped:
                break;
        }

        setPower(out) ;
    }

    public boolean isDeployed() {
        return Math.abs(getPosition() - deployed_position_) < target_threshold_ ;
    }

    public boolean isStowed() {
        return Math.abs(getPosition() - stowed_position_) < target_threshold_ ;
    }

    public void setSpinnerPower(double p) throws BadMotorRequestException, MotorRequestFailedException {
        spinner_power_ = p ;
        spinner_.set(p);
    }

    @Override
    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;

        if (name.equals("spinner-power")) {
            v = new SettingsValue(spinner_power_) ;
        }

        return v ;
    }
}

