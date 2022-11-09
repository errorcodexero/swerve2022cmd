package frc.robot.subsystems;

import org.xero1425.XeroRobot;
import org.xero1425.subsystems.motorsubsystem.XeroMotorSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class ConveyorSubsystem extends XeroMotorSubsystem {
    private final static String SubsystemName = "conveyor" ;

    private enum State {
        Idle,
        WaitForIntake,
        WaitForMiddle,
        WaitForShooter,
        WaitForIntake2,
        Eject,
        StartShoot,
        Shooting,
    }

    private State state_ ;

    private DigitalInput intake_sensor_;
    private DigitalInput shooter_sensor_;
    private DigitalInput middle_sensor_;

    private double eject_power_ ;
    private double collect_power_ ;
    private double shoot_power_ ;

    private double duration_eject_ ;
    private double duration_shoot_ ;

    private int ball_count_ ;
    private boolean isStop_ ;
    private Timer timer_ ;

    public ConveyorSubsystem(XeroRobot robot) throws Exception {
        super(robot, SubsystemName) ;

        int sensor ;
        
        sensor = getSetting("sensors:intake").getInteger() ;
        intake_sensor_ = new DigitalInput(sensor) ;

        sensor = getSetting("sensors:shooter").getInteger() ;
        shooter_sensor_ = new DigitalInput(sensor) ;

        sensor = getSetting("sensors:middle").getInteger() ;        
        middle_sensor_ = new DigitalInput(sensor) ;
        
        state_ = State.Idle ;
        timer_ = new Timer() ;
        isStop_ = false ;

        eject_power_ = getSetting("power:eject").getDouble() ;
        collect_power_ = getSetting("power:collectg").getDouble() ;
        shoot_power_ = getSetting("power:shoot").getDouble() ;

        duration_eject_ = getSetting("duration:eject").getDouble() ;
        duration_shoot_ = getSetting("duration:shoot").getDouble() ;
    }

    @Override
    public void periodic() {
        super.periodic();

        switch(state_) {
            case Idle:
                IdleProc() ;
                break ;
                case WaitForIntake:
                WaitForIntakeProc();
                break;

            case WaitForMiddle:
                WaitForMiddleProc();
                break;

            case WaitForShooter:
                WaitForShooterProc();
                break;

            case WaitForIntake2:
                WaitForIntake2Proc();
                break;

            case Eject:
               EjectProc(); 
               break;

            case StartShoot:
                StartShootProc() ;
                break ;

            case Shooting:
                ShootingProc() ;
                break ;
        }
    }
  
    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
    }

    public int getBallCount() {
        return ball_count_ ;
    }

    public boolean isFull() {
        return ball_count_ == 2 ;
    }

    public void collect() {
        if (state_ == State.Idle) {
            state_ = State.WaitForIntake ;
        }
    }

    public void shoot() {
        if (state_ == State.Idle) {
            state_ = State.StartShoot ;
        }
    }

    public void eject() {
        setPower(eject_power_) ;
        state_ = State.Eject ;
        timer_.reset() ;
    }

    public void stop() {
        isStop_ = true ;
    }

    public void setPreloadedBall() {
        ball_count_ = 1;
    }

    private boolean getIntakeSensor() {
        return !intake_sensor_.get() ;
    }

    private boolean getMiddleSensor() {
        return !middle_sensor_.get() ;
    }

    private boolean getShooterSensor() {
        return !shooter_sensor_.get() ;
    }
    
    private void WaitForIntakeProc() {
        //
        // In this method, we have started a collect operation but currently have no balls.
        // We are waiting for a ball to break the intake sensor at which point we will start
        // the conveyor motor to move the ball into the conveyor. 
        //
        if(isStop_) {
            setPower(0.0) ;
            state_ = State.Idle; 
        }
        if (getIntakeSensor()) {
            setPower(collect_power_) ;
            state_ = State.WaitForMiddle;
        }
    }

    private void WaitForMiddleProc() {
        //
        // After the intake sensor is broken and the motor is turned on, we check to see
        // if there is a second ball at the intake sensor. If there is, we keep the motor running 
        // and wait for the shooter sensor to be broken. If the middle sensor is broken
        // and there isn't another ball behind it, we turn off the motor to park the ball in the middle. 
        //
        if (getIntakeSensor() && getMiddleSensor()) {
            setPower(collect_power_) ;
            ball_count_ = 2;
            state_ = State.WaitForShooter;
        } else if (getMiddleSensor()) {
            setPower(0.0) ;
            ball_count_ = 1;
            state_ = State.WaitForIntake2;
        }
    }

    private void WaitForShooterProc() {
        //
        // In this case, we will always have two balls in the conveyor. Here, we are waiting for the
        // shooter sensor to be broken so that we can park the balls there. After we stop the motors, we set
        // the state to WaitForIntake. (should it be set to idle??)
        //
        if (getShooterSensor()) {
            setPower(0.0) ;
            ball_count_ = 2;
            state_ = State.Idle;
        }
    }

    private void WaitForIntake2Proc() {
        //
        // In this case, we have one ball stationed at the middle sensor. We are waiting for a second ball
        // to break the intake sensor so that we can turn on the motor power and move the balls towards the shooter.
        //
        if(isStop_) {
            setPower(0.0) ;
            state_ = State.Idle;
        }
        if (getIntakeSensor() && getMiddleSensor()) {
            setPower(collect_power_) ;
            ball_count_ = 1; 
            state_ = State.WaitForShooter;
        }
    }
     
    private void EjectProc() {
        if (timer_.hasElapsed(duration_eject_)) {
            ball_count_ = 0; 
            setPower(0.0) ;;
            state_ = State.Idle; 
        }
    }

    private void IdleProc() {
        if(isStop_){
            isStop_ = false; 
        }
    }

    private void StartShootProc() {
        if (isStop_) {
            setPower(0.0) ;
            state_ = State.Idle ;
        }
        else {
            setPower(shoot_power_) ;
            state_ = State.Shooting ;
            timer_.reset() ;
        }
    }

    private void ShootingProc() {
        if (isStop_ || timer_.hasElapsed(duration_shoot_)) {
            setPower(0.0) ;
            state_ = State.Idle ;
            ball_count_ = 0 ;
        }
    }
}
