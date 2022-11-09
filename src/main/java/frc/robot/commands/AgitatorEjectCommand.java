package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AgitatorSubsystem;

public class AgitatorEjectCommand extends CommandBase {
    private final AgitatorSubsystem subsystem_;

    public AgitatorEjectCommand(AgitatorSubsystem sub) {
        subsystem_ = sub ;
        addRequirements(sub) ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        try {
            subsystem_.eject() ;
        }
        catch(Exception ex) {
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        try {
            subsystem_.off() ;
        }
        catch(Exception ex) {
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false ;
    }
}
