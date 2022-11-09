package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDeployCommand extends CommandBase {
    private final IntakeSubsystem subsystem_;

    public IntakeDeployCommand(IntakeSubsystem sub) {
        subsystem_ = sub ;
        addRequirements(sub) ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        subsystem_.deploy();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return subsystem_.isDeployed() ;
    }
}
