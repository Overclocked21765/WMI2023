package org.firstinspires.ftc.teamcode.mechanisms.beta.command.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.mechanisms.beta.command.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class LiftHighCommand extends CommandBase {
    private final LiftSubsystem m_lift;

    public LiftHighCommand(LiftSubsystem subsystem){
        m_lift = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        m_lift.setTarget(Constants.HIGH_POSITION);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
