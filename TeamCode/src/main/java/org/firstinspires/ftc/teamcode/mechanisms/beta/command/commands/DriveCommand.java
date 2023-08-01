package org.firstinspires.ftc.teamcode.mechanisms.beta.command.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.mechanisms.beta.command.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private DriveSubsystem m_drive;
    private DoubleSupplier m_forward;
    private DoubleSupplier m_strafe;
    private DoubleSupplier m_rotation;

    public DriveCommand(){
            
    }
}
