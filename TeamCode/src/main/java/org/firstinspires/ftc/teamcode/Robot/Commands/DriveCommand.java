package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private Drivetrain m_drive;
    private DoubleSupplier x, y, turn;
    //private boolean robotCentric;

    public DriveCommand(Drivetrain m_drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier turn) {
        this.m_drive = m_drive;
        this.x = x;
        this.y = y;
        this.turn = turn;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_drive.startTeleOp();
    }

    @Override
    public void execute() {
        //Check controller values and correct as needed
        m_drive.drive(x.getAsDouble(), y.getAsDouble(), turn.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {return false;}
}
