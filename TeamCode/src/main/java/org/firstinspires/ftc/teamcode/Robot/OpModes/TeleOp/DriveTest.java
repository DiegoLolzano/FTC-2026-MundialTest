package org.firstinspires.ftc.teamcode.Robot.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

@TeleOp (name = "DriveTest", group = "Tests")
public class DriveTest extends CommandOpMode {
    private Drivetrain m_drive;

    @Override
    public void initialize() {
        m_drive = new Drivetrain(hardwareMap, telemetry, true, true);

        GamepadEx g1 = new GamepadEx(gamepad1);

        m_drive.setDefaultCommand(new DriveCommand(m_drive,
                g1::getLeftX,
                g1::getLeftY,
                g1::getRightY));

        schedule(new RunCommand(() -> {telemetry.update();}));
    }
}
