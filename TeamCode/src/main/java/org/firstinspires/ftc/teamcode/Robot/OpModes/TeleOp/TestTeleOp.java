package org.firstinspires.ftc.teamcode.Robot.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Robot.CerboUtil.ShotCalculator;
import org.firstinspires.ftc.teamcode.Robot.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.TurretCommand;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

@TeleOp (name = "TestTeleOp")
public class TestTeleOp extends CommandOpMode {
    private Drivetrain m_drive;
    private Intake m_intake;
    private Shooter m_shooter;
    private Turret m_turret;

    GamepadEx g1;

    @Override
    public void initialize() {
        m_drive = new Drivetrain(hardwareMap, telemetry, true, true);
        m_intake = new Intake(hardwareMap);
        m_shooter = new Shooter(hardwareMap, telemetry);
        m_turret = new Turret(hardwareMap, telemetry);

        g1 = new GamepadEx(gamepad1);

        m_drive.setDefaultCommand(new DriveCommand(m_drive,
                g1::getLeftX,
                g1::getLeftY,
                g1::getRightY));

        m_turret.setDefaultCommand(new TurretCommand(m_drive, m_turret, telemetry, new Pose2d(0.0, 0.0, Math.toDegrees(0))));
    }
}
