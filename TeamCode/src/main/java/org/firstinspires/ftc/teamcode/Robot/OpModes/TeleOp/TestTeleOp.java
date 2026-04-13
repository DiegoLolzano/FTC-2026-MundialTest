package org.firstinspires.ftc.teamcode.Robot.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
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

    Trigger leftTrigger;
    Trigger rightTrigger;

    Pose2d testGoal;

    @Override
    public void initialize() {
        m_drive = new Drivetrain(hardwareMap, telemetry, true, true);
        m_intake = new Intake(hardwareMap);
        m_shooter = new Shooter(hardwareMap, telemetry);
        m_turret = new Turret(hardwareMap, telemetry);

        g1 = new GamepadEx(gamepad1);

        leftTrigger = new Trigger(() -> g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);
        rightTrigger = new Trigger(() -> g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1);

        testGoal = new Pose2d(0.0, 0.0, Math.toDegrees(0.0));

        m_drive.setDefaultCommand(new DriveCommand(m_drive,
                g1::getLeftX,
                g1::getLeftY,
                g1::getRightY));

        //CHECK TURRET LOOKAHEAD POSITION
        m_turret.setDefaultCommand(new TurretCommand(m_drive,
                m_turret,
                telemetry,
                testGoal));

        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(() -> m_intake.intakeRoller(), m_intake))
                .whenReleased(new RunCommand(() -> m_intake.stopRoller()));

        rightTrigger
                .whileActiveContinuous(new RunCommand(() -> m_intake.outtakeRoller(), m_intake))
                .whenInactive(new RunCommand(() -> m_intake.stopRoller(), m_intake));

        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new ParallelCommandGroup(
                        new RunCommand(() -> m_intake.feedShooter(), m_intake),
                        new RunCommand(() -> m_intake.openShooterPath(), m_intake)
                ))
                .whenReleased(new ParallelCommandGroup(
                        new RunCommand(() -> m_intake.blockShooterPath(), m_intake),
                        new RunCommand(() -> m_intake.stopRoller(), m_intake)
                ));

        //CHECK DRIVE TO TARGET DISTANCE
        leftTrigger
                .whileActiveContinuous(new RunCommand(() -> {
                    ShotCalculator.ShootingParameters params = ShotCalculator.getInstance().calculate(
                            m_drive.getPose(),
                            m_drive.getVelocity(),
                            testGoal
                    );
                    if (params.isValid) {
                        m_shooter.setRPM((int) params.flywheelRPM);
                    }
                }, m_shooter))
                .whenInactive(new InstantCommand(() -> m_shooter.stopShooter(), m_shooter));

        schedule(new RunCommand(() -> {
            telemetry.update();
        }));
    }
}
