package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.CerboUtil.ShotCalculator;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class TurretCommand extends CommandBase {
    private final Drivetrain m_drive;
    private final Turret m_turret;
    //private final Shooter m_shooter;
    private final Pose2d targetPoint; // GOAL_POS
    private final Telemetry tl;

    private static final double MIN_LIMIT = -57.0;
    private static final double MAX_LIMIT = 270.0;

    public TurretCommand(Drivetrain m_drive, Turret m_turret, Telemetry tl, Pose2d targetPoint) {
        this.m_drive = m_drive;
        this.m_turret = m_turret;
        this.tl = tl;
        this.targetPoint = targetPoint;

        addRequirements(m_turret);
    }

    @Override
    public void execute() {
        ShotCalculator.ShootingParameters params = ShotCalculator.getInstance().calculate(
                m_drive.getPose(),
                m_drive.getVelocity(),
                targetPoint
        );

        if (!params.isValid) return;

        double targetAngleFieldDegrees = Math.toDegrees(params.targetFieldHeading);

        double robotHeading = Math.toDegrees(m_drive.getPose().getHeading());
        double relativeTarget = angleWrap(targetAngleFieldDegrees - robotHeading);

        double finalSetpoint = relativeTarget;

        if (finalSetpoint < MIN_LIMIT) {
            if (finalSetpoint + 360 <= MAX_LIMIT) finalSetpoint += 360;
        } else if (finalSetpoint > MAX_LIMIT) {
            if (finalSetpoint - 360 >= MIN_LIMIT) finalSetpoint -= 360;
        }

        double clampedSetpoint = Math.max(MIN_LIMIT, Math.min(MAX_LIMIT, finalSetpoint));

        m_turret.setTurretPosition(clampedSetpoint);
        //m_shooter.setRPM(params.flywheelRPM);

        tl.addData("Compensated Distance", params.distanceToTarget);
    }

    private double angleWrap(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees <= -180) degrees += 360;
        return degrees;
    }
}