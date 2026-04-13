package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret extends SubsystemBase {
    DcMotorEx turretMotor;
    HardwareMap hw;
    Telemetry tl;

    public static double kP = 0.0,
            kI = 0.0,
            kD = 0.0;

    public static PIDController pidController = new PIDController(kP, kI, kD);

    private boolean pidMode = false;

    public Turret(HardwareMap hw, Telemetry tl) {
        this.hw = hw;
        this.tl = tl;

        turretMotor = hw.get(DcMotorEx.class, "turretMotor");
    }

    public void setTurretPosition(double degrees) {
        pidMode = true;
        pidController.setSetPoint(degrees);
    }

    public double getTurretPosition(){
        double currentDegrees = ticksToDegrees();
        return -currentDegrees;
    }

    public double ticksToDegrees(){
        double motorTicksPerRev = 8192;
        int ticks = turretMotor.getCurrentPosition();
        double gearRatio = 5;
        double ticksPerTurretRev = motorTicksPerRev * gearRatio;
        return ((ticks / ticksPerTurretRev) * 360);
    }

    private double normalizeDegrees(double angleDeg) {
        double normalizedDeg = angleDeg % 360;
        if(normalizedDeg < 0) normalizedDeg += 360;
        return normalizedDeg;
    }

    @Override
    public void periodic() {
        double currentDegrees = -ticksToDegrees();

        pidController.setPID(kP, Math.abs(pidController.getPositionError()) < 0.9 ? 0:kI, kD);

        double power = pidController.calculate(currentDegrees);

        turretMotor.setPower(power);
    }
}
