package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret extends SubsystemBase {
    DcMotorEx turretMotor;
    HardwareMap hw;
    Telemetry tl;

    public Turret(HardwareMap hw, Telemetry tl) {
        this.hw = hw;
        this.tl = tl;

        turretMotor = hw.get(DcMotorEx.class, "turretMotor");
    }

    private double normalizeDegrees(double angleDeg) {
        double normalizedDeg = angleDeg % 360;
        if(normalizedDeg < 0) normalizedDeg += 360;
        return normalizedDeg;
    }
}
