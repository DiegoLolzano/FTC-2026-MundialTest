package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.CerboUtil.Interpolation.InterpolatingDouble;
import org.firstinspires.ftc.teamcode.Robot.CerboUtil.Interpolation.InterpolatingTreeMap;

public class Shooter extends SubsystemBase {
    //CHECK IF ITS RIGHT/LEFT OR UPPER/LOWER SHOOTER MOTOR
    DcMotorEx lowerShooter, upperShooter;
    HardwareMap hw;
    Telemetry tl;

    public static double kP = 16.0,
                         kI = 0.0,
                         kD = 0.0,
                         kF = 13.0;

    private double oldP, oldI, oldD, oldF;

    public Shooter(HardwareMap hw, Telemetry tl) {
        this.hw = hw;
        this.tl = tl;

        lowerShooter = hw.get(DcMotorEx.class, "lowerShooter");
        lowerShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidOrig = lowerShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoef = new PIDFCoefficients(kP, kI, kD, kF);
        lowerShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);
        
        upperShooter = hw.get(DcMotorEx.class, "upperShooter");
        upperShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidOrig2 = upperShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        //CHECK IF WE NEED TWO PIDFs VALUES FOR EACH SHOOTER
        upperShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);

        updatePIDFCoef();
    }

    public void setRPM(double RPM) {
        double TICKS_PER_REV = 28; // REV HD Hex motor (no gearbox)
        double gearRatio = 1;

        double targetTicksPerSeconds = (RPM * TICKS_PER_REV * gearRatio) / 60;
        lowerShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lowerShooter.setVelocity(targetTicksPerSeconds);
        lowerShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lowerShooter.setVelocity(targetTicksPerSeconds);
    }

    public void stopShooter() {
        lowerShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowerShooter.setPower(0.0);
        lowerShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowerShooter.setPower(0.0);
    }

    public double getLeftRPM() {
        double ticksPerSecond = lowerShooter.getVelocity();// ticks/sec
        double TICKS_PER_REV = 28.0; // REV HD Hex motor (no gearbox)
        double gearRatio = 1;

        return (ticksPerSecond * 60) / (TICKS_PER_REV * gearRatio);
    }

    public double getRightRPM() {
        double ticksPerSecond = upperShooter.getVelocity();// ticks/sec
        double TICKS_PER_REV = 28.0; // REV HD Hex motor (no gearbox)
        double gearRatio = 1;

        return (ticksPerSecond * 60) / (TICKS_PER_REV * gearRatio);
    }

    public double getAverageRPMS() {
        return (getLeftRPM() + getRightRPM()) / 2;
    }

    public void updatePIDFCoef() {
        if(kP != oldP || kI != oldI || kD != oldD || kF != oldF) {
            PIDFCoefficients newPid = new PIDFCoefficients(kP, kI, kD, kF);
            lowerShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPid);
            upperShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPid);
        }
    }

    @Override
    public void periodic() {
        updatePIDFCoef();

        tl.addData("Lower Shooter RPMs", getLeftRPM());
        tl.addData("Upper Shooter RPMs", getRightRPM());
        tl.addData("Average RPMs", getAverageRPMS());

    }
}
