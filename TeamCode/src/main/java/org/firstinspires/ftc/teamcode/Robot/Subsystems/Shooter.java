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
    DcMotorEx leftShooter, rightShooter;

    HardwareMap hw;

    Telemetry tl;

    public static double kP = 0.0,
                         kI = 0.0,
                         kD = 0.0,
                         kF = 0.0;

    private double oldP, oldI, oldD, oldF;

    static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
            kDistanceToShooterSpeed = new InterpolatingTreeMap<>();

    static {
        //DISTANCE AND SHOOTER SPEED RESPECTIVELY, TUNE AND CHECK FOR VALUES
        kDistanceToShooterSpeed.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
    }

    public Shooter(HardwareMap hw, Telemetry tl) {
        this.hw = hw;
        this.tl = tl;

        leftShooter = hw.get(DcMotorEx.class, "leftShooter");
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidOrig = leftShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoef = new PIDFCoefficients(kP, kI, kD, kF);
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);
        
        rightShooter = hw.get(DcMotorEx.class, "rightShooter");
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidOrig2 = rightShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        //CHECK IF WE NEED TWO PIDFs VALUES FOR EACH SHOOTER
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);

        updatePIDFCoef();
    }

    public void setRPM(double RPM){
        double TICKS_PER_REV = 28; // REV HD Hex motor (no gearbox)
        double gearRatio = 1;

        double targetTicksPerSeconds = (RPM * TICKS_PER_REV * gearRatio) / 60;
        leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftShooter.setVelocity(targetTicksPerSeconds);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShooter.setVelocity(targetTicksPerSeconds);
    }

    public double getLeftRPM(){
        double ticksPerSecond = leftShooter.getVelocity();// ticks/sec
        double TICKS_PER_REV = 28.0; // REV HD Hex motor (no gearbox)
        double gearRatio = 1;

        return (ticksPerSecond * 60) / (TICKS_PER_REV * gearRatio);
    }

    public double getRightRPM(){
        double ticksPerSecond = rightShooter.getVelocity();// ticks/sec
        double TICKS_PER_REV = 28.0; // REV HD Hex motor (no gearbox)
        double gearRatio = 1;

        return (ticksPerSecond * 60) / (TICKS_PER_REV * gearRatio);
    }

    public double getAverageRPMS(){
        return (getLeftRPM() + getRightRPM()) / 2;
    }

    public void updatePIDFCoef() {
        if(kP != oldP || kI != oldI || kD != oldD || kF != oldF) {
            PIDFCoefficients newPid = new PIDFCoefficients(kP, kI, kD, kF);
            leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPid);
            rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPid);
        }
    }

    public double getInterpolatedShooterSpeed(double distance) {
        return kDistanceToShooterSpeed.getInterpolated(new InterpolatingDouble(distance)).value;
    }

    @Override
    public void periodic() {
        updatePIDFCoef();

        tl.addData("Left Shooter RPMs", getLeftRPM());
        tl.addData("Right Shooter RPMs", getRightRPM());
        tl.addData("Average RPMs", getAverageRPMS());

    }
}
