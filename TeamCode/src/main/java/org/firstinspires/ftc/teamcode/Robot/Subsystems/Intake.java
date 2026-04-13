package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Intake extends SubsystemBase {
    DcMotorEx intakeRoller;
    //Check Servo function for correct naming
    ServoEx blockerServo;

    HardwareMap hw;
    public Intake(HardwareMap hw) {
        this.hw = hw;

        intakeRoller = hw.get(DcMotorEx.class, "intakeRoller");
        intakeRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        blockerServo = new ServoEx(hw, "blockerServo", 0, 180);
        blockerServo.setInverted(true);
    }

    /* INTAKE ROLLER */
    public void IntakeRoller() {
        intakeRoller.setPower(.88);
    }

    public void outtakeRoller() {
        intakeRoller.setPower(-1);
    }

    public void stopRoller() {
        intakeRoller.setPower(0.0);
    }

    public void feedShooter() {
        intakeRoller.setPower(.3);
    }

    public void overrideRollerSpeed(double power) {
        intakeRoller.setPower(power);
    }

    /* SERVO BLOCKER */
    public void blockShooterPath() {
        blockerServo.set(57);
    }

    public void openShooterPath() {
        blockerServo.set(95);
    }

    public void overrideBlockerAngle(double angle) {
        blockerServo.set(angle);
    }
}
