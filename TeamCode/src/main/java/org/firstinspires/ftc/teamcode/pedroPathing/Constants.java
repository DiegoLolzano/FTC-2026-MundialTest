package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

public class Constants {
    /*
     * TODO: PLACEHOLDER VALUES
     *  VALUES THAT NEED TO BE CHECKED:
     *  -MASS
     *  -DRIVE MOTOR DIRECTIONS
     *  -LOCALIZATION
     *  -PIDFs CONSTANTS
     */
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.5)
            .forwardZeroPowerAcceleration(-30.74650453781977)
            .lateralZeroPowerAcceleration(-74.88549106855973)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0.005, 0))//.translationalPIDFCoefficients(new PIDFCoefficients(0.11,0,0.009,0.044))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0072, 0, 0.00008, 0, 0))
            .centripetalScaling(0.001);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftFrontMotorName("lf")
            .leftRearMotorName("lr")
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)//FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)//REVERSE)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)//FORWARD)
                .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)//FORWARD)
            .useBrakeModeInTeleOp(true)
            .xVelocity(71.03902933916709)
            .yVelocity(46.70697742372048);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-4.33)//6.08
            .strafePodX(1.5)//0.2
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .yawScalar(1);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.999,
            0.1,
            0.1,
            0.009,
            50,
            0.9,
            10,
            0.9
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
