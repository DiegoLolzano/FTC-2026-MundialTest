package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.panels.Panels;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

/*
* TODO: NEEDS LOCALIZATION AND PIDFs COEFFICIENT TUNING FOR AUTO USE
*  CHECK: https://pedropathing.com/docs/pathing/tuning/localization
*/
public class Drivetrain extends SubsystemBase {
    HardwareMap hw;
    Telemetry tl;
    Follower m_follower;
    Panels panels;
    static PoseHistory poseHistory;

    boolean isBlueAlliance;
    boolean isClose;
    public Drivetrain(HardwareMap hw, Telemetry tl, boolean isBlueAlliance, boolean isClose){
        this.hw = hw;
        this.tl = tl;
        this.m_follower = Constants.createFollower(hw);
        poseHistory = m_follower.getPoseHistory();
        this.isBlueAlliance = isBlueAlliance;
        this.isClose = isClose;

        //CENTER OF THE STARTING CONE, BACK OF THE ROBOT IN THE WALL
        this.m_follower.setPose(new Pose(71.0, 7.5, Math.toRadians(45)));

        setSubsystem("Drivetrain");

        m_follower.getPoseTracker();
    }

    public void drawCurrent() {
        try {
            Drawing.drawRobot(m_follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public void drawCurrentAndHistory() {
        Drawing.drawPoseHistory(poseHistory);
        drawCurrent();
    }

    public void startPose(Pose pose) {m_follower.setStartingPose(pose);}

    public void startTeleOp() {m_follower.startTeleOpDrive();}

    public void followPath(PathChain path) {m_follower.followPath(path);}

    public PathBuilder pathBuilder() {return m_follower.pathBuilder();}

    public boolean isPathFinished() {return !m_follower.isBusy();}

    public void stopPathFollowing() {m_follower.breakFollowing();}

    public Follower getFollower() {return m_follower;}

    public void drive(double x, double y, double turn) {
        if(isBlueAlliance) {
            m_follower.setTeleOpDrive(x, y, turn, false);
        } else {
            m_follower.setTeleOpDrive(-x, -y, turn, false);
        }
    }

    public Pose getPose() {
        return m_follower.getPose();
    }

    public Vector getVelocity() {
        return m_follower.getVelocity();
    }

    public double getPower() {
        return m_follower.getMaxPowerScaling();
    }

    private double normalizeDegrees(double angleDeg) {
        double normalizedDeg = angleDeg % 360;
        if(normalizedDeg < 0) normalizedDeg += 360;
        return normalizedDeg;
    }

    @Override
    public void periodic() {
        m_follower.update();
    }

}
