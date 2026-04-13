package org.firstinspires.ftc.teamcode.Robot.CerboUtil.Interpolation;

public interface Interpolable<T> {
    T interpolate(T other, double x);
}