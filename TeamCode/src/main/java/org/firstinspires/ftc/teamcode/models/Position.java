package org.firstinspires.ftc.teamcode.models;

public class Position {
    public double X;
    public double Y;

    public Position(double x, double y) {
        this.X = x;
        this.Y = y;

    }

    @Override
    public String toString() {
        return " Position [x = " + X + ", y  =" + Y + "]";
    }
}