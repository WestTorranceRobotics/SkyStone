package org.westtorrancerobotics.lib.hardware.drive;

import org.westtorrancerobotics.lib.spline.geom.Angle;
import org.westtorrancerobotics.lib.spline.geom.Point;

public class MecanumController implements MecanumDrive {
    
    private final MecanumDrive train;

    public MecanumController(MecanumDrive train) {
        this.train = train;
    }
    
    public void translate(double x, double y, TranslationMethod way) {
        translate(new Angle(x, y), Math.hypot(x, y), way);
    }
    
    public void translate(Angle ang, double speed, TranslationMethod way) {
        double x = ang.getValue(Angle.AngleUnit.RADIANS, Angle.AngleOrientation.COMPASS_HEADING) % (2*Math.PI);
        x = x < 0 ? x + 2*Math.PI : x;
        double a = Math.cos(x - Math.PI / 4);
        double b = Math.cos(x + Math.PI / 4);
        double sc = 1;
        switch (way) {
            case CONSTANT_POWER:
                sc = 1 / (a + b) * speed;
                break;
            case CONSTANT_SPEED:
                sc = speed;
                break;
            case MAX_SPEED:
                sc = Math.max(Math.abs(a), Math.abs(b));
                sc = 1 / sc;
                break;
        }
        a *= sc;
        b *= sc;
        train.setMotorPowers(a, b, b, a);
    }
    
    public void rotate(double power) {
        setMotorPowers(power, power, -power, -power);
    }
    
    public void spinDrive(double x, double y, double turn, TranslTurnMethod way) {
        spinDrive(new Angle(x, y), Math.hypot(x, y), turn, way);
    }
    
    public void spinDrive(Angle ang, double speed, double turn, TranslTurnMethod way) {
        double x = ang.getValue(Angle.AngleUnit.RADIANS, Angle.AngleOrientation.COMPASS_HEADING);
        double a = Math.cos(x - Math.PI / 4);
        double b = Math.cos(x + Math.PI / 4);
        double sc = 1;
        double scTurn = 0;
        double turnPwr;
        switch(way) {
            case CONSTANT_TRANSLATION_SPEED:
                sc = speed;
                scTurn = 1 - Math.max(Math.abs(a), Math.abs(b)) * speed;
                break;
            case EQUAL_POWERS:
                sc = speed / (Math.abs(a) + Math.abs(b));
                scTurn = 0.5;
                break;
            case EQUAL_SPEED_RATIOS:
                double max = Math.max(Math.abs(turn), Math.abs(speed));
                scTurn = Math.abs(turn) * max / (Math.abs(turn) + Math.abs(speed));
                sc = max - scTurn;
                double fact = 1 - (1 - Math.max(Math.abs(a), Math.abs(b))) * sc;
                scTurn /= fact;
                sc /= fact;
                break;
            case CONSTANT_BOTH_SPEED:
                if (Math.abs(speed + turn - 1) > 0.0001) {
                    throw new IllegalArgumentException("Speed and turn must add to one " +
                            "for CONSTANT_BOTH_SPEED drive mode");
                }
                sc = speed;
                scTurn = turn;
                break;
        }
        a *= sc;
        b *= sc;
        turnPwr = turn * scTurn;
        this.train.setMotorPowers(a + turnPwr, b + turnPwr, b - turnPwr, a - turnPwr);
    }
    
    public void orbit(Point relativeCenter, double power) {
        double x = relativeCenter.x;
        double y = relativeCenter.y;
        double r = Math.hypot(x, y);
        Angle ang = new Angle(y, -x);
        if (getWheelbaseWidth() > r) {
            spinDrive(ang, r / getWheelbaseWidth(), 1, TranslTurnMethod.EQUAL_SPEED_RATIOS);
        } else {
            spinDrive(ang, 1, getWheelbaseWidth() / r, TranslTurnMethod.EQUAL_SPEED_RATIOS);
        }
    }

    @Override
    public double getWheelbaseWidth() {
        return train.getWheelbaseWidth();
    }
    
    @Override
    public void setMotorPowers(double frontLeft, double backLeft, double frontRight, double backRight) {
        train.setMotorPowers(frontLeft, backLeft, frontRight, backRight);
    }
}
