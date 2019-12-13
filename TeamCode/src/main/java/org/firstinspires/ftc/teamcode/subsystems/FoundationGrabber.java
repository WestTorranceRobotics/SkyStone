package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.ButtonAndEncoderData;

import java.util.function.BooleanSupplier;
import java.util.function.DoublePredicate;

public class FoundationGrabber {

    private Servo leftHook;
    private RevColorSensorV3 leftEye;
    private Servo rightHook;
    private RevColorSensorV3 rightEye;

    private final double LEFT_GRABBED_POSITION = 0.31;
    private final double LEFT_UNGRABBED_POSITION = 0.847;
    private final double RIGHT_GRABBED_POSITION = 0.61;
    private final double RIGHT_UNGRABBED_POSITION = 0.076;

    private final double RED_TO_BLUE_THRESHOLD = 1.6;
    private final double GREEN_TO_BLUE_THRESHOLD = 2.5;
    private final double MAX_STONE_VISIBLE_DISTANCE_IN = 4;

    private static FoundationGrabber instance = null;

    public static synchronized FoundationGrabber getInstance() {
        return instance != null ? instance : (instance = new FoundationGrabber());
    }

    private FoundationGrabber() {}

    public void init(HardwareMap hardwareMap) {
        leftHook = hardwareMap.get(Servo.class, "foundationHookLeft");


        double aParam = 519.837;
        double bInvParam = 0.467;
        double cParam = 175.572;
        double dParam = -0.753;
        leftEye = new RevColorSensorV3(
                hardwareMap.get(RevColorSensorV3.class, "ssColorLeft").getDeviceClient()
        ) {
            @Override
            protected double inFromOptical(int rawOptical) {
                if (rawOptical <= cParam) {
                    return Double.POSITIVE_INFINITY;
                }
                return Math.pow((rawOptical - cParam) / aParam, -bInvParam) + dParam;
            }
        };

        rightHook = hardwareMap.get(Servo.class, "foundationHookRight");

        rightEye = new RevColorSensorV3(
                hardwareMap.get(RevColorSensorV3.class, "ssColorRight").getDeviceClient()
        ) {
            @Override
            protected double inFromOptical(int rawOptical) {
                if (rawOptical <= cParam) {
                    return Double.POSITIVE_INFINITY;
                }
                return Math.pow((rawOptical - cParam) / aParam, -bInvParam) + dParam;
            }
        };


    }

    public void setGrabbed(Hook hook, boolean grabbed) {
        switch (hook) {
            case LEFT:
                leftHook.setPosition(grabbed ? LEFT_GRABBED_POSITION : LEFT_UNGRABBED_POSITION);
                break;
            case RIGHT:
                rightHook.setPosition(grabbed ? RIGHT_GRABBED_POSITION : RIGHT_UNGRABBED_POSITION);
                break;
            case BOTH:
                leftHook.setPosition(grabbed ? LEFT_GRABBED_POSITION : LEFT_UNGRABBED_POSITION);
                rightHook.setPosition(grabbed ? RIGHT_GRABBED_POSITION : RIGHT_UNGRABBED_POSITION);
                break;
            default:
                throw new IllegalArgumentException("Unknown or invalid hook " + hook + " provided.");
        }
    }

    public SeenObject getView(Hook hook, double distance) {
        if (distance > MAX_STONE_VISIBLE_DISTANCE_IN) {
            return SeenObject.NOTHING;
        }
        switch (hook) {
            case LEFT:
                NormalizedRGBA rgbaL = leftEye.getNormalizedColors();
                return rgbaL.red > rgbaL.blue * RED_TO_BLUE_THRESHOLD && rgbaL.green > rgbaL.blue * GREEN_TO_BLUE_THRESHOLD
                    ? SeenObject.STONE : SeenObject.SKYSTONE;
            case RIGHT:
                NormalizedRGBA rgbaR = rightEye.getNormalizedColors();
                return rgbaR.red > rgbaR.blue * RED_TO_BLUE_THRESHOLD && rgbaR.green > rgbaR.blue * GREEN_TO_BLUE_THRESHOLD
                        ? SeenObject.STONE : SeenObject.SKYSTONE;
            default:
                throw new IllegalArgumentException("Unknown or invalid hook " + hook + " provided.");
        }
    }

    public double getDistance(Hook hook) {
        switch (hook) {
            case LEFT:
                return leftEye.getDistance(DistanceUnit.INCH);
            case RIGHT:
                return rightEye.getDistance(DistanceUnit.INCH);
            default:
                throw new IllegalArgumentException("Unknown or invalid hook " + hook + " provided.");
        }
    }

    public boolean getDistancePredicated(Hook hook, DoublePredicate predicate) {
        switch (hook) {
            case LEFT:
                return predicate.test(getDistance(Hook.LEFT));
            case RIGHT:
                return predicate.test(getDistance(Hook.RIGHT));
            case EITHER:
                return predicate.test(getDistance(Hook.LEFT)) || predicate.test(getDistance(Hook.RIGHT));
            case BOTH:
                return predicate.test(getDistance(Hook.LEFT)) && predicate.test(getDistance(Hook.RIGHT));
            default:
                throw new IllegalArgumentException("Unknown or invalid hook " + hook + " provided.");
        }
    }

    public enum Hook {
        LEFT,
        RIGHT,
        BOTH,
        EITHER
    }

    public enum SeenObject {
        STONE,
        SKYSTONE,
        NOTHING
    }
}
