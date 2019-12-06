package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.text.DecimalFormat;


@TeleOp(name="RUN THIS YOU MEERKAT", group="none")
//@Disabled
public class simpleOdo extends OpMode {
    private double orange;
    private double distance;
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private String xyd;
    private DcMotorEx rightBack;
    private DcMotor hEncoder;
    private DcMotor vEncoder1;
    private DcMotor vEncoder2;
    private double lastVertical1 = 0;
    private double lastVertical2 = 0;
    private double lastHorizontal = 0;
    private RevColorSensorV3 leftEye;
    private RevColorSensorV3 rightEye;
    private double deltaTheta;
    private double horizontal;
    private double vertical2;
    private double vertical1;
    private double theta;
    private double centerY = 0;
    private final double WHEEL_DISTANCE = 14.5;
    private double lastTheta = 0;
    // index 0 is x, index 1 is y, index 2 is heading
    private double[] position = new double[3];
    private final double TICKS_TO_INCHES = (2 * Math.PI) / 4096;
    private static simpleOdo instance = null;
    private double xReturn;
    private double yReturn;
    private double dReturn;
    DecimalFormat round = new DecimalFormat("#0.00");

    public static synchronized simpleOdo getInstance() {
        return instance != null ? instance : (instance = new simpleOdo());
    }
    public void init() {
        leftEye = hardwareMap.get(RevColorSensorV3.class, "ssColorLeft");
        rightEye = hardwareMap.get(RevColorSensorV3.class, "ssColorRight");
        position [0] = 0;
        position[1] = 0;
        position[2] = 0;
        vEncoder1 = hardwareMap.get(DcMotorEx.class, "intakeLeft/odometerLeftY");
        vEncoder2 = hardwareMap.get(DcMotorEx.class, "intakeRight/odometerRightY");
        hEncoder = hardwareMap.get(DcMotorEx.class, "liftRight/odometerX");
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        vEncoder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vEncoder1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vEncoder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vEncoder2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }@Override
    public void init_loop() {
    }
    @Override
    public void start() {
    }
    @Override
    public void loop() {
        vertical1 = vEncoder1.getCurrentPosition() * TICKS_TO_INCHES;
        vertical2 = vEncoder2.getCurrentPosition() * TICKS_TO_INCHES;
        horizontal = hEncoder.getCurrentPosition() * TICKS_TO_INCHES;
        centerY = ((vertical2 - lastVertical2) + (vertical1 - lastVertical1)) / 2;
        deltaTheta = (((vertical2 - lastVertical2) - (vertical1 - lastVertical1)) / 14);
        theta = lastTheta + deltaTheta;
        position[0] =  Math.cos(theta) > 0.015|| Math.cos(theta) <-0.015 ? position [0]  + (horizontal-lastHorizontal) * Math.cos(theta)
            : position [0] + Math.sin(theta) * centerY;
        position[1] =  Math.cos(theta) > 0.015|| Math.cos(theta) <-0.015 ? position[1] + centerY * Math.cos(theta)
                : position[1] + Math.sin(theta) * (horizontal - lastHorizontal) ;
        position[2] = ((theta) * 180) / Math.PI;
        xyd = "(" + round.format(position[0]) + ", " + round.format(position[1]) + ", " + round.format(position[2]) + ")";
        distance = vEncoder2.getCurrentPosition();
        if (gamepad2.a) {vEncoder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        telemetry.addData("degrees", orange);
        telemetry.addData("x, y, heading", xyd);
        lastVertical1 = vertical1;
        lastVertical2 = vertical2;
        lastHorizontal = horizontal;
        lastTheta = theta;
        }
    @Override
    public void stop() {

    }
    }





