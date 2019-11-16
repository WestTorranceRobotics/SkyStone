package org.firstinspires.ftc.teamcode.opmodes.utility;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.ButtonAndEncoderData;
import org.firstinspires.ftc.teamcode.subsystems.ExpansionHub;

@TeleOp(name = "Concept Tester", group = "none")
public class ExperimentalStuff extends OpMode {

    private Robot bot;
    private BNO055IMU backupGyro1;
    private DcMotorEx motorTest;
    private RevTouchSensor touchSensor;

    @Override
    public void init() {
        bot = Robot.getInstance();
        bot.init(hardwareMap);

        backupGyro1 = hardwareMap.get(BNO055IMU.class, "imu1");
        backupGyro1.initialize(new BNO055IMU.Parameters());

        motorTest = hardwareMap.get(DcMotorEx.class, "leftFront");
        touchSensor = hardwareMap.get(RevTouchSensor.class, "foundationTouchSide");
    }

    @Override
    public void init_loop() {
        double volts = bot.secondHub.voltageBattery(ExpansionHub.VoltageUnits.VOLTS);
        telemetry.addData("Battery Voltage", volts);
        telemetry.addData("Firmware Version Control", bot.controlHub.getFirmwareVersion());
        telemetry.addData("Firmware Version Second", bot.secondHub.getFirmwareVersion());
        telemetry.update();
    }

    @Override
    public void start() {
        bot.runtime.reset();
    }

    @Override
    public void loop() {
        ButtonAndEncoderData data = ButtonAndEncoderData.getLatest();
        data.clear();
        data.addHubData(bot.controlHub);
        data.addHubData(bot.secondHub);

        double rev = backupGyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        bot.driveTrain.spinDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("Rev Gyro Z", rev);
        telemetry.addData("Motor Current", bot.controlHub.getMotorCurrentDraw(motorTest, ExpansionHub.CurrentDrawUnits.AMPS));
        telemetry.addData("Motor Voltage", bot.controlHub.getMotorVoltagePercent(motorTest));

        telemetry.addData("Motor Encoder", data.getCurrentPosition(motorTest));
        telemetry.addData("Button Pressed", data.isPressed(touchSensor));

        telemetry.update();
    }

    @Override
    public void stop() {
        bot.close();
    }
}
