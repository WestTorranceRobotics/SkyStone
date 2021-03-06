package org.firstinspires.ftc.teamcode.opmodes.utility.calibration;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Map;

@TeleOp(name = "ServoTester", group = "none")
public class ServoTester extends OpMode {

    private Object[] servos;
    private int index;
    private int pressed;
    private double[] positions;

    @Override
    public void init() {
        servos = hardwareMap.servo.entrySet().toArray();
        telemetry.addData("Number of servos found", servos.length);
        telemetry.addLine("Dpad right to move forward, left to move in reverse.");
        telemetry.addLine("Dpad up and down to switch servos.");
        telemetry.update();
        if (servos.length == 0) {
            throw new IllegalArgumentException("Needs at least one servo plugged in");
        }
        index = 0;
        positions = new double[servos.length];
        pressed = 0;
    }

    @Override
    public void loop() {
        Servo arm = (Servo) ((Map.Entry) servos[index]).getValue();
        String name = (String) ((Map.Entry) servos[index]).getKey();
        if (gamepad1.dpad_right && !gamepad1.dpad_left && positions[index] < 1.0) {
            positions[index] += 0.001;
        }
        if (gamepad1.dpad_left && !gamepad1.dpad_right && positions[index] > 0.0) {
            positions[index] -= 0.001;
        }
        arm.setPosition(positions[index]);
        telemetry.addData("Servo", name);
        telemetry.addData("Position", positions[index]);
        telemetry.update();
        if (gamepad1.dpad_up && !gamepad1.dpad_down && pressed != 1) {
            pressed = 1;
            index++;
        }
        if (gamepad1.dpad_down && !gamepad1.dpad_up && pressed != -1) {
            pressed = -1;
            index += servos.length - 1;
        }
        if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
            pressed = 0;
        }
        index %= servos.length;
    }
}
