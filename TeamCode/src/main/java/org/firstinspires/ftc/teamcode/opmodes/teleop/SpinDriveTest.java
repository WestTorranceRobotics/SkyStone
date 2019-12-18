package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@TeleOp(name = "test drive", group = "none")
public class SpinDriveTest extends OpMode {

    private DriveTrain driveTrain;
    private boolean initialized;

    @Override
    public void init() {
        initialized = false;
        new Thread(() -> {
            driveTrain = DriveTrain.getInstance();
            driveTrain.init(hardwareMap);
            driveTrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            initialized = true;
        }).start();
    }

    @Override
    public void loop() {
        if (!initialized) {
            return;
        }
        driveTrain.spinDrive(0, 0.3, 0);
    }
}
