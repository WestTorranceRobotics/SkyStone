package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.westtorrancerobotics.lib.ftc.ButtonAndEncoderData;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class Robot implements AutoCloseable {

    public final ElapsedTime runtime;

    public final DriveTrain driveTrain;
    public final FoundationGrabber foundationGrabber;
    public final Lift lift;
    public final StoneManipulator stoneManipulator;
    public final Camera camera;
    public ExpansionHub controlHub;
    public ExpansionHub secondHub;

    private static Robot instance = null;

    public static synchronized Robot getInstance() {
        return instance != null ? instance : (instance = new Robot());
    }

    private Robot() {

        runtime = new ElapsedTime();

        driveTrain = DriveTrain.getInstance();
        foundationGrabber = FoundationGrabber.getInstance();
        lift = Lift.getInstance();
        stoneManipulator = StoneManipulator.getInstance();
        camera = Camera.getInstance();

    }

    public void init(HardwareMap hardwareMap) {
        controlHub = ExpansionHub.getAvailableHubs(hardwareMap).get("Control Hub");
        secondHub = ExpansionHub.getAvailableHubs(hardwareMap).get("Expansion Hub");

        ButtonAndEncoderData data = ButtonAndEncoderData.getLatest();
        data.clear();
        data.addHubData(controlHub);
        data.addHubData(secondHub);

        driveTrain.init(hardwareMap);
        foundationGrabber.init(hardwareMap);
        lift.init(hardwareMap);
        stoneManipulator.init(hardwareMap);
//        camera.init(hardwareMap);
    }

    @Override
    public void close() {
//        camera.stop();
    }
}
