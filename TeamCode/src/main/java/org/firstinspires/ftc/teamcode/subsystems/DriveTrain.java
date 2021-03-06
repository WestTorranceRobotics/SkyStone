package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.westtorrancerobotics.lib.ftc.ButtonAndEncoderData;
import org.westtorrancerobotics.lib.ftc.MecanumDriveImpl;
import org.westtorrancerobotics.lib.ftc.TestableGyro;
import org.westtorrancerobotics.lib.spline.geom.Angle;
import org.westtorrancerobotics.lib.spline.geom.Location;
import org.westtorrancerobotics.lib.hardware.drive.MecanumController;
import org.westtorrancerobotics.lib.hardware.drive.MecanumDrive;

public class DriveTrain {

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private MecanumController mecanumController;

    private TestableGyro imus;

    private ColorSensor lineSpotter;
    private static final int RED_THRESHOLD  = 1800;
    private static final int BLUE_THRESHOLD = 2000;

    private Odometer odometer;

    private static DriveTrain instance = null;

    public static synchronized DriveTrain getInstance() {
        return instance != null ? instance : (instance = new DriveTrain());
    }

    private DriveTrain() {}

    public void init(HardwareMap hardwareMap) {
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lineSpotter = hardwareMap.get(ColorSensor.class, "lineColor");

        BNO055IMU backupGyro1 = hardwareMap.get(BNO055IMU.class, "imu1");
        BNO055IMU backupGyro2 = hardwareMap.get(BNO055IMU.class, "imu2");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.calibrationDataFile = null;//gyro 1 config when made
        backupGyro1.initialize(params);
        params.calibrationDataFile = null;//gyro 2 config when made
        backupGyro2.initialize(params);

        MecanumDrive wheels = new MecanumDriveImpl(leftFront, leftBack, rightFront, rightBack);
        mecanumController = new MecanumController(wheels);

        odometer = Odometer.getInstance();
        odometer.init(hardwareMap);
    }

    public void setMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        leftFront.setZeroPowerBehavior(mode);
        leftBack.setZeroPowerBehavior(mode);
        rightFront.setZeroPowerBehavior(mode);
        rightBack.setZeroPowerBehavior(mode);
    }

    public void spinDrive(double x, double y, double turn) {
        mecanumController.spinDrive(x, y, turn, MecanumDrive.TranslTurnMethod.EQUAL_SPEED_RATIOS);
    }

    public void spinDrive(Angle ang, double speed, double turn) {
        mecanumController.spinDrive(ang, speed, turn, MecanumDrive.TranslTurnMethod.EQUAL_SPEED_RATIOS);
    }

    public void updateLocation() {
        odometer.update();
    }

    public void setLocation(Location l) {
        odometer.myLocation = l;
    }

    public Location getLocation() {
        return odometer.myLocation;
    }

    public boolean onRedLine() {
        return lineSpotter.red() > RED_THRESHOLD;
    }

    public boolean onBlueLine() {
        return lineSpotter.blue() > BLUE_THRESHOLD;
    }

    public double gyro() {
        return imus.getHeading().getValue(Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING);
    }

    private static class Odometer {
        private Location myLocation;
        private Wheel leftY;
        private Wheel rightY;
        private Wheel x;

        private final double TICKS_TO_INCHES = (2 * Math.PI) / 4096;

        private static Odometer instance = null;

        public static synchronized Odometer getInstance() {
            return instance != null ? instance : (instance = new Odometer());
        }

        private Odometer() {}

        public void init(HardwareMap hardwareMap) {
            leftY = new Wheel(new Location(-6.815,1.645,
                    new Angle(180, Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING)),
                    hardwareMap.get(DcMotorEx.class, "intakeLeft/odometerLeftY"));
            rightY = new Wheel(new Location(6.815,1.645,
                    new Angle(0, Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING)),
                    hardwareMap.get(DcMotorEx.class, "intakeRight/odometerRightY"));
            x = new Wheel(new Location(7.087,-1.980,
                    new Angle(-90, Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING)),
                    hardwareMap.get(DcMotorEx.class, "liftRight/odometerX"));
        }

        public void update() {
            double dly = leftY.getMovedInches();
            double dry = rightY.getMovedInches();
            double dx = x.getMovedInches();
            if (isZero(dly + dry)) {
                leftY.lastEnc += dly;
                rightY.lastEnc += dry;
                x.lastEnc += dx;
                Angle thetaY = myLocation.direction;
                Angle thetaX = Angle.add(thetaY, Angle.EAST, Angle.AngleOrientation.COMPASS_HEADING);
                double fieldDx = dx * thetaX.getX() + dry * thetaY.getX();
                double fieldDy = dx * thetaX.getY() + dry * thetaY.getY();
                myLocation.translate(fieldDx, fieldDy);
                return;
            }
            double[] solved = solve(new double[][]{
                    {
                            Math.cos(leftY.fetchDirection()),
                            -Math.sin(leftY.fetchDirection()),
                            -dly,
                            Math.cos(leftY.fetchDirection()) * leftY.relativeLocation.x
                                    - Math.sin(leftY.fetchDirection()) * leftY.relativeLocation.y
                    },
                    {
                            Math.cos(rightY.fetchDirection()),
                            -Math.sin(rightY.fetchDirection()),
                            -dry,
                            Math.cos(rightY.fetchDirection()) * rightY.relativeLocation.x
                                    - Math.sin(rightY.fetchDirection()) * rightY.relativeLocation.y
                    },
                    {
                            Math.cos(x.fetchDirection()),
                            -Math.sin(x.fetchDirection()),
                            -dx,
                            Math.cos(x.fetchDirection()) * x.relativeLocation.x
                                    - Math.sin(x.fetchDirection()) * x.relativeLocation.y
                    }
            });
            double rotCenterRelX = solved[0];
            double rotCenterRelY = solved[1];
            double rotRadCw = 1 / solved[2];
            double convT = myLocation.direction.getValue(Angle.AngleUnit.RADIANS, Angle.AngleOrientation.UNIT_CIRCLE);
            myLocation.direction = new Angle(
                    myLocation.direction.getValue(Angle.AngleUnit.RADIANS, Angle.AngleOrientation.COMPASS_HEADING) + rotRadCw,
                    Angle.AngleUnit.RADIANS,
                    Angle.AngleOrientation.COMPASS_HEADING
            );
            double hw = myLocation.x;
            double kw = myLocation.y;
            double hr = hw + Math.sin(convT) * rotCenterRelX + Math.cos(convT) * rotCenterRelY;
            double kr = kw - Math.cos(convT) * rotCenterRelX + Math.sin(convT) * rotCenterRelY;
            double r = Math.hypot(hw-hr, kw-kr);
            double theta = -rotRadCw + Math.atan2(kw-kr, hw-hr);
            myLocation.setLocation(hr + r * Math.cos(theta), kr + r * Math.sin(theta));
        }

        private class Wheel {
            private final Location relativeLocation;
            private final double direction;
            private final DcMotorEx encoder;
            private long lastEnc;

            private Wheel (Location relativeLocation, DcMotorEx encoder) {
                this.relativeLocation = relativeLocation;
                this.encoder = encoder;
                lastEnc = ButtonAndEncoderData.getLatest().getCurrentPosition(encoder);
                direction = relativeLocation.direction.getValue(Angle.AngleUnit.RADIANS, Angle.AngleOrientation.COMPASS_HEADING);
            }

            double getMovedInches() {
                long dx = ButtonAndEncoderData.getLatest().getCurrentPosition(encoder) - lastEnc;
                lastEnc += dx;
                return dx * TICKS_TO_INCHES;
            }

            double fetchDirection() {
                return direction;
            }
        }

        private double[] solve(double[][] augmentedMatrix) {
            double[][] matrix = new double[augmentedMatrix.length][augmentedMatrix.length + 1];
            for (int i = 0; i < matrix.length; i++) {
                System.arraycopy(augmentedMatrix[i], 0, matrix[i], 0, matrix[i].length);
            }
            for (int i = 0; i < matrix.length; i++) {
                for (int j = i; j < matrix.length; j++) {
                    if (!isZero(matrix[j][i])) {
                        swapRows(matrix, i, j);
                        break;
                    }
                }
                scale(matrix, i, 1 / matrix[i][i]);
                matrix[i][i] = 1;
                for (int j = i + 1; j < matrix.length; j++) {
                    if (isZero(matrix[j][i])) {
                        continue;
                    }
                    scale(matrix, j, -1 / matrix[j][i]);
                    addRow(matrix, i, j);
                    matrix[j][i] = 0;
                }
            }
            for (int i = matrix.length - 1; i >= 0; i--) {
                for (int j = i - 1; j >= 0; j--) {
                    if (isZero(matrix[j][i])) {
                        continue;
                    }
                    double sc = -matrix[j][i];
                    scale(matrix, i, sc);
                    addRow(matrix, i, j);
                    scale(matrix, i, 1/sc);
                }
            }
            double[] solutions = new double[matrix.length];
            for (int k = 0; k < matrix.length; k++) {
                solutions[k] = matrix[k][matrix.length];
            }
            return solutions;
        }

        private boolean isZero(double number) {
            return Math.abs(number) < 1e-9;
        }

        private void swapRows(double[][] matrix, int row1, int row2) {
            double[] rowa = matrix[row2];
            matrix[row2] = matrix[row1];
            matrix[row1] = rowa;
        }

        private void scale(double[][] matrix, int row, double constant) {
            for (int i = 0; i < matrix[row].length; i++) {
                matrix[row][i] *= constant;
            }
        }

        private void addRow(double[][] matrix, int addend, int rowChanged) {
            for (int i = 0; i < matrix[rowChanged].length; i++) {
                matrix[rowChanged][i] += matrix[addend][i];
            }
        }
    }
}
