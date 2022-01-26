package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Auto_Spin_Park_Blue", group="")
public class Auto_Spin_Park_Blue extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timmer = new ElapsedTime();
    DcMotor LB_Drive;
    DcMotor RB_Drive;
    DcMotor LF_Drive;
    DcMotor RF_Drive;
    DcMotor Intake;
    DcMotor Spinner;
    DcMotor elevator;
    Servo elevator_tilt;
    DistanceSensor sensorRange;
    ColorSensor ColourSensor_1;
    double Colour_Blue = 0;
    double Colour_Green = 0;
    double Colour_Red = 0;
    double Colour_Measure_1 = 0;
    double Distance_found = 0;
    double duck_distance = 6;
    double level = 0;
    long straif_time = 0;
    double power = -0.3;
    double spinner_power = -0.25;
    double elevator_power = 0;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;

        ColourSensor_1 = hardwareMap.get(ColorSensor.class, "coloursensor_1");
        ColourSensor_1.enableLed(true);

        // Set Motors for drive and intake rotation
        LB_Drive = hardwareMap.get(DcMotor.class, "LB_Drive");
        RB_Drive = hardwareMap.get(DcMotor.class, "RB_Drive");
        LF_Drive = hardwareMap.get(DcMotor.class, "LF_Drive");
        RF_Drive = hardwareMap.get(DcMotor.class, "RF_Drive");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Spinner = hardwareMap.get(DcMotor.class, "Spinner");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        elevator_tilt = hardwareMap.get(Servo.class, "elevator_tilt");

        // Set motor direction so moving proper direction
        LB_Drive.setDirection(DcMotor.Direction.REVERSE);
        RB_Drive.setDirection(DcMotor.Direction.FORWARD);
        LF_Drive.setDirection(DcMotor.Direction.REVERSE);
        RF_Drive.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Spinner.setDirection(DcMotor.Direction.FORWARD);
        elevator_tilt.setPosition(0.3);

        // Setup Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        // wait for start button.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();
        //colour sensor test
        timmer.reset();

        // Move forward towards duck and stop
        //800 orginal time
        timmer.reset();
        while(timmer.milliseconds() < 100) {
            correction = checkDirection();
            LB_Drive.setPower(power + correction);
            RB_Drive.setPower(power - correction);
            LF_Drive.setPower(power + correction);
            RF_Drive.setPower(power - correction);
        }
        LB_Drive.setPower(0);
        RB_Drive.setPower(0);
        LF_Drive.setPower(0);
        RF_Drive.setPower(0);

            straif_time = (2800);

        // straif to spinner
        timmer.reset();
        while(timmer.milliseconds() < straif_time) {
            correction = checkDirection();
            LB_Drive.setPower((power * -1) - correction);
            RB_Drive.setPower(power + correction);
            LF_Drive.setPower(power + correction);
            RF_Drive.setPower((power * -1) - correction);
        }
        LB_Drive.setPower(0);
        RB_Drive.setPower(0);
        LF_Drive.setPower(0);
        RF_Drive.setPower(0);

        sleep(200);

        Spinner.setPower(spinner_power);

        timmer.reset();
        while(timmer.milliseconds() < 3000) {
            Spinner.setPower(spinner_power);
        }
        Spinner.setPower(0);

        //straif towards park
        timmer.reset();
        while(timmer.milliseconds() < 2400) {
            correction = checkDirection();
            LB_Drive.setPower(power + correction);
            RB_Drive.setPower((power * -1) - correction);
            LF_Drive.setPower((power * -1) + correction);
            RF_Drive.setPower(power - correction);
        }

        // spin 90 to fit through gap
        timmer.reset();
        while(timmer.milliseconds() < 1000) {
            LB_Drive.setPower((power*-1));
            RB_Drive.setPower((power*1));
            LF_Drive.setPower((power*-1));
            RF_Drive.setPower((power*1));
        }

        //Drive to park
        timmer.reset();
        while(timmer.milliseconds() < 2500) {
            correction = checkDirection_90();
            correction=0;
            LB_Drive.setPower((power * 1) + correction);
            RB_Drive.setPower((power * -1) - correction);
            LF_Drive.setPower((power * -1) + correction);
            RF_Drive.setPower((power * 1) - correction);
        }
        //Park
        LB_Drive.setPower(0);
        RB_Drive.setPower(0);
        LF_Drive.setPower(0);
        RF_Drive.setPower(0);

        timmer.reset();
        while(timmer.milliseconds() < 5000) {
            correction = checkDirection_90();
            // correction=0;
            LB_Drive.setPower(power + correction);
            RB_Drive.setPower(power - correction);
            LF_Drive.setPower(power + correction);
            RF_Drive.setPower(power - correction);
        }
        LB_Drive.setPower(0);
        RB_Drive.setPower(0);
        LF_Drive.setPower(0);
        RF_Drive.setPower(0);

    }
    // Function to get angle change from gyro
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .02;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private double checkDirection_180() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .02;

        angle = getAngle_180();

        if (angle == 180)
            correction = 0;             // no adjustment.
        else
            correction = 180-angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private double getAngle_180() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = (angles.firstAngle) - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection_90() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .02;

        angle = getAngle_90();

        if (angle == 90)
            correction = 0;             // no adjustment.
        else if (angle > 90)
            correction = angle - 90;        // reverse sign of angle for correction.
        else
            correction = 90 - angle;

        correction = correction * gain;

        return correction;
    }

    private double getAngle_90() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = (angles.firstAngle) - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}