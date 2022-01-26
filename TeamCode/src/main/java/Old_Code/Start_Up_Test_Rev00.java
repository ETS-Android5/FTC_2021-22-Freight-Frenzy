/* Code Generated by Team 16595, based off of 2019-2020 SkyStone Robot Configuration
Basic Test of System to ensure all actuators work on robot
    1) Move robot side to side
    2) lower and lift intake
    3) rotate intake
Rev00 - Base Code - September 17,2020
*/

package Old_Code; // Overall Package to be Used by Code

//Libraries Used in code
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// Start of Opmode, Name shows up on phone, group tells you what type of program is running
// Class name must match name of Code
//@Autonomous(name="Start_Up_Test", group="")
public class Start_Up_Test_Rev00 extends LinearOpMode {

    // Global Variables

    @Override
    public void runOpMode() {

        //Local Variables
        ElapsedTime runtime = new ElapsedTime();
        DcMotor leftbackDrive = null;
        DcMotor rightbackDrive = null;
        DcMotor leftfrontDrive = null;
        DcMotor rightfrontDrive = null;
        DcMotorSimple rightRotate = null;
        DcMotorSimple leftRotate = null;
        DcMotorSimple RotateIntake = null;
        ElapsedTime timmer = new ElapsedTime();
        BNO055IMU imu;
        Orientation lastAngles = new Orientation();

        // Set Motors for drive and intake rotation, Must match configured names in app
        leftbackDrive = hardwareMap.get(DcMotor.class, "leftbackdrive");
        rightbackDrive = hardwareMap.get(DcMotor.class, "rightbackdrive");
        leftfrontDrive = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        rightfrontDrive = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        RotateIntake = hardwareMap.get(DcMotorSimple.class, "RotateIntake");
        rightRotate = hardwareMap.get(DcMotorSimple.class, "leftIntake");
        leftRotate = hardwareMap.get(DcMotorSimple.class, "rightIntake");

        // Set motor direction so moving proper direction
        leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightbackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightfrontDrive.setDirection(DcMotor.Direction.FORWARD);
        RotateIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRotate.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRotate.setDirection(DcMotorSimple.Direction.REVERSE);

        // Setup IMU to use as Gyro for orientation of Robot
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        // Telling Operator IMU started Successfully
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // wait for start button to be  pressed by operator
        waitForStart();

        // Inform operator robot program is running
        telemetry.addData("Mode", "running");
        telemetry.update();

        //Make robot turn left
        timmer.reset();
        while (timmer.milliseconds() < (500))
        {
            leftbackDrive.setPower(+0.2);
            rightbackDrive.setPower(-0.2);
            leftfrontDrive.setPower(+0.2);
            rightfrontDrive.setPower(-0.2);
        }

        //Make robot turn right
        timmer.reset();
        while (timmer.milliseconds() < (500))
        {
            leftbackDrive.setPower(-0.2);
            rightbackDrive.setPower(+0.2);
            leftfrontDrive.setPower(-0.2);
            rightfrontDrive.setPower(+0.2);
        }

        //Stop robot movement motors
        leftbackDrive.setPower(0);
        rightbackDrive.setPower(0);
        leftfrontDrive.setPower(0);
        rightfrontDrive.setPower(0);

        //place down robot intake
        RotateIntake.setPower(-0.3);
        sleep(750);
        RotateIntake.setPower(0);
        sleep(1000);

        //Rotate intake wheels on then off
        leftRotate.setPower(1.0);
        rightRotate.setPower(1.0);
        sleep(750);
        leftRotate.setPower(0);
        rightRotate.setPower(0);

        //lift robot intake up
        RotateIntake.setPower(0.6);
        sleep(500);
        RotateIntake.setPower(0);
        sleep(1000);

    }
}