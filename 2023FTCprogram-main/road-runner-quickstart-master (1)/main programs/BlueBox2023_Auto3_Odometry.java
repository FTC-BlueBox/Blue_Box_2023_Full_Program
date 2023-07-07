/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * DO NOT DELETE ALYSSA
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.annotation.Target;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.apriltag.AprilTagDetection;
import java.util.ArrayList;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

//@Autonomous(name = "BlueBot2023_Auto3_Odometry", group = "")
public class BlueBox2023_Auto3_Odometry extends LinearOpMode
{
    OpenCvCamera camera;

    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double fx = 622.001;
    double fy = 622.001;
    double cx = 319.803;
    double cy = 241.251;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 8192;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 2.3622;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    // UNITS ARE METERS
    double tagsize = 0.04445;

    AprilTagDetection tagOfInterest = null;

    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    private Servo SERVO_ARM_DELIVER, SERVO_HOLD_DELIVER, SERVO_TURRET;
    private Servo SERVO_LINEARRACK_INTAKE, SERVO_ARM_INTAKE, SERVO_GRIPPER_INTAKE;
    private Servo SERVO_GUIDE;

    private DcMotor MOTOR1;                     // Front Left
    private DcMotor MOTOR2;                     // Front Right
    private DcMotor MOTOR3;                     // Back Left
    private DcMotor MOTOR4;                     // Back Right
    private DcMotor MOTOR_LINEARRACK_DELIVER;
    private DcMotor ENCODER1;
    private DcMotor ENCODER2;

    static final double TICKS_TO_INCHES = 15.3;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    boolean                 aButton, bButton, touched;



    @Override
    public void runOpMode()
    {

        int ParkingPosition = 1;
        String Recognition;


        double SlowSpeedFlag              = -1;     // Robot Movement at Regular Speed
        double SlowSpeedFactor            = 0.3;    // Speed at Slow Mode Movement (at 30% Power)
        double ReduceSpeedFactor          = 0.6;    // Reduction in speed at Fast Mode Movement
        double MotorPower                 = 0;

        double ServoXPosition             = 0.51;   // Turret on the Delivery Linear Rack
        double ServoPositionStep          = 0.003;  // Turret turn step

        double ServoLinearRackIntakeStep = 0.02;    // Servo Step to push or pull the Intake Linear Rack outward or inward

        int     DeliverArmPositionFlag     = 0;     // 0 = Intake, 1 = Deliver
        int     DeliverHoldPositionFlag    = 1;     // 0 = Unhold (Open), 1 = Hold (Close) the Cone
        int     IntakeArmPositionFlag      = 0;     // 0 = Inside, 1 = Outside
        int     IntakeGripperPositionFlag  = 0;     // 0 = Gripper Close, 1 = Gripper

        int     LinearRackDeliverLevel     = 0;     // 0 = Ground, 1 = Low Junction, 2 = Middle Junction, 3 = High Junction
        int     GroundLevel                = 10;
        int     LowLevel                   = 500;
        int     MediumLevel                = 700;
        int     HighLevel                  = 750;


        double  TurretHomePosition            = 0.51;   // 0.51 = Face Front, Higher Number = Turn Clockwise (5T Torque Servo)
        double  ArmDeliverOutPosition         = 0.0;    // 0.0 = Intake Position, Swing Outside to Juntion
        double  ArmDeliverHomePosition        = 0.90;   // 0.90 = Intake Position, Lower Number = Swing back (AGFRC A86BHM Servo)
        double  HoldConePosition              = 0.85;   // 0.95 = Hold the Cone in the holder (Speed Servo)
        double  ReleaseConePosition           = 0.30;   // 0.40 = Release the Cone in the holder (Speed Servo)

        double  ClawOpenPosition              = 0.10;   // 0.10 = Gripper Claw Open
        double  ClawClosePosition             = 0.55;   // 0.55 = Gripper Claw Close
        double  LinearRackIntakeHomePosition   = 0.1;   // 0.1 = Intake Linear Rack inside the robot at initial position
        double  LinearRackIntakeFarOutPosition = 0.7;   // Intake Linear Rack Maximum outreach position
        double  LinearRackIntakePosition       = LinearRackIntakeHomePosition;     // Intake Linear Rack current position

        int     Mode = 1;                           // 1 = Collect Cone at Sub Station, 2 = Collect Cone at Cone Stack Line
        int     Position = 0;                       // 1 = Out, 0 = In
        int     StackPosition = 5;                  // 1 = Lowest Level, 5 = Highest Level
        int GuidePosition = 0;                 //0 = Guide Up, 1 = Guide Down

        boolean LastGamepad1_Y    = false;          // Gamepad1 Button Y is not pressed (Intake Arm)
        boolean CurrentGamepad1_Y = false;          // Gamepad1 Button Y is not pressed (Intake Arm)
        boolean LastGamepad1_B    = false;          // Gamepad1 Button B is not pressed (Intake Cone Gripper)
        boolean CurrentGamepad1_B = false;          // Gamepad1 Button B is not pressed (Intake Cone Gripper)

        boolean LastGamepad2_Y    = false;          // Gamepad2 Button Y is not pressed (Deliver Arm)
        boolean CurrentGamepad2_Y = false;          // Gamepad2 Button Y is not pressed (Deliver Arm)
        boolean LastGamepad2_B    = false;          // Gamepad2 Button B is not pressed (Deliver Cone Holder)
        boolean CurrentGamepad2_B = false;          // Gamepad2 Button B is not pressed (Deliver Cone Holder)
        boolean CurrentGamepad2_A = false;
        boolean LastGamepad2_A = false;





        // Hardware (DC Motor, Servo, Sensor) Definition
        SERVO_ARM_DELIVER       = hardwareMap.get(Servo.class, "SERVO-ARM-DELIVER");
        SERVO_HOLD_DELIVER      = hardwareMap.get(Servo.class, "SERVO-HOLD-DELIVER");
        SERVO_TURRET            = hardwareMap.get(Servo.class, "SERVO-TURRET");
        SERVO_LINEARRACK_INTAKE = hardwareMap.get(Servo.class, "SERVO-LINEARRACK-INTAKE");
        SERVO_ARM_INTAKE        = hardwareMap.get(Servo.class, "SERVO-ARM-INTAKE");
        SERVO_GRIPPER_INTAKE    = hardwareMap.get(Servo.class, "SERVO-GRIPPER-INTAKE");
        SERVO_GUIDE = hardwareMap.get(Servo.class, "SERVO-GUIDE");

        MOTOR1 = hardwareMap.get(DcMotor.class, "MOTOR1");
        MOTOR2 = hardwareMap.get(DcMotor.class, "MOTOR2");
        MOTOR3 = hardwareMap.get(DcMotor.class, "MOTOR3");
        MOTOR4 = hardwareMap.get(DcMotor.class, "MOTOR4");
        MOTOR_LINEARRACK_DELIVER = hardwareMap.get(DcMotor.class, "MOTOR-LINEARRACK-DELIVER");
        ENCODER1 = hardwareMap.get(DcMotor.class, "ENCODER1");
        ENCODER2 = hardwareMap.get(DcMotor.class, "ENCODER2");


        SERVO_TURRET.setPosition(0.51);             // 0.51 = Face Front, Higher Number = Turn Clockwise (5T Torque Servo)
        SERVO_ARM_DELIVER.setPosition(ArmDeliverHomePosition);    // 0.9 = Intake Position, Lower Number = Swing back (AGFRC A86BHM Servo)
        SERVO_HOLD_DELIVER.setPosition(ReleaseConePosition);         // 0.95 = Hold the Cone in the holder

        SERVO_LINEARRACK_INTAKE.setPosition(LinearRackIntakeHomePosition);     // 0.02 = Linear Rack inside the Robot, Higher Number = Linear Rack moves outward (Torque Servo)
        SERVO_GRIPPER_INTAKE.setPosition(ClawOpenPosition );                   // 0 = Gripper Close (don't set to zero), 0.35 = Gripper Open
        SERVO_ARM_INTAKE.setPosition(0.45);                                    // 0 = Intake Gripper swing over to outside of the robot, 0.62 = Inside Robot
        SERVO_GUIDE.setPosition(0.18);

        // Initialization - DC Motors
        MOTOR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MOTOR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MOTOR3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MOTOR4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MOTOR_LINEARRACK_DELIVER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ENCODER1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ENCODER2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MOTOR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR_LINEARRACK_DELIVER.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        ENCODER1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ENCODER2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ENCODER1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ENCODER2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MOTOR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MOTOR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MOTOR3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MOTOR4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path:", "Starting at %7d:%7d", ENCODER2.getCurrentPosition(), MOTOR1.getCurrentPosition());
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/

        /** Wait for the game to begin */
        //telemetry.addData(">", "Press Play to start op mode");

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested() && !opModeIsActive())
        {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        //tagOfInterest.id
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
            //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            telemetry.addData("ENCODER1 ", ENCODER1.getCurrentPosition());
            telemetry.addData("ENCODER2 ", ENCODER2.getCurrentPosition());
            telemetry.update();
            sleep(20);
        }

//Now the robot starts

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if (opModeIsActive())
        {
            MoveXRight(0.3,49.2);
            sleep(100);
            MoveYBack(0.2,0.5);
            sleep(100);
            rotate(0.4, -33);
            SERVO_HOLD_DELIVER.setPosition(HoldConePosition);     // Close and Hold the Cone
            DeliverHoldPositionFlag = 1;
            SERVO_ARM_DELIVER.setPosition(ArmDeliverOutPosition);   // Deliver corn to the junction
            DeliverArmPositionFlag = 0;
            sleep(1000);
            SERVO_HOLD_DELIVER.setPosition(ReleaseConePosition);
            sleep(200);
            SERVO_TURRET.setPosition(TurretHomePosition);           // Turn and ensure the Turret is in the middle position before swinging the Deliver Arm into inside of the robot
            SERVO_ARM_DELIVER.setPosition(ArmDeliverHomePosition);  // Deliver Arm swing into inside of the building
            sleep(500);
            rotate(0.4, 28);
            //end of cone 1

            SERVO_ARM_INTAKE.setPosition(0.17);
            StackPosition = 4;
            IntakeArmPositionFlag = 1;
            sleep(200);
            SERVO_GRIPPER_INTAKE.setPosition(ClawOpenPosition);     // Gripper Claws Open
            IntakeGripperPositionFlag = 1;

            SERVO_LINEARRACK_INTAKE.setPosition(LinearRackIntakeFarOutPosition);
            Position = 1;

            SERVO_HOLD_DELIVER.setPosition(ReleaseConePosition);
            DeliverHoldPositionFlag = 0;
            sleep(700);
            MoveYFront(0.2,7.9);
            sleep(200);

            SERVO_GRIPPER_INTAKE.setPosition(ClawClosePosition);        // Gripper Claw Close
            IntakeGripperPositionFlag = 0;
            sleep(500);
            SERVO_ARM_INTAKE.setPosition(0.45);                         // Intake Outside
            IntakeArmPositionFlag = 0;
            sleep(100);

            SERVO_LINEARRACK_INTAKE.setPosition(LinearRackIntakeHomePosition);
            sleep(500);

            SERVO_GRIPPER_INTAKE.setPosition(ClawOpenPosition);         // Gripper Claw Open
            IntakeGripperPositionFlag = 1;
            sleep(900);
            Position = 0;

            SERVO_HOLD_DELIVER.setPosition(HoldConePosition);
            DeliverHoldPositionFlag = 1;
            sleep(300);

            MoveYBack(0.3,3.75);
            sleep(500);
            rotate(0.3, -31);
            SERVO_HOLD_DELIVER.setPosition(HoldConePosition);     // Close and Hold the Cone
            DeliverHoldPositionFlag = 1;
            SERVO_ARM_DELIVER.setPosition(ArmDeliverOutPosition);   // Deliver cone to the junction
            DeliverArmPositionFlag = 0;
            sleep(1000);
            SERVO_HOLD_DELIVER.setPosition(ReleaseConePosition);
            sleep(200);
            SERVO_TURRET.setPosition(TurretHomePosition);           // Turn and ensure the Turret is in the middle position before swinging the Deliver Arm into inside of the robot
            SERVO_ARM_DELIVER.setPosition(ArmDeliverHomePosition);  // Deliver Arm swing into inside of the building
            sleep(500);
            rotate(0.3, 31);
            //end of cone 2

            SERVO_ARM_INTAKE.setPosition(0.15);
            StackPosition = 4;
            IntakeArmPositionFlag = 1;
            sleep(200);
            SERVO_GRIPPER_INTAKE.setPosition(ClawOpenPosition);     // Gripper Claws Open
            IntakeGripperPositionFlag = 1;

            SERVO_LINEARRACK_INTAKE.setPosition(LinearRackIntakeFarOutPosition);
            Position = 1;

            SERVO_HOLD_DELIVER.setPosition(ReleaseConePosition);
            DeliverHoldPositionFlag = 0;
            sleep(700);
            MoveYFront(0.2,7.15);
            sleep(200);

            SERVO_GRIPPER_INTAKE.setPosition(ClawClosePosition);        // Gripper Claw Close
            IntakeGripperPositionFlag = 0;
            sleep(500);
            SERVO_ARM_INTAKE.setPosition(0.45);                         // Intake Outside
            IntakeArmPositionFlag = 0;
            sleep(100);

            SERVO_LINEARRACK_INTAKE.setPosition(LinearRackIntakeHomePosition);
            sleep(500);

            SERVO_GRIPPER_INTAKE.setPosition(ClawOpenPosition);         // Gripper Claw Open
            IntakeGripperPositionFlag = 1;
            sleep(900);
            Position = 0;

            SERVO_HOLD_DELIVER.setPosition(HoldConePosition);
            DeliverHoldPositionFlag = 1;
            sleep(300);

            MoveYBack(0.3,3.85);
            sleep(500);
            rotate(0.3, -31);
            SERVO_HOLD_DELIVER.setPosition(HoldConePosition);     // Close and Hold the Cone
            DeliverHoldPositionFlag = 1;
            SERVO_ARM_DELIVER.setPosition(ArmDeliverOutPosition);   // Deliver corn to the junction
            DeliverArmPositionFlag = 0;
            sleep(1000);
            SERVO_HOLD_DELIVER.setPosition(ReleaseConePosition);
            sleep(200);
            SERVO_TURRET.setPosition(TurretHomePosition);           // Turn and ensure the Turret is in the middle position before swinging the Deliver Arm into inside of the robot
            SERVO_ARM_DELIVER.setPosition(ArmDeliverHomePosition);  // Deliver Arm swing into inside of the building
            rotate(0.3, 31);
            if(tagOfInterest.id == 0){
                MoveYFront(1,12);
            } else if (tagOfInterest.id == 1) {
                MoveYFront(0.5,2);
            } else if (tagOfInterest.id == 2) {
                MoveYBack(1,11);
            } else if (tagOfInterest == null) {

            }
            telemetry.addData("ENCODER1 ", ENCODER1.getCurrentPosition());
            telemetry.addData("ENCODER2 ", ENCODER2.getCurrentPosition());
            telemetry.update();
        }
    }

    //
//
//
    public void MoveYFront(double speed, double frontInches){
        int newYTargetForward;
        int motorDirection;
        if (opModeIsActive()){
            newYTargetForward = ENCODER2.getCurrentPosition() + (int)(frontInches * COUNTS_PER_INCH);

            while (opModeIsActive() && (ENCODER2.getCurrentPosition() < newYTargetForward)){  //(runtime.seconds() < timeout) &&
                MOTOR2.setPower(speed);
                MOTOR1.setPower(-speed);
                MOTOR3.setPower(-speed);
                MOTOR4.setPower(speed);

                telemetry.addData("Path1", "Running to %7d",  newYTargetForward);
                telemetry.addData("Path2", "Running at %7d", ENCODER2.getCurrentPosition());
                telemetry.update();
            }
            MOTOR1.setPower(0);
            MOTOR3.setPower(0);
            MOTOR2.setPower(0);
            MOTOR4.setPower(0);

            ENCODER2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ENCODER2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }



    public void MoveYBack(double speed, double frontInches){ //double timeout
        int newYTargetBack;
        int motorDirection;
        if (opModeIsActive()){
            newYTargetBack = ENCODER2.getCurrentPosition() + (int)(frontInches * COUNTS_PER_INCH);

            while (opModeIsActive() && (Math.abs(ENCODER2.getCurrentPosition()) < newYTargetBack)){ //(runtime.seconds() < timeout) &&
                MOTOR2.setPower(-speed);
                MOTOR1.setPower(speed);
                MOTOR3.setPower(speed);
                MOTOR4.setPower(-speed);

                telemetry.addData("Path1", "Running to %7d",  -newYTargetBack);
                telemetry.addData("Path2", "Running at %7d", ENCODER2.getCurrentPosition());
                telemetry.update();
            }
            MOTOR1.setPower(0);
            MOTOR3.setPower(0);
            MOTOR2.setPower(0);
            MOTOR4.setPower(0);

            ENCODER2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ENCODER2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void MoveXLeft(double speed, double frontInches){  //double timeout
        int newXTargetLeft;
        if (opModeIsActive()){
            newXTargetLeft = ENCODER1.getCurrentPosition() + (int)(frontInches * COUNTS_PER_INCH);

            while (opModeIsActive() && (ENCODER1.getCurrentPosition() < newXTargetLeft)){ //(runtime.seconds() < timeout) &&
                MOTOR2.setPower(speed);
                MOTOR1.setPower(speed);
                MOTOR3.setPower(-speed);
                MOTOR4.setPower(-speed);

                telemetry.addData("Path1", "Running to %7d",  newXTargetLeft);
                telemetry.addData("Path2", "Running at %7d", ENCODER1.getCurrentPosition());
                telemetry.update();
            }
            MOTOR1.setPower(0);
            MOTOR3.setPower(0);
            MOTOR2.setPower(0);
            MOTOR4.setPower(0);

            ENCODER1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ENCODER1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void MoveXRight(double speed, double frontInches){ //double timeout
        int newXTargetRight;
        int motorDirection;
        if (opModeIsActive()){
            newXTargetRight = ENCODER1.getCurrentPosition() + (int)(frontInches * COUNTS_PER_INCH);

            while (opModeIsActive() &&(Math.abs(ENCODER1.getCurrentPosition()) < newXTargetRight)){  // (runtime.seconds() < timeout) &&
                MOTOR2.setPower(-speed);
                MOTOR1.setPower(-speed);
                MOTOR3.setPower(speed);
                MOTOR4.setPower(speed);

                telemetry.addData("Path1", "Running to %7d",  -newXTargetRight);
                telemetry.addData("Path2", "Running at %7d", ENCODER1.getCurrentPosition());
                telemetry.update();
            }
            MOTOR1.setPower(0);
            MOTOR3.setPower(0);
            MOTOR2.setPower(0);
            MOTOR4.setPower(0);

            ENCODER1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ENCODER1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
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
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(double power,int degrees)
    {
        double  FinalPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            FinalPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            FinalPower = power;
        }
        else return;

        // set power to rotate.
        MOTOR1.setPower(FinalPower);
        MOTOR3.setPower(FinalPower);
        MOTOR2.setPower(FinalPower);
        MOTOR4.setPower(FinalPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {
            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
            }

        // turn the motors off.
        MOTOR1.setPower(0);
        MOTOR3.setPower(0);
        MOTOR2.setPower(0);
        MOTOR4.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
    //
//
//
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}






