//===============================================================================================================
// FTC Blue Box #11780
// Alyssa Wan, Hannah Robinson, Izzy Daniels, Chloe Darcy, Anna Sun,
// Gauri Valiyodiyil, Sree Krothapalli, Juliana Troisi, Asritha Sistla,Sasha Machavarapu
// 2022-2023 PowerPlay
// TeleOp Program: BlueBot2023_TeleOp3.java
// Date Created: October 14, 2022
// Date Modified: February 14, 2023 (Implement the turret turn left or right on Cone Cyclying sequence)
//===============================================================================================================

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.logging.Level;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "BlueBot2023_TeleOp3", group = "")
public class BlueBot2023_TeleOp3 extends LinearOpMode
{
    // TeleOp Configuration
    // int ALLIANCE_COLOR = 2;                 // ALLIANCE_COLOR (1=Blue, 2=Red)
    String ALLIANCE_COLOR = "Press X or B";

    private DigitalChannel LED_FRONT_GREEN, LED_FRONT_RED;
    private DigitalChannel LED_RIGHT_GREEN, LED_LEFT_RED;

    private Servo SERVO_ARM_DELIVER, SERVO_HOLD_DELIVER, SERVO_TURRET;
    private Servo SERVO_LINEARRACK_INTAKE, SERVO_ARM_INTAKE, SERVO_GRIPPER_INTAKE;
    private Servo SERVO_GUIDE;

    private DcMotor MOTOR1;                     // Front Left
    private DcMotor MOTOR2;                     // Front Right
    private DcMotor MOTOR3;                     // Back Left
    private DcMotor MOTOR4;                     // Back Right
    private DcMotor MOTOR_LINEARRACK_DELIVER;

    //
    // This function is executed when this Op Mode is selected from the Driver Station.
    //
    @Override
    public void runOpMode()
    {
        // Initialization - Variables
        double  RobotFrontFlag         = 1;             // 1 = Intake is the front, -1 =  Deliver (Back) is the front
        double  SlowSpeedFlag          = -1;            // Robot Movement at Regular Speed
        double  SlowSpeedFactor        = 0.3;           // Speed at Slow Mode Movement (at 30% Power)
        double  ReduceSpeedFactor      = 0.6;           // Reduction in speed at Fast Mode Movement
        double  MotorPower             = 0;             // Motor Power

        double  ServoXPosition         = 0.51;          // Turret on the Delivery Linear Rack
        double  ServoPositionStep      = 0.002;         // Turret turn step
        double  ServoYPosition;

        double  LinearRackIntakeHomePosition   = 0.1;                           // 0.1 = Intake Linear Rack inside the robot at initial position
        double  LinearRackIntakeFarOutPosition = 0.7;                           // Intake Linear Rack Maximum outreach position
        double  LinearRackIntakePosition       = LinearRackIntakeHomePosition;  // Intake Linear Rack current position
        double  ServoLinearRackIntakeStep      = 0.01;                          // Servo Step to push or pull the Intake Linear Rack outward or inward

        int     LinearRackDeliverLevel        = 0;      // 0 = Ground, 1 = Low Junction, 2 = Middle Junction, 3 = High Junction
        int     GroundLevel                   = 10;     // Ground Junction Level go to position
        int     HighLevel                     = 780;    // High Junction Level go to position

        double  TurretHomePosition            = 0.51;   // 0.51 = Face Front, Higher Number = Turn Clockwise (5T Torque Servo)
        double  TurretRightPosition           = 0.47;   // 0.47 = Turn Right on Cone Cycling
        double  TurretLeftPosition            = 0.54;   // 0.54 = Turn Right on Cone Cycling

        double  ArmDeliverOutPosition         = 0.0;    // 0.00 = Intake Position, Swing Outside to Juntion
        double  ArmDeliverHomePosition        = 0.90;   // 0.90 = Intake Position, Lower Number = Swing back (AGFRC A86BHM Servo)

        double  HoldConePosition              = 0.85;   // 0.85 = Hold the Cone in the holder (Speed Servo)
        double  ReleaseConePosition           = 0.30;   // 0.30 = Release the Cone in the holder (Speed Servo)


        double  ArmIntakeHomePosition         = 0.46;   // 0.45 Intake inside the Robot
        double  ClawOpenPosition              = 0.10;   // 0.10 = Gripper Claw Open
        double  ClawClosePosition             = 0.55;   // 0.55 = Gripper Claw Close


        int     Mode            = 1;                // 1 = Collect Cone at Sub Station, 2 = Collect Cone at Cone Stack Line
        int     CyclePosition   = 1;                // 1 = Right High junction, 2 = Left High Junction, 0 = Center High Junction
        int     Position        = 0;                // 1 = Out, 0 = In
        int     StackPosition   = 5;                // 1 = Lowest Level, 5 = Highest Level
        int     GuidePosition   = 0;                // 0 = Guide Up, 1 = Guide DownF
        int     OuttakePosition = 0;                // 0 = Down, 1 = Up

        int     DeliverArmPositionFlag     = 1;     // 0 = Deliver (Back), 1 = Intake (Front)
        int     DeliverHoldPositionFlag    = 0;     // 0 = Unhold (Open), 1 = Hold (Close) the Cone
        int     IntakeArmPositionFlag      = 0;     // 0 = Inside, 1 = Outside
        int     IntakeGripperPositionFlag  = 0;     // 0 = Gripper Close, 1 = Gripper

        boolean LastGamepad1_X    = false;          // Gamepad1 Button X is not pressed (Toggle Slow Speed)
        boolean CurrentGamepad1_X = false;          // Gamepad1 Button X is not pressed (Toggle Slow Speed)
        boolean LastGamepad1_Y    = false;          // Gamepad1 Button Y is not pressed (Intake Arm)
        boolean CurrentGamepad1_Y = false;          // Gamepad1 Button Y is not pressed (Intake Arm)
        boolean LastGamepad1_B    = false;          // Gamepad1 Button B is not pressed (Intake Cone Gripper)
        boolean CurrentGamepad1_B = false;          // Gamepad1 Button B is not pressed (Intake Cone Gripper)
        boolean LastGamepad1_A    = false;          // Gamepad1 Button A is not pressed (Toggle the robot Front)
        boolean CurrentGamepad1_A = false;          // Gamepad1 Button A is not pressed (Toggle the robot Front)

        boolean LastGamepad2_X              = false;    // Gamepad2 Button X is not pressed (Guide Arm)
        boolean CurrentGamepad2_X           = false;    // Gamepad2 Button X is not pressed (Guide Arm)
        boolean LastGamepad2_Y              = false;    // Gamepad2 Button Y is not pressed (Deliver Arm)
        boolean CurrentGamepad2_Y           = false;    // Gamepad2 Button Y is not pressed (Deliver Arm)
        boolean LastGamepad2_B              = false;    // Gamepad2 Button B is not pressed (Deliver Cone Holder)
        boolean CurrentGamepad2_B           = false;    // Gamepad2 Button B is not pressed (Deliver Cone Holder)
        boolean LastGamepad2_A              = false;    // Gamepad2 Button A is not pressed (Linear Rack at Ground Level)
        boolean CurrentGamepad2_A           = false;    // Gamepad2 Button A is not pressed (Linear Rack at Ground Level)

        // Digital Definition (LED)
        LED_FRONT_GREEN = hardwareMap.get(DigitalChannel.class, "LED-FRONT-GREEN");
        LED_FRONT_RED   = hardwareMap.get(DigitalChannel.class, "LED-FRONT-RED");
        LED_FRONT_GREEN.setMode(DigitalChannel.Mode.OUTPUT);
        LED_FRONT_RED.setMode(DigitalChannel.Mode.OUTPUT);
        LED_FRONT_GREEN.setState(false);                       // Turn OFF Green LED
        LED_FRONT_RED.setState(true);                          // Turn ON Red LED (Intake is the Robot Front)

        LED_RIGHT_GREEN = hardwareMap.get(DigitalChannel.class, "LED-RIGHT-GREEN");
        LED_LEFT_RED    = hardwareMap.get(DigitalChannel.class, "LED-LEFT-RED");
        LED_RIGHT_GREEN.setMode(DigitalChannel.Mode.OUTPUT);
        LED_LEFT_RED.setMode(DigitalChannel.Mode.OUTPUT);
        LED_RIGHT_GREEN.setState(false);                        // Turn OFF Green LED
        LED_LEFT_RED.setState(true);                          // Turn ON Red LED (Intake is the Robot Front)

        // Hardware (DC Motor, Servo, Sensor) Definition
        SERVO_ARM_DELIVER       = hardwareMap.get(Servo.class, "SERVO-ARM-DELIVER");
        SERVO_HOLD_DELIVER      = hardwareMap.get(Servo.class, "SERVO-HOLD-DELIVER");
        SERVO_TURRET            = hardwareMap.get(Servo.class, "SERVO-TURRET");
        SERVO_LINEARRACK_INTAKE = hardwareMap.get(Servo.class, "SERVO-LINEARRACK-INTAKE");
        SERVO_ARM_INTAKE        = hardwareMap.get(Servo.class, "SERVO-ARM-INTAKE");
        SERVO_GRIPPER_INTAKE    = hardwareMap.get(Servo.class, "SERVO-GRIPPER-INTAKE");
        SERVO_GUIDE             = hardwareMap.get(Servo.class, "SERVO-GUIDE");

        MOTOR1 = hardwareMap.get(DcMotor.class, "MOTOR1");
        MOTOR2 = hardwareMap.get(DcMotor.class, "MOTOR2");
        MOTOR3 = hardwareMap.get(DcMotor.class, "MOTOR3");
        MOTOR4 = hardwareMap.get(DcMotor.class, "MOTOR4");
        MOTOR_LINEARRACK_DELIVER = hardwareMap.get(DcMotor.class, "MOTOR-LINEARRACK-DELIVER");

        // Initialization - Servo Motors
        SERVO_TURRET.setPosition(TurretHomePosition);                         // 0.51 = Face Front, Higher Number = Turn Clockwise (5T Torque Servo)
        SERVO_ARM_DELIVER.setPosition(ArmDeliverHomePosition);                // 0.9 = Intake Position, Lower Number = Swing back (AGFRC A86BHM Servo)
        SERVO_HOLD_DELIVER.setPosition(ReleaseConePosition);                  // 0.85 = Hold the Cone in the holder

        SERVO_LINEARRACK_INTAKE.setPosition(LinearRackIntakeHomePosition);     // 0.02 = Linear Rack inside the Robot, Higher Number = Linear Rack moves outward (Torque Servo)
        SERVO_GRIPPER_INTAKE.setPosition(ClawOpenPosition );                   // 0 = Gripper Close (don't set to zero), 0.35 = Gripper Open
        SERVO_ARM_INTAKE.setPosition(ArmIntakeHomePosition);                   // 0 = Intake Gripper swing over to outside of the robot, 0.45 = Inside Robot
        SERVO_GUIDE.setPosition(0.18);                                         // 0.18 = Guideline upright

        // Initialization - DC Motors
        MOTOR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_LINEARRACK_DELIVER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MOTOR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR_LINEARRACK_DELIVER.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(!opModeIsActive())
        {
            // Alliance Color (X=Blue, B=Red)
            if      (gamepad1.x || gamepad2.x)  ALLIANCE_COLOR = "BLUE";
            else if (gamepad1.b || gamepad2.b)  ALLIANCE_COLOR = "RED";

            telemetry.addData("Alliance", ALLIANCE_COLOR);
            telemetry.update();

            //waitForStart();                           // Wait for the Play Button press
        }

        if (opModeIsActive())
        {
            MOTOR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MOTOR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MOTOR3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MOTOR4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MOTOR_LINEARRACK_DELIVER.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            // ===============================================================================================================
            // TeleOp Main Loop
            // ===============================================================================================================
            while (opModeIsActive())
            {
                CurrentGamepad1_X = gamepad1.x;             // Toggle Robot Motor Speed
                CurrentGamepad1_Y = gamepad1.y;             // Toggle Intake Arm In or Out
                CurrentGamepad1_B = gamepad1.b;             // Toggle Intake Gripper Close or Open
                CurrentGamepad1_A = gamepad1.a;             // Toggle the front of the robot
                CurrentGamepad2_X = gamepad2.x;             // Toggle the pole guide Up or Down
                CurrentGamepad2_Y = gamepad2.y;             // Swing the Deliver Arm In or Out
                CurrentGamepad2_B = gamepad2.b;             // Toggle the Hold or Release the cone
                CurrentGamepad2_A = gamepad2.a;             // Move the Vertical Linear Rack Up or Down


                // ===============================================================================================================
                // To toggle the Robot's movement speed between regular or slow speed
                // To toggle the Robot's Left/Right
                // ===============================================================================================================
                if (CurrentGamepad1_X && !LastGamepad1_X)   SlowSpeedFlag  = -SlowSpeedFlag;
                if (CurrentGamepad1_A && !LastGamepad1_A)   RobotFrontFlag = -RobotFrontFlag;

                if (RobotFrontFlag == 1)
                {
                    LED_FRONT_GREEN.setState(false);        // Turn ON Green LED
                    LED_FRONT_RED.setState(true);
                }
                else
                {
                    LED_FRONT_GREEN.setState(true);         // Turn ON Red LED
                    LED_FRONT_RED.setState(false);
                }

                if (CurrentGamepad2_X && !LastGamepad2_X)
                {
                    if (CyclePosition == 1)                 // At Left Position
                    {
                        LED_RIGHT_GREEN.setState(true);       // Turn ON Green LED
                        LED_LEFT_RED.setState(false);

                        CyclePosition = 2;                    // Change to Right Position
                    }
                    else if (CyclePosition == 2)            // At Right Position
                    {
                        LED_RIGHT_GREEN.setState(true);       // Turn OFF LED
                        LED_LEFT_RED.setState(true);

                        CyclePosition = 0;                    // Change to Center Position
                    }
                    else if (CyclePosition == 0)            // At Center Position
                    {
                        LED_RIGHT_GREEN.setState(false);      // Turn ON Red LED
                        LED_LEFT_RED.setState(true);

                        CyclePosition = 1;                    // Change to Left Position
                    }
                }

                // ===============================================================================================================
                // Intake Arm and Gripper Movement
                // ===============================================================================================================
                if (CurrentGamepad1_Y && !LastGamepad1_Y)
                {
                    if (IntakeArmPositionFlag == 0)
                    {
                        SERVO_ARM_INTAKE.setPosition(0.02);        // Intake Outside
                        IntakeArmPositionFlag = 1;
                    }
                    else
                    {
                        SERVO_ARM_INTAKE.setPosition(0.45);         //Intake Inside maximum = 0.62
                        IntakeArmPositionFlag = 0;
                    }
                }
                else if (CurrentGamepad1_B && !LastGamepad1_B)
                {
                    if (IntakeGripperPositionFlag == 0 )
                    {
                        SERVO_GRIPPER_INTAKE.setPosition(ClawOpenPosition);    // Gripper Claw Open
                        IntakeGripperPositionFlag = 1;
                    }
                    else
                    {
                        SERVO_GRIPPER_INTAKE.setPosition(ClawClosePosition);   // Gripper Claw Close
                        IntakeGripperPositionFlag = 0;
                    }
                }

                // ============================================================================================================================
                // Semi-Automatic (Cycling): Cone Delivery Sequence (Vertical Linear Rack Up/Down, Arm Deliver swing in/out, Cone Release/Hold)
                // ============================================================================================================================
                if (gamepad2.right_bumper)
                {
                    MOTOR1.setPower(0);
                    MOTOR2.setPower(0);
                    MOTOR3.setPower(0);
                    MOTOR4.setPower(0);

                    if (OuttakePosition == 0)
                    {
                        SERVO_HOLD_DELIVER.setPosition(HoldConePosition);     // Ensure to Close and Hold the Cone before linear rack move up
                        DeliverHoldPositionFlag = 1;
                        sleep(100);

                        MOTOR_LINEARRACK_DELIVER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        MOTOR_LINEARRACK_DELIVER.setTargetPosition(HighLevel);
                        MOTOR_LINEARRACK_DELIVER.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        if (MOTOR_LINEARRACK_DELIVER.isBusy())  MOTOR_LINEARRACK_DELIVER.setPower(1);
                        else                                    MOTOR_LINEARRACK_DELIVER.setPower(0);
                        LinearRackDeliverLevel = 3;

                        SERVO_ARM_DELIVER.setPosition(0);                     // Deliver Cone to the junction
                        DeliverArmPositionFlag = 0;

                        if      (CyclePosition == 1)  SERVO_TURRET.setPosition(TurretRightPosition);  // 0.47  Turret turn Right
                        else if (CyclePosition == 2)  SERVO_TURRET.setPosition(TurretLeftPosition);
                        else                          SERVO_TURRET.setPosition(TurretHomePosition);

                        OuttakePosition = 1;
                        sleep(150);
                    }
                    else if (OuttakePosition == 1)
                    {
                        SERVO_HOLD_DELIVER.setPosition(ReleaseConePosition);
                        DeliverHoldPositionFlag = 0;
                        sleep(200);

                        SERVO_TURRET.setPosition(TurretHomePosition);           // Turn and ensure the Turret is in the middle position before swinging the Deliver Arm into inside of the robot
                        SERVO_ARM_DELIVER.setPosition(ArmDeliverHomePosition);  // Deliver Arm swing into inside of the building
                        SERVO_GUIDE.setPosition(0.18);                          // Ensure the guide is back upright
                        DeliverArmPositionFlag = 1;
                        sleep(200);

                        MOTOR_LINEARRACK_DELIVER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        MOTOR_LINEARRACK_DELIVER.setTargetPosition(0);
                        MOTOR_LINEARRACK_DELIVER.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        if (MOTOR_LINEARRACK_DELIVER.isBusy())  MOTOR_LINEARRACK_DELIVER.setPower(0.8);
                        else                                    MOTOR_LINEARRACK_DELIVER.setPower(0);

                        LinearRackDeliverLevel = 0;
                        OuttakePosition = 0;

                        sleep(150);
                    }
                }


                // ===============================================================================================================
                // Deliver Arm and Cone Holding Movements, Junction Distance Guide
                // ===============================================================================================================
                if (CurrentGamepad2_Y && !LastGamepad2_Y)                     // Deliver Arm
                {
                    if (DeliverArmPositionFlag == 1)                            // Deliver Arm is inside (Front)
                    {
                        if (DeliverHoldPositionFlag == 0)                       // If the Cone Holder is open
                        {
                            SERVO_HOLD_DELIVER.setPosition(HoldConePosition);     // Close and Hold the Cone
                            DeliverHoldPositionFlag = 1;
                            sleep(300);
                        }

                        SERVO_ARM_DELIVER.setPosition(ArmDeliverOutPosition);   // Deliver cone to the junction
                        DeliverArmPositionFlag = 0;
                    }
                    else                                                        // Deliver Arm is outside (Back)
                    {
                        SERVO_TURRET.setPosition(TurretHomePosition);           // Turn and ensure the Turret is in the middle position before swinging the Deliver Arm into inside of the robot
                        SERVO_ARM_DELIVER.setPosition(ArmDeliverHomePosition);  // Deliver Arm swing into inside of the building
                        DeliverArmPositionFlag = 1;
                    }
                }
                else if (CurrentGamepad2_B && !LastGamepad2_B)
                {
                    if (DeliverHoldPositionFlag == 1)
                    {
                        SERVO_HOLD_DELIVER.setPosition(ReleaseConePosition);      // Open and Release the Cone
                        DeliverHoldPositionFlag = 0;
                    }
                    else
                    {
                        SERVO_HOLD_DELIVER.setPosition(HoldConePosition);       // Close and Hold the Cone
                        DeliverHoldPositionFlag = 1;
                    }
                }
                else if (CurrentGamepad2_A && !LastGamepad2_A)
                {
                    if (LinearRackDeliverLevel != 3)                        // Deliver Linear Rack moves up to High Junction Level
                    {
                        if (DeliverHoldPositionFlag == 0)                     // If the Cone Holder is open
                        {
                            SERVO_HOLD_DELIVER.setPosition(HoldConePosition);   // Close and Hold the Cone
                            DeliverHoldPositionFlag = 1;
                            sleep(300);
                        }

                        MOTOR_LINEARRACK_DELIVER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        MOTOR_LINEARRACK_DELIVER.setTargetPosition(HighLevel);

                        MOTOR_LINEARRACK_DELIVER.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        if (MOTOR_LINEARRACK_DELIVER.isBusy())  MOTOR_LINEARRACK_DELIVER.setPower(1);
                        else                                    MOTOR_LINEARRACK_DELIVER.setPower(0);
                        LinearRackDeliverLevel = 3;

                        sleep(300);
                    }
                    else                                                        // Deliver Linear Rack moves down to initial home level
                    {
                        if (DeliverArmPositionFlag == 0)                          // To make sure the Deliver Arm is swing back inside the robot
                        {
                            SERVO_ARM_DELIVER.setPosition(ArmDeliverHomePosition);  // Deliver Arm swing into inside of the building
                            DeliverArmPositionFlag = 1;
                        }

                        MOTOR_LINEARRACK_DELIVER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        MOTOR_LINEARRACK_DELIVER.setTargetPosition(0);

                        MOTOR_LINEARRACK_DELIVER.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        if (MOTOR_LINEARRACK_DELIVER.isBusy())  MOTOR_LINEARRACK_DELIVER.setPower(0.8);
                        else                                    MOTOR_LINEARRACK_DELIVER.setPower(0);
                        LinearRackDeliverLevel = 0;

                        sleep(300);
                    }
                }

/*      // Swing down the guide - to measure the distance to the Junction
        else if (CurrentGamepad2_X && !LastGamepad2_X)
        {
          if (GuidePosition == 0)
          {
            SERVO_GUIDE.setPosition(0.5);
            GuidePosition = 1;
          }
          else
          {
            SERVO_GUIDE.setPosition(0.18);
            GuidePosition = 0;
          }
        }
*/

                // ===============================================================================================================
                // Robot Movements
                // ===============================================================================================================
                if (gamepad1.right_stick_y != 0)        // Robot Movement: Forward and Backward
                {
                    if (SlowSpeedFlag == 1) MotorPower = gamepad1.right_stick_y * SlowSpeedFactor;
                    else                    MotorPower = gamepad1.right_stick_y * ReduceSpeedFactor;

                    //if (RobotFrontFlag == 1)              // Robot Cone Intake is the Front
                    //{
                    MOTOR1.setPower(MotorPower);
                    MOTOR2.setPower(-MotorPower);
                    MOTOR3.setPower(MotorPower);
                    MOTOR4.setPower(-MotorPower);
                    //}
                    //else                                  // Robot Cone Deliver (Back) is the Front
                    //{
                    //  MOTOR1.setPower(-MotorPower);
                    //  MOTOR2.setPower(MotorPower);
                    //  MOTOR3.setPower(-MotorPower);
                    //  MOTOR4.setPower(MotorPower);
                    //}
                }
                else if (gamepad1.left_stick_x != 0)    // Robot Movement: Sideway Right or Left
                {
                    if (SlowSpeedFlag == 1) MotorPower = -gamepad1.left_stick_x * SlowSpeedFactor;
                    else                    MotorPower = -gamepad1.left_stick_x * ReduceSpeedFactor;

                    if (RobotFrontFlag == 1)              // Robot Cone Intake is the Front
                    {
                        MOTOR1.setPower(MotorPower);
                        MOTOR2.setPower(MotorPower);
                        MOTOR3.setPower(-MotorPower);
                        MOTOR4.setPower(-MotorPower);
                    }
                    else                                  // Robot Cone Deliver (Back) is the Front
                    {
                        MOTOR1.setPower(-MotorPower);
                        MOTOR2.setPower(-MotorPower);
                        MOTOR3.setPower(MotorPower);
                        MOTOR4.setPower(MotorPower);
                    }
                }
                else if (gamepad1.right_stick_x != 0)    // Robot (Front=Intake/Deliver) Movement: TurnRight or Left
                {
                    if (SlowSpeedFlag == 1) MotorPower = gamepad1.right_stick_x * SlowSpeedFactor;            // Slow Speed Mode
                    else                    MotorPower = gamepad1.right_stick_x * ReduceSpeedFactor;          // Reduce the turning speed by 50%

                    MOTOR1.setPower(-MotorPower);
                    MOTOR2.setPower(-MotorPower);
                    MOTOR3.setPower(-MotorPower);
                    MOTOR4.setPower(-MotorPower);
                }

                // ===============================================================================================================
                // Linear Rack - Intake Movement
                // ===============================================================================================================
                else if (gamepad1.dpad_up == true)      // Intake Linear Rack Outward
                {
                    LinearRackIntakePosition = LinearRackIntakePosition + ServoLinearRackIntakeStep;
                    if (LinearRackIntakePosition > LinearRackIntakeFarOutPosition)   LinearRackIntakePosition = LinearRackIntakeFarOutPosition;     // Set Linear Rack Outward Limit (Clockwise)

                    SERVO_LINEARRACK_INTAKE.setPosition(LinearRackIntakePosition);
                    sleep(20);
                }
                else if (gamepad1.dpad_down == true)    // Intake Linear Rack inward
                {
                    LinearRackIntakePosition = LinearRackIntakePosition - ServoLinearRackIntakeStep;
                    if (LinearRackIntakePosition < LinearRackIntakeHomePosition)   LinearRackIntakePosition = LinearRackIntakeHomePosition;         // Set Linear Rack Inward Limit (Anti-Clockwise)

                    SERVO_LINEARRACK_INTAKE.setPosition(LinearRackIntakePosition);
                    sleep(20);
                }

                // ===============================================================================================================
                // Turret Positioning
                // ===============================================================================================================
                else if ((gamepad2.dpad_right == true) && (DeliverArmPositionFlag == 0))    // Only able turrel turn when the Deliver Arm is outside the robot
                {
                    ServoXPosition = ServoXPosition + ServoPositionStep;

                    if (LinearRackDeliverLevel == 3)
                    {
                        if (ServoXPosition > 0.58)   ServoXPosition = 0.58;           // Set Clockwise Turn Limit 0.6 0.53
                    }
                    else
                    {
                        if (ServoXPosition > 0.55)   ServoXPosition = 0.55;           // Set Clockwise Turn Limit 0.6 0.53
                    }

                    SERVO_TURRET.setPosition(ServoXPosition);
                    sleep(20);
                }
                else if ((gamepad2.dpad_left == true) && (DeliverArmPositionFlag == 0))
                {
                    ServoXPosition = ServoXPosition - ServoPositionStep;
                    if (ServoXPosition < 0.45)   ServoXPosition = 0.45;             // Set Anti-Clockwise Turn Limit 0.4

                    SERVO_TURRET.setPosition(ServoXPosition);
                    sleep(20);
                }
                else if (gamepad2.dpad_up == true)
                {
                    ServoYPosition = SERVO_ARM_DELIVER.getPosition() + ServoPositionStep;

                    if (ServoYPosition > ArmDeliverHomePosition)   ServoYPosition = ArmDeliverHomePosition;
                    SERVO_ARM_DELIVER.setPosition(ServoYPosition);

                    sleep(20);
                }
                else if (gamepad2.dpad_down == true)
                {
                    ServoYPosition = SERVO_ARM_DELIVER.getPosition() - ServoPositionStep;

                    if (ServoYPosition < 0)   ServoYPosition = 0;
                    SERVO_ARM_DELIVER.setPosition(ServoYPosition);

                    sleep(20);
                }

                else
                {
                    MOTOR1.setPower(0);
                    MOTOR2.setPower(0);
                    MOTOR3.setPower(0);
                    MOTOR4.setPower(0);
                }



                // ===============================================================================================================
                // Collect Cone at the Sub Station (Mode 1) or at Cone Stack Line (Mode 2)
                // ===============================================================================================================
                if (gamepad1.left_trigger == 1)
                {
                    if (Mode == 1)
                    {
                        Mode = 2;
                        sleep(100);
                    }
                    else
                    {
                        Mode = 1;
                        sleep(100);
                    }
                }

                if (gamepad1.right_bumper)      // Cone Intake Semi-Automatic (Cycle: To collect cone at sub-statsion)
                {
                    MOTOR1.setPower(0);
                    MOTOR2.setPower(0);
                    MOTOR3.setPower(0);
                    MOTOR4.setPower(0);

                    if (Mode == 1)
                    {
                        if (Position == 0)                                        // Intake Linear Rack at home position (inside the robot)
                        {
                            SERVO_ARM_INTAKE.setPosition(0.02);                     // Intake Arm swing Outside
                            IntakeArmPositionFlag = 1;

                            SERVO_GRIPPER_INTAKE.setPosition(ClawOpenPosition);     // Gripper Claws Open
                            IntakeGripperPositionFlag = 1;

                            SERVO_LINEARRACK_INTAKE.setPosition(LinearRackIntakeFarOutPosition);
                            Position = 1;

                            SERVO_HOLD_DELIVER.setPosition(ReleaseConePosition);
                            DeliverHoldPositionFlag = 0;
                            sleep(300);
                        }
                        else if (Position == 1  && LinearRackDeliverLevel == 0)
                        {
                            SERVO_GRIPPER_INTAKE.setPosition(ClawClosePosition);        // Gripper Claw Close
                            IntakeGripperPositionFlag = 0;
                            sleep(300);

                            SERVO_ARM_INTAKE.setPosition(ArmIntakeHomePosition);        // 0.46 Intake swing back inside
                            IntakeArmPositionFlag = 0;
                            sleep(100);

                            SERVO_LINEARRACK_INTAKE.setPosition(LinearRackIntakeHomePosition + 0.02);
                            sleep(500);

                            SERVO_GRIPPER_INTAKE.setPosition(ClawOpenPosition);    // Gripper Claw Open
                            IntakeGripperPositionFlag = 1;
                            Position = 0;
                            sleep(200);
                        }
                    }
                    else if (Mode == 2)
                    {
                        if (Position == 0)
                        {
                            SERVO_ARM_INTAKE.setPosition(0.03);          // 0.05           // Intake Outside
                            IntakeArmPositionFlag = 1;
                            SERVO_GRIPPER_INTAKE.setPosition(ClawOpenPosition);     // Gripper Claw Open
                            IntakeGripperPositionFlag = 1;

                            Position = 1;
                            sleep(1000);
                        }
                        else if (Position ==1 && DeliverArmPositionFlag == 0)
                        {
                            SERVO_HOLD_DELIVER.setPosition(ReleaseConePosition);
                            DeliverHoldPositionFlag = 0;
                            SERVO_GRIPPER_INTAKE.setPosition(ClawClosePosition);    // Gripper Claw Close
                            IntakeGripperPositionFlag = 0;
                            sleep(300);

                            SERVO_ARM_INTAKE.setPosition(ArmIntakeHomePosition );   // Intake swing back inside
                            IntakeArmPositionFlag = 0;
                            sleep(600);

                            SERVO_GRIPPER_INTAKE.setPosition(ClawOpenPosition);     // Gripper Claw Open
                            IntakeGripperPositionFlag = 1;
                            Position = 0;
                            sleep(500);
                            SERVO_HOLD_DELIVER.setPosition(HoldConePosition);
                            DeliverHoldPositionFlag = 1;
                        }
                    }
                }

                if (gamepad1.left_bumper)
                {
                    if (StackPosition == 5)                         // Cone stack on 5th Level
                    {
                        SERVO_ARM_INTAKE.setPosition(0.15);
                        StackPosition = 4;
                        IntakeArmPositionFlag = 1;
                        sleep(200);
                    }
                    else if (StackPosition == 4)                    // Cone stack on 4th Level
                    {
                        SERVO_ARM_INTAKE.setPosition(0.12);
                        StackPosition = 3;
                        IntakeArmPositionFlag = 1;
                        sleep(200);
                    }
                    else if (StackPosition == 3)                    // Cone stack on 3rd Level
                    {
                        SERVO_ARM_INTAKE.setPosition(0.1);
                        StackPosition = 2;
                        IntakeArmPositionFlag = 1;
                        sleep(200);
                    }
                    else if (StackPosition == 2)                     // Cone stack on 2nd Level
                    {
                        SERVO_ARM_INTAKE.setPosition(0.06);
                        StackPosition = 1;
                        IntakeArmPositionFlag = 1;
                        sleep(200);
                    }
                    else if (StackPosition == 1)                     // Cone stack on Ground Level
                    {
                        SERVO_ARM_INTAKE.setPosition(0.02);
                        StackPosition = 5;
                        IntakeArmPositionFlag = 1;
                        sleep(200);
                    }
                }

                // Save the current toggle buttons status
                LastGamepad1_X = CurrentGamepad1_X;       // Slow Speed Toggle
                LastGamepad1_Y = CurrentGamepad1_Y;       // Intake Arm
                LastGamepad1_B = CurrentGamepad1_B;       // Intake Gripper
                LastGamepad1_A = CurrentGamepad1_A;       // Robot Front

                LastGamepad2_X = CurrentGamepad2_X;       // Junction Distance Guide
                LastGamepad2_Y = CurrentGamepad2_Y;       // Deliver Arm
                LastGamepad2_B = CurrentGamepad2_B;       // Driver Cone Holder
                LastGamepad2_A = CurrentGamepad2_A;       // Vertical Linear Rack Up/down

                telemetry.addData("LinearRack ", MOTOR_LINEARRACK_DELIVER.getCurrentPosition());
                telemetry.update();


            } // End of "while (opModeIsActive())"
        }   // End of "if (opModeIsActive())"
    }


    private void MoveFront(double DISTANCE, double POWER)
    {
        while (opModeIsActive())
        {
            if (MOTOR1.getCurrentPosition() <= -DISTANCE)
            {
                MOTOR1.setPower(0);
                MOTOR3.setPower(0);
            }
            else
            {
                MOTOR1.setPower(-POWER);
                MOTOR3.setPower(-POWER);
            }

            if (MOTOR4.getCurrentPosition() >= DISTANCE)
            {
                MOTOR2.setPower(0);
                MOTOR4.setPower(0);
            }
            else
            {
                MOTOR2.setPower(POWER);
                MOTOR4.setPower(POWER);
            }

            if (MOTOR1.getPower() == 0 && MOTOR4.getPower() == 0) break;
            //telemetry.addData("MOTOR1 ", MOTOR1.getCurrentPosition());
            //telemetry.addData("MOTOR2 ", MOTOR2.getCurrentPosition());
            //telemetry.addData("MOTOR3 ", MOTOR3.getCurrentPosition());
            //telemetry.addData("MOTOR4 ", MOTOR4.getCurrentPosition());
            //telemetry.update();
        }

        ResetMotors();
    }

    private void MoveBack(double DISTANCE, double POWER)
    {
        while (opModeIsActive())
        {
            if (MOTOR1.getCurrentPosition() >= DISTANCE)
            {
                MOTOR1.setPower(0);
                MOTOR3.setPower(0);
            }
            else
            {
                MOTOR1.setPower(POWER);
                MOTOR3.setPower(POWER);
            }

            if (MOTOR4.getCurrentPosition() <= -DISTANCE)
            {
                MOTOR2.setPower(0);
                MOTOR4.setPower(0);
            }
            else
            {
                MOTOR2.setPower(-POWER);
                MOTOR4.setPower(-POWER);
            }

            if (MOTOR1.getPower() == 0 && MOTOR4.getPower() == 0) break;
            //telemetry.addData("MOTOR1 ", MOTOR1.getCurrentPosition());
            //telemetry.addData("MOTOR2 ", MOTOR2.getCurrentPosition());
            //telemetry.addData("MOTOR3 ", MOTOR3.getCurrentPosition());
            //telemetry.addData("MOTOR4 ", MOTOR4.getCurrentPosition());
            //telemetry.update();
        }

        ResetMotors();
    }

    private void MoveSideLeft(double DISTANCE, double POWER)
    {
        while (opModeIsActive())
        {
            if (MOTOR1.getCurrentPosition() >= DISTANCE)
            {
                MOTOR1.setPower(0);
                MOTOR3.setPower(0);
            }
            else
            {
                MOTOR1.setPower(POWER);
                MOTOR3.setPower(-POWER);
            }

            if (MOTOR4.getCurrentPosition() <= -DISTANCE)
            {
                MOTOR2.setPower(0);
                MOTOR4.setPower(0);
            }
            else
            {
                MOTOR2.setPower(POWER);
                MOTOR4.setPower(-POWER);
            }

            if (MOTOR1.getPower() == 0 && MOTOR4.getPower() == 0) break;

            //telemetry.addData("MOTOR1 ", MOTOR1.getCurrentPosition());
            //telemetry.addData("MOTOR2 ", MOTOR2.getCurrentPosition());
            //telemetry.addData("MOTOR3 ", MOTOR3.getCurrentPosition());
            //telemetry.addData("MOTOR4 ", MOTOR4.getCurrentPosition());
            //telemetry.update();
        }

        ResetMotors();
    }

    private void MoveSideRight(double DISTANCE, double POWER)
    {
        while (opModeIsActive())
        {
            if (MOTOR1.getCurrentPosition() <= -DISTANCE)
            {
                MOTOR1.setPower(0);
                MOTOR3.setPower(0);
            }
            else
            {
                MOTOR1.setPower(-POWER);
                MOTOR3.setPower(POWER);
            }

            if (MOTOR4.getCurrentPosition() >= DISTANCE)
            {
                MOTOR2.setPower(0);
                MOTOR4.setPower(0);
            }
            else
            {
                MOTOR2.setPower(-POWER);
                MOTOR4.setPower(POWER);
            }

            if (MOTOR1.getPower() == 0 && MOTOR4.getPower() == 0) break;

            //telemetry.addData("MOTOR1 ", MOTOR1.getCurrentPosition());
            //telemetry.addData("MOTOR2 ", MOTOR2.getCurrentPosition());
            //telemetry.addData("MOTOR3 ", MOTOR3.getCurrentPosition());
            //telemetry.addData("MOTOR4 ", MOTOR4.getCurrentPosition());
            //telemetry.update();
        }

        ResetMotors();
    }

    private void TurnRight(double DISTANCE, double POWER)
    {
        while (opModeIsActive())
        {
            if (MOTOR1.getCurrentPosition() <= -DISTANCE)
            {
                MOTOR1.setPower(0);
                MOTOR3.setPower(0);
            }
            else
            {
                MOTOR1.setPower(-POWER);
                MOTOR3.setPower(-POWER);
            }

            if (MOTOR4.getCurrentPosition() <= -DISTANCE)
            {
                MOTOR2.setPower(0);
                MOTOR4.setPower(0);
            }
            else
            {
                MOTOR2.setPower(-POWER);
                MOTOR4.setPower(-POWER);
            }

            if (MOTOR1.getPower() == 0 && MOTOR4.getPower() == 0) break;

            //telemetry.addData("MOTOR1 ", MOTOR1.getCurrentPosition());
            //telemetry.addData("MOTOR2 ", MOTOR2.getCurrentPosition());
            //telemetry.addData("MOTOR3 ", MOTOR3.getCurrentPosition());
            //telemetry.addData("MOTOR4 ", MOTOR4.getCurrentPosition());
            //telemetry.update();
        }

        ResetMotors();
    }

    private void TurnLeft(double DISTANCE, double POWER)
    {
        while (opModeIsActive())
        {
            if (MOTOR1.getCurrentPosition() >= DISTANCE)
            {
                MOTOR1.setPower(0);
                MOTOR3.setPower(0);
            }
            else
            {
                MOTOR1.setPower(POWER);
                MOTOR3.setPower(POWER);
            }

            if (MOTOR4.getCurrentPosition() >= DISTANCE)
            {
                MOTOR2.setPower(0);
                MOTOR4.setPower(0);
            }
            else
            {
                MOTOR2.setPower(POWER);
                MOTOR4.setPower(POWER);
            }

            if (MOTOR1.getPower() == 0 && MOTOR4.getPower() == 0) break;

            //telemetry.addData("MOTOR1 ", MOTOR1.getCurrentPosition());
            //telemetry.addData("MOTOR2 ", MOTOR2.getCurrentPosition());
            //telemetry.addData("MOTOR3 ", MOTOR3.getCurrentPosition());
            //telemetry.addData("MOTOR4 ", MOTOR4.getCurrentPosition());
            //telemetry.update();
        }

        ResetMotors();
    }

    private void ResetMotors()
    {
        MOTOR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MOTOR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MOTOR3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MOTOR4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
