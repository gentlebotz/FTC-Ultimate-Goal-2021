package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.os.SystemClock.currentThreadTimeMillis;
import static android.os.SystemClock.setCurrentTimeMillis;
import static android.os.SystemClock.sleep;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Driving", group = "Iterative Opmode")
//@Disabled
public class Driving extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motor4 = null;
    private DcMotor motorintake = null;
    private DcMotor motorarm1 = null;
    private DcMotor motorarm2 = null;
    private DcMotor motorshoot = null;
    private double power2 = 0.45;
    private boolean turboStop = true;
    private boolean turbo = true;
    private double testpower = 0.6;
    private Servo servo1 = null;
    private Servo servo2 = null;
    private Servo servo3 = null;
    private int target = 0;
    private double speed = 0.01;
    private int speed2 = 30;
    private int current;
    private boolean motorshooting1 = false;
    private boolean buttonPressed = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override

    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        motorintake = hardwareMap.get(DcMotor.class, "motorintake");
        motorshoot = hardwareMap.get(DcMotor.class, "motorarm1");
        motorarm2 = hardwareMap.get(DcMotor.class, "motorarm2");
        motorarm1 = hardwareMap.get(DcMotor.class, "motorshoot");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);
        motorintake.setDirection(DcMotor.Direction.FORWARD);
        motorarm1.setDirection(DcMotor.Direction.FORWARD);
        motorarm2.setDirection(DcMotor.Direction.FORWARD);
        motorshoot.setDirection(DcMotor.Direction.FORWARD);
        
        motorshoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorarm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double G1leftStickY = -gamepad1.left_stick_y;
        double G1leftStickX = -gamepad1.left_stick_x;
        double G1rightStickX = gamepad1.right_stick_x;
        double G2rightStickY = -gamepad2.right_stick_y;
        double G2rightStickX = -gamepad2.right_stick_x;
        double G2leftStickY = -gamepad2.left_stick_y;
        
        motor1.setPower(power2 * (G1leftStickY + -G1leftStickX + 0.5 * -G1rightStickX));
        motor2.setPower(power2 * (G1leftStickY + G1leftStickX + 0.5 * G1rightStickX));
        motor3.setPower(power2 * (G1leftStickY + G1leftStickX + 0.5 * -G1rightStickX));
        motor4.setPower(power2 * (G1leftStickY + -G1leftStickX + 0.5 * G1rightStickX));
        motorarm2.setPower(0.3 * G2rightStickX);
        current =  motorarm1.getCurrentPosition();
       
        int current = motorarm1.getCurrentPosition();
        int incr = (int)(-gamepad2.left_stick_y * speed2);
        
        target += incr;
        
        if(target > 800)
        {
            target = 800;
        }
        
        if(target < -60)
        {
            target = -60;
        }
        
        if(Math.abs(target-current) > speed2)
        {
            target = current + (int)Math.signum(target-current)*speed2;
        }
        
        motorarm1.setTargetPosition(target);
        motorarm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorarm1.setPower(0.3);
        
        if (gamepad2.b) {
            servo1.setPosition(0.60);
        }
        
        if (gamepad2.x) {
            servo1.setPosition(.93);
        }
        
        if (gamepad2.a) {
            servo2.setPosition(0.65);
        }
        
        if (gamepad2.y) {
            servo2.setPosition(1);
        }
        
        // if (gamepad1.a && motorshooting1 == false) {
        //     motorshooting1 = true;
        //     telemetry.addData(">", motorshooting1);
        // }
        
        // if (gamepad1.a && motorshooting1 == true) {
        //     motorshooting1 = false;
        //     telemetry.addData(">", motorshooting1);
        // }
        
        // if (motorshooting1 == true) {
        //     motorshoot.setPower(0.7);
        //     telemetry.addData(">", "Shooting");
        // }
        
        // if (motorshooting1 == false) {
        //     motorshoot.setPower(0);
        // }
        
        // if (gamepad1.a){ //&& buttonPressed == false){
        //     motorshoot.setPower(0.56);
            
        //     // buttonPressed = true;
            
        //     // if(motorshooting1 == true){
        //     //     motorshoot.setPower(.56);
        //     // }
        
        //     // if(motorshooting1 == false){
        //     //     motorshoot.setPower(0);
        //     // }
            
        //     // if(motorshooting1 == false){
        //     //     motorshooting1 = true;
        //     //     buttonPressed = false;
        //     //     }
            
        //     // else{
        //     //     motorshooting1 = false;
        //     //     buttonPressed = false;
        //     //     }
        // }

        if(gamepad1.a){
            motorshoot.setPower(0.54);
            
        }
        
        if (!gamepad1.a) {
            motorshoot.setPower(0);
        }
        
        if (gamepad1.b) {
            servo3.setPosition(0);
        }
        
        if (!gamepad1.b) {
            servo3.setPosition(0.16);
        }
        
        if (gamepad1.x && !gamepad1.y) {
            motorintake.setPower(1);
        }
        
        if (!gamepad1.x && !gamepad1.y) {
            motorintake.setPower(0);
        }
        
        if (gamepad1.y && !gamepad1.x) {
            motorintake.setPower(-1);
        }
        
        if (gamepad1.left_bumper && turboStop) {
            turboStop = false;
            turbo = !turbo;
            power2 = turbo ? 0.45 : 1;
        }
        
        if(gamepad2.left_bumper){
            target = 450;
        }

        else if (!gamepad1.left_bumper) {
            turboStop = true;
        }

        telemetry.addData("power mode: ", turbo ? "geen turbo" : "turbo");
        //telemetry.addData("yaw is: ", yaw);
        telemetry.addData("runtime", runtime);
        
        telemetry.addData("encoder-shoot", motorarm1.getCurrentPosition() + " target=" + target + " busy=" + motorarm1.isBusy());
        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }

}
