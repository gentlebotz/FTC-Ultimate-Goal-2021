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
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name="AutonomousRingDetect", group="Linear Opmode")

public class WebcamDetectionAutonomous extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motor4 = null;
    private DcMotor motorintake = null;
    private DcMotor armHef = null;
    private DcMotor armDraai = null;
    private DcMotor motorshoot = null;
    
    private Servo handHorizontaal = null;
    private Servo handVerticaal = null;
    private Servo schietServo = null;
    
    private double PowerA = 0.20;
    private boolean holdingWobble = false;
    int Z = 0;

    private int target = 0;
    private double speed = 0.01;
    private int speed2 = 30;
    private int current;

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
     double leftPower, rightPower;
    double offset;
    
    public void resetEncoders(){
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
        
    
    public void forward(int target) {
        // double startTime = runtime.milliseconds();
        //while(runtime.milliseconds() < startTime + waitTime && opModeIsActive()){
        resetEncoders();
        motor1.setTargetPosition(target);
        motor2.setTargetPosition(target);
        motor3.setTargetPosition(target);
        motor4.setTargetPosition(target);
        
        motor1.setPower(PowerA);
        motor2.setPower(PowerA);
        motor3.setPower(PowerA);
        motor4.setPower(PowerA);
        while(opModeIsActive() && motor1.isBusy()){
            correction = checkDirection();
            leftTankSteering(power + correction);
            rightTankSteering(power - correction);
            telemetry.addData("encoder-motor1", motor1.getCurrentPosition() + "  busy=" + motor1.isBusy());
            telemetry.addData("encoder-motor2", motor2.getCurrentPosition() + "  busy=" + motor2.isBusy());
            telemetry.addData("encoder-motor1", motor3.getCurrentPosition() + "  busy=" + motor3.isBusy());
            telemetry.addData("encoder-motor2", motor4.getCurrentPosition() + "  busy=" + motor4.isBusy());
            telemetry.update();
        } 
    }
    
    
    public void turn(double power, int time) {
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        motor1.setPower(power);
        motor2.setPower(-power);
        motor3.setPower(power);
        motor4.setPower(-power);
        sleep(time);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
    
    public void leftTankSteering(double powerL){
        motor1.setPower(powerL);
        motor3.setPower(powerL);
    }
    
    public void rightTankSteering(double powerL){
        motor2.setPower(powerL);
        motor4.setPower(powerL);
    }
    
    
    public void forwardNC(int time){
        motor1.setPower(PowerA);
        motor2.setPower(PowerA);
        motor3.setPower(PowerA);
        motor4.setPower(PowerA);
        sleep(time);
    }

    public void left(int target) {
        resetEncoders();
        motor1.setTargetPosition(-target);
        motor2.setTargetPosition(target);
        motor3.setTargetPosition(target);
        motor4.setTargetPosition(-target);
        
        motor1.setPower(1.2 * PowerA);
        motor2.setPower(1.2 * PowerA);
        motor3.setPower(1.2 * PowerA);
        motor4.setPower(1.2 * PowerA);
        while(opModeIsActive() && motor1.isBusy()){
            telemetry.addData("encoder-motor1", motor1.getCurrentPosition() + "  busy=" + motor1.isBusy());
            telemetry.addData("encoder-motor2", motor2.getCurrentPosition() + "  busy=" + motor2.isBusy());
            telemetry.addData("encoder-motor1", motor3.getCurrentPosition() + "  busy=" + motor3.isBusy());
            telemetry.addData("encoder-motor2", motor4.getCurrentPosition() + "  busy=" + motor4.isBusy());
            telemetry.update();
        }
    }

    public void right(int target) {
        resetEncoders();
        motor1.setTargetPosition(target);
        motor2.setTargetPosition(-target);
        motor3.setTargetPosition(-target);
        motor4.setTargetPosition(target);
        
        motor1.setPower(PowerA);
        motor2.setPower(PowerA);
        motor3.setPower(PowerA);
        motor4.setPower(PowerA);
        while(opModeIsActive() && motor1.isBusy()){
            telemetry.addData("encoder-motor1", motor1.getCurrentPosition() + "  busy=" + motor1.isBusy());
            telemetry.addData("encoder-motor2", motor2.getCurrentPosition() + "  busy=" + motor2.isBusy());
            telemetry.addData("encoder-motor1", motor3.getCurrentPosition() + "  busy=" + motor3.isBusy());
            telemetry.addData("encoder-motor2", motor4.getCurrentPosition() + "  busy=" + motor4.isBusy());
            telemetry.update();
        }
    }

    public void backward(int target) {
        resetEncoders();
        motor1.setTargetPosition(-target);
        motor2.setTargetPosition(-target);
        motor3.setTargetPosition(-target);
        motor4.setTargetPosition(-target);
        
        motor1.setPower(0.30);
        motor2.setPower(0.30);
        motor3.setPower(0.30);
        motor4.setPower(0.30);
        while(opModeIsActive() && motor1.isBusy()){
            telemetry.addData("encoder-motor1", motor1.getCurrentPosition() + "  busy=" + motor1.isBusy());
            telemetry.addData("encoder-motor2", motor2.getCurrentPosition() + "  busy=" + motor2.isBusy());
            telemetry.addData("encoder-motor1", motor3.getCurrentPosition() + "  busy=" + motor3.isBusy());
            telemetry.addData("encoder-motor2", motor4.getCurrentPosition() + "  busy=" + motor4.isBusy());
            telemetry.update();
        }
    }
    public void turnOffMotors(){
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        motorintake.setPower(0);
        motorshoot.setPower(0);
    }
    
    public void shoot() {
        motorshoot.setPower(.63);
        sleep(2500);
        schietServo.setPosition(0.00);
        sleep(200);
        schietServo.setPosition(0.16);
        sleep(200);
        left(60);
        forward(0);
        motorintake.setPower(-1.0);
        motorshoot.setPower(.64);
        sleep(1000);
        schietServo.setPosition(0.00);
        sleep(200);
        schietServo.setPosition(0.16);
        motorintake.setPower(0);
        motorshoot.setPower(0);
        turnOffMotors();
    }
    
    public void powerShots() {
        //Drive to middle and shoot powershot targets
        forward(70);
        turnOffMotors();
        sleep(200);
        //forward(1285);
        
        //turn(PowerA, 20);
        shoot();
        
        //turn(PowerA, 110);
        //shoot(1000);
        
        turnOffMotors();
        sleep(1500);
    }
    
    public void dropWobble(){
        sleep(400);
        armHef.setTargetPosition(160);
        armHef.setPower(.5);
        sleep(1000);
        handVerticaal.setPosition(0.7);
        armDraai.setTargetPosition(-425);
        armDraai.setPower(.15);
        sleep(2000);
        handHorizontaal.setPosition(0.4);
        sleep(750);
        armDraai.setTargetPosition(-220);
        armHef.setTargetPosition(0);
        sleep(600);
        armDraai.setPower(0);
        sleep(100);
        armHef.setPower(-.08);
        sleep(800);
        turnOffMotors();
    }
    
    public void dropA(){
        forward(1450);
        right(240); //was 260
        turnOffMotors();
        sleep(100);
        dropWobble();
        sleep(500);
        left(142);
        sleep(200);
        armHef.setTargetPosition(10);
        armHef.setPower(.14);
        armDraai.setTargetPosition(-237);
        armDraai.setPower(-.2);
        sleep(600);
        //armDraai.setPower(0);
        backward(1110);
        sleep(2000); //2200
        handHorizontaal.setPosition(.93);
        sleep(2000);
        handVerticaal.setPosition(0.9);
        armHef.setTargetPosition(0);
        armHef.setPower(.45);
        armDraai.setTargetPosition(-400); //was 320
        armDraai.setPower(.3);
        sleep(100);
        forward(1100); //was 1350
        sleep(100);
        //left(100);
        dropWobble();
    }
    
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "VUFORIA_KEY";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        
        //Hardwaremap
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        motorintake = hardwareMap.get(DcMotor.class, "motorintake");
        motorshoot = hardwareMap.get(DcMotor.class, "motorarm1");
        armDraai = hardwareMap.get(DcMotor.class, "motorarm2");
        armHef = hardwareMap.get(DcMotor.class, "motorshoot");
        handHorizontaal = hardwareMap.get(Servo.class, "servo1");
        handVerticaal = hardwareMap.get(Servo.class, "servo2");
        schietServo = hardwareMap.get(Servo.class, "servo3");
        
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);
        motorintake.setDirection(DcMotor.Direction.FORWARD);
        armHef.setDirection(DcMotor.Direction.FORWARD);
        armDraai.setDirection(DcMotor.Direction.FORWARD);
        motorshoot.setDirection(DcMotor.Direction.FORWARD);
        
        motorshoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        motor1.setTargetPosition(0);
        motor2.setTargetPosition(0);
        motor3.setTargetPosition(0);
        motor4.setTargetPosition(0);
        
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        armHef.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armHef.setTargetPosition(0);
        armHef.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDraai.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDraai.setTargetPosition(0);
        armDraai.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.0, 16.0/9.0);
        }

        handHorizontaal.setPosition(.92);
        handVerticaal.setPosition(1);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        offset = getAngle();
        
        
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        // dropWobble();
        // dropA();
        // sleep(10000);
        schietServo.setPosition(0.16);
        if (opModeIsActive()) {
            while (opModeIsActive() && Z < 1) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      if (updatedRecognitions.size() == 0 ) {
                          // empty list.  no objects recognized.
                          telemetry.addData("TFOD", "No items detected.");
                          telemetry.addData("Target Zone", "A");
                          telemetry.update();
                          tfod.shutdown();
                          powerShots();
                          dropA();
                          Z = Z + 1;
                      } else {
                          // list is not empty.
                          // step through the list of recognitions and display boundary info.
                          int i = 0;
                          for (Recognition recognition : updatedRecognitions) {
                              telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                              telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                      recognition.getLeft(), recognition.getTop());
                              telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                      recognition.getRight(), recognition.getBottom());

                              // check label to see which target zone to go after.
                              if (recognition.getLabel().equals("Single")) {
                                  telemetry.addData("Target Zone", "B");
                                  telemetry.update();
                                  tfod.shutdown();
                                  powerShots();
                                  left(300);
                                  turnOffMotors();
                                  // //Location B
                                  forward(2000);
                                  turnOffMotors();
                                  sleep(500);
                                  left(200);
                                  turnOffMotors();
                                  sleep(100);
                                  dropWobble();
                                  backward(300);
                                  turnOffMotors();
                                  Z = Z + 1;
                              } else if (recognition.getLabel().equals("Quad")) {
                                  telemetry.addData("Target Zone", "C");
                                  telemetry.update();
                                  tfod.shutdown();
                                  powerShots();
                                  left(300);
                                  turnOffMotors();
                                  // //Location C
                                  // //turn(PowerA,);
                                  forward(2550);
                                  turnOffMotors();
                                  sleep(100);
                                  right(500);
                                  turnOffMotors();
                                  dropWobble();
                                  backward(900);
                                  turnOffMotors();
                                  Z = Z + 1;
                              } else {
                                  telemetry.addData("Target Zone", "UNKNOWN");
                                  telemetry.update();
                                  tfod.shutdown();
                                  powerShots();
                                  dropA();
                                  Z = Z + 1;
                              }
                          }
                      }
                      telemetry.update();
                    }
                }
            }
        }
        Z = Z + 1;
        
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    
    /**
     * Resets the cumulative angle tracking to zero.
     */
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
        double correction, angle, gain = .015;

        angle = getAngle() - offset;

        if (angle == -55 )
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
    private void rotate(double degrees, double power)
    {
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        double  leftPower, rightPower;
        degrees  = degrees - 25;
        
        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        rightTankSteering(leftPower);
        leftTankSteering(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }   
}