package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Main", group="Linear Opmode")
public class Main extends LinearOpMode {
  private boolean running = false;

  private Drive getDriver() {
    DcMotor m1 = null;
    DcMotor m2 = null;
    DcMotor m3 = null;
    DcMotor m4 = null;
    Servo s1 = null;
    DistanceSensor sensor = null;
    m1 = hardwareMap.get(DcMotor.class, "1");
    m2 = hardwareMap.get(DcMotor.class, "2");
    m3 = hardwareMap.get(DcMotor.class, "3");
    m4 = hardwareMap.get(DcMotor.class, "4");
    s1 = hardWareMap.get(Servo.class, "steer");
    sensor = hardWareMap.get(DistanceSensor.class, "rs");

    Motor motor1 = new Motor(m1, true); //left back
    Motor motor2 = new Motor(m2, false); //right back
    Motor motor3 = new Motor(m3, true); //left front
    Motor motor4 = new Motor(m4, false); //right front
    return new Drive(motor1, motor2, motor3, motor4, s1, sensor);
  }

  @Override
  public void runOpMode() {
    Drive driver = getDriver();
    waitForStart();
    this.running = true;
    driver.startEmergencyBrakeCheck();
    while (opModeIsActive() && running) {
      driver.turnStraight();
      sleep(1000);
      driver.forward();
    }
  }
}
