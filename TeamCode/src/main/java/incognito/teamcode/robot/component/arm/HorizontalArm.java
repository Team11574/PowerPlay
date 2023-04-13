package incognito.teamcode.robot.component.arm;

import static incognito.teamcode.config.WorldSlideConstants.HS_CLAW_WAIT_TIME;
import static incognito.teamcode.config.WorldSlideConstants.HS_DS_CONE_DISTANCE_CM;
import static incognito.teamcode.config.WorldSlideConstants.HS_DS_CONE_SUPER_DISTANCE_CM;
import static incognito.teamcode.config.WorldSlideConstants.HS_HINGE_WAIT_TIME;
import static incognito.teamcode.config.WorldSlideConstants.HS_LEVER_WAIT_TIME;
import static incognito.teamcode.config.WorldSlideConstants.S_RUN_TO_POSITION_POWER;
import static incognito.teamcode.robot.component.arm.HorizontalArm.Position.CLAW_OUT;
import static incognito.teamcode.robot.component.arm.HorizontalArm.Position.GROUND;
import static incognito.teamcode.robot.component.arm.HorizontalArm.Position.MANUAL;
import static incognito.teamcode.robot.component.arm.HorizontalArm.Position.OUT;
import static incognito.teamcode.robot.component.arm.HorizontalArm.Position.SUPER_OUT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import incognito.teamcode.robot.component.servoImplementations.Claw;
import incognito.teamcode.robot.component.servoImplementations.HorizontalHinge;
import incognito.teamcode.robot.component.servoImplementations.Lever;
import incognito.teamcode.robot.component.slide.HorizontalSlide;

public class HorizontalArm extends Arm {

    public enum Position {
        IN, WAIT_IN, WAIT_OUT, OUT, MANUAL, CLAW_OUT, GROUND, SUPER_OUT, UP
    }

    public HorizontalSlide slide;
    public Lever lever;
    public HorizontalHinge hinge;
    public Claw claw;
    public DistanceSensor distanceSensor;
    private Position currentPosition;
    private Position lastPosition = null;
    public Lever.HorizontalLeverPosition leverOutPositionStorage = Lever.HorizontalLeverPosition.OUT;

    public HorizontalArm(HorizontalSlide slide, Lever lever, HorizontalHinge hinge, Claw claw, DistanceSensor distanceSensor) {
        this.slide = slide;
        this.lever = lever;
        this.hinge = hinge;
        this.claw = claw;
        this.distanceSensor = distanceSensor;
        goToPosition(Position.IN);
    }

    /*
    - extending
    - stopping for unknown reasons
    - stopping at a cone
    - moving lever to different heights

    button(s) to increase/decrease current height by 1
    auto decrement height on next run?
    queue what you want next height to be?
    set next height as its going out?

     */

    public void goToPosition(Position position) {
        if (position == currentPosition) return;
        lastPosition = currentPosition;
        currentPosition = position;
        if (position == MANUAL) {
            slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slide.setPower(0);
            return;
        } else if (slide.getMode() == DcMotor.RunMode.RUN_USING_ENCODER || Math.abs(slide.getPower()) < 0.1) {
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(S_RUN_TO_POSITION_POWER);
        }
        switch (position) {
            case IN:
                slide.goToSetPosition(HorizontalSlide.Position.IN);
                lever.goToSetPosition(Lever.HorizontalLeverPosition.IN);
                hinge.goToSetPosition(HorizontalHinge.Position.IN);
                closeClaw();
                break;
            case WAIT_IN:
                if (lastPosition == Position.IN) {
                    // ignore
                    return;
                }
                slide.goToSetPosition(HorizontalSlide.Position.IN);
                lever.goToSetPosition(Lever.HorizontalLeverPosition.MID);
                hinge.goToSetPosition(HorizontalHinge.Position.IN);
                closeClaw();
                break;
            case CLAW_OUT:
                // no slide
                lever.goToSetPosition(leverOutPositionStorage);
                levelHinge();
                openClaw();
                break;
            case GROUND:
                slide.goToSetPosition(HorizontalSlide.Position.IN);
                lever.goToSetPosition(Lever.HorizontalLeverPosition.OUT);
                hinge.goToSetPosition(HorizontalHinge.Position.GROUND);
                closeClaw();
                break;
            case WAIT_OUT:
                if (lastPosition == Position.OUT) {
                    // ignore
                    return;
                }
                // no slide
                lever.goToSetPosition(Lever.HorizontalLeverPosition.MID);
                hinge.goToSetPosition(HorizontalHinge.Position.MID);
                openClaw();
                break;
            case SUPER_OUT:
            case OUT:
                extendSlide();
                lever.goToSetPosition(leverOutPositionStorage);
                levelHinge();
                openClaw();
                break;
            case UP:
                slide.goToSetPosition(HorizontalSlide.Position.IN);
                lever.goToSetPosition(Lever.HorizontalLeverPosition.IN);
                hinge.goToSetPosition(HorizontalHinge.Position.MID);
                openClaw();
                break;
        }
    }

    public void extendSlide() {
        slide.goToSetPosition(HorizontalSlide.Position.OUT);
    }

    public void openClaw() {
        claw.open();
    }

    public void closeClaw() {
        claw.close();
    }

    public void toggleClaw() {
        claw.toggle();
    }

    public void levelHinge() {
        /*
        hingeLow = 0.46
        hinge2 = 0.35
        hinge3 = 0.29
        hinge4 = 0.15
        hinge5 = 0.08
        hingeHigh = 0.6
        leverLow = 0.04
        lever2 = 0.15
        lever3 = 0.26
        lever4 = 0.38
        lever5 = 0.46
        leverHigh = 0.8

        lever @ 0 => hinge @ 0.6
        lever @ 0.8 => hinge @ 0

         */
        //hinge.setPosition((0.8 - lever.getPosition() + 0.05)/0.8 * 0.6);
        hinge.goToSetPosition(HorizontalHinge.Position.valueOf(leverOutPositionStorage.name()));
    }

    public void setPower(double power) {
        goToPosition(MANUAL);
        slide.setPower(power);
    }

    public void storeLeverHeight(Lever.HorizontalLeverPosition position) {
        if (position == Lever.HorizontalLeverPosition.IN) {
            position = Lever.HorizontalLeverPosition.OUT;
        }
        leverOutPositionStorage = position;
        if (getPosition() == OUT || getPosition() == CLAW_OUT || getPosition() == GROUND || getPosition() == MANUAL || getPosition() == SUPER_OUT) {
            lever.goToSetPosition(leverOutPositionStorage);
            levelHinge();
        }
    }

    public void incrementLeverHeight() {
        Lever.HorizontalLeverPosition pos = leverOutPositionStorage.increment();
        if (pos == Lever.HorizontalLeverPosition.IN) {
            pos = Lever.HorizontalLeverPosition.MID;
        }
        storeLeverHeight(pos);
    }

    public void decrementLeverHeight() {
        Lever.HorizontalLeverPosition pos = leverOutPositionStorage.decrement();
        if (pos == Lever.HorizontalLeverPosition.IN) {
            pos = Lever.HorizontalLeverPosition.OUT;
        }
        storeLeverHeight(pos);
    }

    public boolean atPosition() {
        return slide.atSetPosition()
                && lever.atSetPosition(HS_LEVER_WAIT_TIME)
                && hinge.atSetPosition(HS_HINGE_WAIT_TIME)
                && claw.atSetPosition(HS_CLAW_WAIT_TIME);
    }

    public Position getPosition() {
        return currentPosition;
    }

    public Position getLastPosition() {
        return lastPosition;
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    public void update() {
        if (getPosition() == OUT) {
            if (getDistance() < HS_DS_CONE_DISTANCE_CM) {
                slide.setPower(0);
                //slide.setTargetPosition(slide.getPosition() + HS_CONE_JUMP_DISTANCE);
                closeClaw();
            }
        } else if (getPosition() == SUPER_OUT) {
            if (getDistance() < HS_DS_CONE_SUPER_DISTANCE_CM) {
                slide.setPower(0);
            }
        }
    }
}
