package incognito.teamcode.robot.component.arm;

import static incognito.teamcode.config.WorldSlideConstants.HS_CLAW_WAIT_TIME;
import static incognito.teamcode.config.WorldSlideConstants.HS_DS_CONE_DISTANCE_CM;
import static incognito.teamcode.config.WorldSlideConstants.HS_HINGE_END;
import static incognito.teamcode.config.WorldSlideConstants.HS_HINGE_START;
import static incognito.teamcode.config.WorldSlideConstants.HS_HINGE_WAIT_TIME;
import static incognito.teamcode.config.WorldSlideConstants.HS_LEVER_MID;
import static incognito.teamcode.config.WorldSlideConstants.HS_LEVER_WAIT_TIME;
import static incognito.teamcode.robot.component.arm.HorizontalArm.Position.MANUAL;
import static incognito.teamcode.robot.component.arm.HorizontalArm.Position.OUT;

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
        IN, WAIT_IN, WAIT_OUT, OUT, MANUAL, CLAW_OUT, GROUND
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
        if (position == lastPosition) return;
        if (position == MANUAL) {
            slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slide.setPower(0);
            return;
        } else if (slide.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        switch (position) {
            case IN:
                slide.goToSetPosition(HorizontalSlide.Position.IN);
                lever.goToSetPosition(Lever.HorizontalLeverPosition.IN);
                hinge.goToSetPosition(HorizontalHinge.Position.IN);
                closeClaw();
                break;
            case WAIT_IN:
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
                hinge.goToSetPosition(HorizontalHinge.Position.IN);
                break;
            case WAIT_OUT:
                // no slide
                lever.goToSetPosition(Lever.HorizontalLeverPosition.MID);
                hinge.goToSetPosition(HorizontalHinge.Position.MID);
                openClaw();
                break;
            case OUT:
                extendSlide();
                lever.goToSetPosition(leverOutPositionStorage);
                levelHinge();
                openClaw();
                break;
        }
        lastPosition = currentPosition;
        currentPosition = position;
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
        hingeLow = 0
        hingeHigh = 0.6

        leverLow = 0
        leverHigh = 0.8
         */
        hinge.setPosition(lever.getPosition() / HS_LEVER_MID * HS_HINGE_START);
    }

    public void storeLeverHeight(Lever.HorizontalLeverPosition position) {
        leverOutPositionStorage = position;
        lever.goToSetPosition(leverOutPositionStorage);
        if (getPosition() == OUT || getPosition() == MANUAL) {
            // i dont know what i wanted to do here...
        }
    }

    public void incrementLeverHeight() {
        storeLeverHeight(leverOutPositionStorage.increment());
    }

    public void decrementLeverHeight() {
        storeLeverHeight(leverOutPositionStorage.decrement());
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

    public void update() {
        if (getPosition() == OUT) {
            if (distanceSensor.getDistance(DistanceUnit.CM) < HS_DS_CONE_DISTANCE_CM) {
                slide.setTargetPosition(slide.getPosition());
            }
        }
    }
}
