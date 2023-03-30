package incognito.teamcode.robot.component.arm;

import static incognito.teamcode.config.WorldSlideConstants.HS_CLAW_WAIT_TIME;
import static incognito.teamcode.config.WorldSlideConstants.HS_DS_CONE_DISTANCE_CM;
import static incognito.teamcode.config.WorldSlideConstants.HS_HINGE_WAIT_TIME;
import static incognito.teamcode.config.WorldSlideConstants.HS_LEVER_WAIT_TIME;
import static incognito.teamcode.robot.component.arm.HorizontalArm.Position.HOLD_CONE;
import static incognito.teamcode.robot.component.arm.HorizontalArm.Position.MANUAL;
import static incognito.teamcode.robot.component.arm.HorizontalArm.Position.OUT;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import incognito.teamcode.robot.component.servoImplementations.Claw;
import incognito.teamcode.robot.component.servoImplementations.Hinge;
import incognito.teamcode.robot.component.servoImplementations.Lever;
import incognito.teamcode.robot.component.slide.HorizontalSlide;

public class HorizontalArm extends Arm {

    public enum Position {
        IN, HOLD_CONE, OUT, MANUAL
    }

    public HorizontalSlide slide;
    public Lever lever;
    public Hinge hinge;
    public Claw claw;
    public DistanceSensor distanceSensor;
    private Position currentPosition;
    public Lever.HorizontalLeverPosition leverOutPositionStorage = Lever.HorizontalLeverPosition.IN;

    public HorizontalArm(HorizontalSlide slide, Lever lever, Hinge hinge, Claw claw, DistanceSensor distanceSensor) {
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
        switch (position) {
            case IN:
            case HOLD_CONE:
                slide.goToSetPosition(HorizontalSlide.Position.IN);
                closeClaw();
                break;
            case OUT:
                extendSlide();
                openClaw();
                break;
        }
        lever.goToSetPosition(leverOutPositionStorage);
        if (position != HOLD_CONE && position != MANUAL) {
            levelHinge();
        } else {
            // do later, unlevel hinge
        }
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

    public void levelHinge() {

    }

    public void storeLeverHeight(Lever.HorizontalLeverPosition position) {
        leverOutPositionStorage = position;
        lever.goToSetPosition(leverOutPositionStorage);
        if (getPosition() == OUT || getPosition() == MANUAL) {

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

    public void update() {
        if (getPosition() == OUT) {
            if (distanceSensor.getDistance(DistanceUnit.CM) < HS_DS_CONE_DISTANCE_CM) {
                slide.setTargetPosition(slide.getPosition());
            }
        }
    }
}
