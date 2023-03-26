package incognito.teamcode.robot.component.arm;

import incognito.teamcode.robot.component.servoImplementations.Claw;
import incognito.teamcode.robot.component.servoImplementations.Hinge;
import incognito.teamcode.robot.component.servoImplementations.Lever;
import incognito.teamcode.robot.component.slide.VerticalSlide;

public class VerticalArm extends Arm {

    public enum Position {
        INTAKE, LOW, MEDIUM, HIGH
    }

    public VerticalSlide slide;
    public Lever lever;
    public Hinge hinge;
    public Claw claw;
    private Position currentPosition;

    public VerticalArm(VerticalSlide slide, Lever lever, Hinge hinge, Claw claw) {
        this.slide = slide;
        this.lever = lever;
        this.hinge = hinge;
        this.claw = claw;
        goToPosition(Position.INTAKE);
    }

    public void goToPosition(Position position) {
        switch (position) {
            case INTAKE:
                slide.goToSetPosition(VerticalSlide.Position.INTAKE);
                lever.goToSetPosition(Lever.VerticalLeverPosition.INTAKE);
                hinge.goToSetPosition(Hinge.Position.INTAKE);
                openClaw();
                break;
            case LOW:
                slide.goToSetPosition(VerticalSlide.Position.LOW);
                lever.goToSetPosition(Lever.VerticalLeverPosition.LOW);
                hinge.goToSetPosition(Hinge.Position.LOW_UP);
                closeClaw();
                break;
            case MEDIUM:
                slide.goToSetPosition(VerticalSlide.Position.MEDIUM);
                lever.goToSetPosition(Lever.VerticalLeverPosition.MEDIUM);
                hinge.goToSetPosition(Hinge.Position.MEDIUM_UP);
                closeClaw();
                break;
            case HIGH:
                slide.goToSetPosition(VerticalSlide.Position.HIGH);
                lever.goToSetPosition(Lever.VerticalLeverPosition.HIGH);
                hinge.goToSetPosition(Hinge.Position.HIGH_UP);
                closeClaw();
                break;
        }
        currentPosition = position;
    }

    public void openClaw() {
        claw.open();
    }

    public void closeClaw() {
        claw.close();
    }

    public void levelHinge() {

    }

    public void hingeUp() {
        switch (currentPosition) {
            case INTAKE:
                hinge.goToSetPosition(Hinge.Position.INTAKE);
                break;
            case LOW:
                hinge.goToSetPosition(Hinge.Position.LOW_UP);
                break;
            case MEDIUM:
                hinge.goToSetPosition(Hinge.Position.MEDIUM_UP);
                break;
            case HIGH:
                hinge.goToSetPosition(Hinge.Position.HIGH_UP);
                break;
        }
    }

    public void hingeDown() {
        switch (currentPosition) {
            case INTAKE:
                hinge.goToSetPosition(Hinge.Position.INTAKE);
                break;
            case LOW:
                hinge.goToSetPosition(Hinge.Position.LOW_DOWN);
                break;
            case MEDIUM:
                hinge.goToSetPosition(Hinge.Position.MEDIUM_DOWN);
                break;
            case HIGH:
                hinge.goToSetPosition(Hinge.Position.HIGH_DOWN);
                break;
        }
    }

    public Position getPosition() {
        return currentPosition;
    }
}
