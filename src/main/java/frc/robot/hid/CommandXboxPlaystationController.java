// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hid;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * For a controller that looks like a playstation controller but acts like an xbox.
 * A subclass of {@link XboxController} with {@link Trigger} factories for command-based.
 *
 * @see XboxController
 */
@SuppressWarnings({"MethodName", "unused"})
public class CommandXboxPlaystationController extends CommandGenericHID {
    private final XboxController hid;

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public CommandXboxPlaystationController(int port) {
        super(port);
        hid = new XboxController(port);
    }

    /**
     * Get the underlying GenericHID object.
     *
     * @return the wrapped GenericHID object
     */
    @Override
    public XboxController getHID() {
        return hid;
    }

    /**
     * Constructs an event instance around the left bumper's digital signal.
     *
     * @return an event instance representing the left bumper's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #leftBumper(EventLoop)
     */
    public Trigger leftBumper() {
        return leftBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the left bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right bumper's digital signal attached to the given
     *     loop.
     */
    public Trigger leftBumper(EventLoop loop) {
        return hid.leftBumper(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the right bumper's digital signal.
     *
     * @return an event instance representing the right bumper's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #rightBumper(EventLoop)
     */
    public Trigger rightBumper() {
        return rightBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the right bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left bumper's digital signal attached to the given
     *     loop.
     */
    public Trigger rightBumper(EventLoop loop) {
        return hid.rightBumper(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the left stick button's digital signal.
     *
     * @return an event instance representing the left stick button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #leftStick(EventLoop)
     */
    public Trigger leftStick() {
        return leftStick(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the left stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left stick button's digital signal attached to the
     *     given loop.
     */
    public Trigger leftStick(EventLoop loop) {
        return hid.leftStick(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the right stick button's digital signal.
     *
     * @return an event instance representing the right stick button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #rightStick(EventLoop)
     */
    public Trigger rightStick() {
        return rightStick(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the right stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right stick button's digital signal attached to the
     *     given loop.
     */
    public Trigger rightStick(EventLoop loop) {
        return hid.rightStick(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the A button's digital signal.
     *
     * @return an event instance representing the A button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #x(EventLoop)
     */
    public Trigger x() {
        return x(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the A button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the A button's digital signal attached to the given
     *     loop.
     */
    public Trigger x(EventLoop loop) {
        return hid.a(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the B button's digital signal.
     *
     * @return an event instance representing the B button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #circle(EventLoop)
     */
    public Trigger circle() {
        return circle(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the B button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the B button's digital signal attached to the given
     *     loop.
     */
    public Trigger circle(EventLoop loop) {
        return hid.b(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the X button's digital signal.
     *
     * @return an event instance representing the X button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #square(EventLoop)
     */
    public Trigger square() {
        return square(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the X button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the X button's digital signal attached to the given
     *     loop.
     */
    public Trigger square(EventLoop loop) {
        return hid.x(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the Y button's digital signal.
     *
     * @return an event instance representing the Y button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #triangle(EventLoop)
     */
    public Trigger triangle() {
        return triangle(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the Y button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the Y button's digital signal attached to the given
     *     loop.
     */
    public Trigger triangle(EventLoop loop) {
        return hid.y(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @return an event instance representing the start button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #options(EventLoop)
     */
    public Trigger options() {
        return options(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the start button's digital signal attached to the given
     *     loop.
     */
    public Trigger options(EventLoop loop) {
        return hid.start(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @return an event instance representing the back button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #share(EventLoop)
     */
    public Trigger share() {
        return share(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the back button's digital signal attached to the given
     *     loop.
     */
    public Trigger share(EventLoop loop) {
        return hid.back(loop).castTo(Trigger::new);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param loop the event loop instance to attach the Trigger to.
     * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
     *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
     * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
     *     threshold, attached to the given event loop
     */
    public Trigger leftTrigger(EventLoop loop, double threshold) {
        return hid.leftTrigger(threshold, loop).castTo(Trigger::new);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
     *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
     * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
     *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *     button loop}.
     */
    public Trigger leftTrigger(double threshold) {
        return leftTrigger(CommandScheduler.getInstance().getDefaultButtonLoop(), threshold);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @return a Trigger instance that is true when the left trigger's axis exceeds 0.5, attached to
     *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger leftTrigger() {
        return leftTrigger(0.5);
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
     *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
     * @param loop the event loop instance to attach the Trigger to.
     * @return a Trigger instance that is true when the right trigger's axis exceeds the provided
     *     threshold, attached to the given event loop
     */
    public Trigger rightTrigger(double threshold, EventLoop loop) {
        return hid.rightTrigger(threshold, loop).castTo(Trigger::new);
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
     *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
     * @return a Trigger instance that is true when the right trigger's axis exceeds the provided
     *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *     button loop}.
     */
    public Trigger rightTrigger(double threshold) {
        return rightTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @return a Trigger instance that is true when the right trigger's axis exceeds 0.5, attached to
     *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger rightTrigger() {
        return rightTrigger(0.5);
    }

    /**
     * Get the X axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftX() {
        return hid.getLeftX();
    }

    /**
     * Get the X axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightX() {
        return hid.getRightX();
    }

    /**
     * Get the Y axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftY() {
        return hid.getLeftY();
    }

    /**
     * Get the Y axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightY() {
        return hid.getRightY();
    }

    /**
     * Get the left trigger (LT) axis value of the controller. Note that this axis is bound to the
     * range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getLeftTriggerAxis() {
        return hid.getLeftTriggerAxis();
    }

    /**
     * Get the right trigger (RT) axis value of the controller. Note that this axis is bound to the
     * range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getRightTriggerAxis() {
        return hid.getRightTriggerAxis();
    }
}
