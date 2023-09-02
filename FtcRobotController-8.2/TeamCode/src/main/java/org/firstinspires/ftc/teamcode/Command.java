package org.firstinspires.ftc.teamcode;

public abstract class Command {
    public abstract void start();
    public abstract void execute();
    public abstract void end();
    public abstract boolean isFinished();
}
