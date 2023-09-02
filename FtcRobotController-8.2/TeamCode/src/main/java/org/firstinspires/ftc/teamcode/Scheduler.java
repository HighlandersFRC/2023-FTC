package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.ftccommon.CommandList;

import java.util.ArrayList;
import java.util.List;

public class Scheduler {
    List<Command> commandList = new ArrayList<>();
    int currentCommand;
    Command command = new Command() {
        @Override
        public void start() {

        }

        @Override
        public void execute() {

        }

        @Override
        public void end() {

        }

        @Override
        public boolean isFinished() {
            return false;
        }
    };
    public void add(Command command){
        commandList.add(command);
        command.start();
    }

    public void update(){
        int length = commandList.size();
        for (int i = 0; i < length; i++){
            if (commandList.get(i).isFinished()){
                command.end();
            }
        }
    }
}