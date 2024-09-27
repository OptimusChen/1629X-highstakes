#pragma once

class Arm {
    public:
        int motorPort;
        int rotPort;
        int target;

        Arm(int motorPort, int rotPort);

        void prime();
        void with_ring();
        void stow();
        void move();
        float arm_pos();
};