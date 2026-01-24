#pragma once

//define actuator board pins to be imported where needed
namespace actuator_board_pins {
    struct Layout {


        int ACTUATOR_1;
        int ACTUATOR_2;
        int ACTUATOR_3;
        int ACTUATOR_4;
        int ACTUATOR_5;
        int ACTUATOR_6;
        int ACTUATOR_7;
        int ACTUATOR_8;
        int ACTUATOR_9;
        int ACTUATOR_10;

        int CURRENT_SENSE_1;
        int CURRENT_SENSE_2;
        int CURRENT_SENSE_3;
        int CURRENT_SENSE_4;
        int CURRENT_SENSE_5;
        int CURRENT_SENSE_6;
        int CURRENT_SENSE_7;
        int CURRENT_SENSE_8;
        int CURRENT_SENSE_9;
        int CURRENT_SENSE_10;

    };

    const Layout Actuator_Board = {
        7, 5, 48, 21, 36, 6, 4, 47, 14, 35,

        18, 9, 13, 11, 1, 17, 8, 10, 12, 2
    };
}
