#include <SimplyAtomic.h>  // For the position read, to avoid missed counts
#include <Servo.h> // Pi Library for Servo Control

// ---------- Pen servo (continuous rotation, timed spool) ----------
#define SERVO_PIN 6                       // GPIO 6: PWM Pin for the Servo

const int PEN_DOWN_US = 700;              // Pulse Width command for full speed Clockwise Rotation
const int PEN_UP_US   = 2300;             // Pulse Width command for full speed Counter-Clockwise Rotation
const unsigned long PEN_DOWN_MS = 300;    // Driving time to lower the pen
const unsigned long PEN_UP_MS   = 300;    // Driving time to raise the pen

volatile bool pen_down_cmd = false;       // PEN COMMAND SET BY MAIN MOTOR CONTROL LOOP: true = pen should be down, false = pen should be up
bool pen_is_down = false;                 // Where pen loop thinks pen is (On boot = false = up)
bool pen_moving  = false;
unsigned long pen_move_end = 0;           // Used in arithmetic for how long pen servo is driven
Servo pen; // Variable called pen, its type is servo :O

// --- Utility Pins ---
#define BUTTON_PIN 5 // on-off button

// ---------- Motor 1 pins ----------
#define ENCA 0     // interrupt pin encoder A
#define ENCB 1     // interrupt pin encoder B
#define PWM_pin 2  // pin to send pwm value to motor driver
#define IN1 3      // motor driver direction pins
#define IN2 4

// ---------- Motor 2 pins (NOTE: Remember motor 2 is upside down on the mount. This is not reflected in the IN3 & IN4 pins or it breaks.) ----------
#define ENCA_2 11
#define ENCB_2 12
#define PWM_pin_2 13
#define IN3 14
#define IN4 15

// ---------- Motor 1 variables ----------
volatile int posi = 0;     // volatile, integer because counts are discrete
float ref = 0;            // Active encoder point motors aim for
float kp = 15;            // proportional gain
int pos = 0;               // output encoder position from the gearbox
int e = 0;                 // error between ref and pos
int u_amplitude = 0;       // amplitude of the PID calculated control signal
float u_f = 0;             // float version of u_amplitude
int u_max_amplitude = 0; // control signal amplitude (must be < 256)
int u_sign = 0;            // required sign (Drive Direction) of control signal
long time_start = 0;
const int shape1_m1[] = {-73, -73, -74, -75, -76, -77, -78, -79, -80, -81, -82, -83, -84, -85, -87, -88, -89, -91, -92, -94, -95, -97, -98, -100, -102, -104, -105, -107, -109, -111, -112, -114, -116, -118, -119, -121, -123, -125, -127, -128, -130, -132, -134, -136, -137, -139, -141, -143, -145, -146, -148, -150, -152, -154, -156, -158, -159, -161, -163, -165, -167, -169, -171, -173, -174, -176, -178, -180, -182, -184, -186, -188, -190, -192, -194, -196, -198, -200, -202, -204, -206, -208, -209, -211, -213, -214, -216, -217, -219, -220, -221, -223, -224, -225, -226, -227, -228, -230, -231, -232, -233, -232, -231, -231, -230, -229, -228, -227, -227, -226, -225, -224, -223, -222, -221, -220, -219, -218, -217, -216, -215, -213, -212, -211, -210, -209, -208, -207, -206, -204, -203, -202, -201, -200, -200, -199, -198, -197, -196, -195, -195, -194, -193, -193, -192, -191, -191, -190, -190, -190, -189, -189, -188, -188, -188, -188, -187, -187, -187, -187, -187, -187, -187, -187, -188, -188, -188, -188, -189, -189, -190, -190, -191, -192, -192, -193, -194, -195, -196, -197, -198, -200, -201, -202, -204, -205, -206, -208, -209, -211, -212, -214, -216, -217, -219, -221, -223, -226, -228, -231, -234, -229, -225, -222, -218, -215, -212, -208, -205, -202, -199, -196, -193, -189, -186, -183, -179, -176, -173, -169, -165, -162, -158, -154, -151, -147, -143, -140, -136, -133, -129, -126, -123, -119, -116, -113, -110, -106, -103, -100, -97, -94, -91, -88, -85, -82, -79, -76, -73, -70, -67, -64, -61, -58, -55, -52, -50, -47, -44, -41, -38, -35, -33, -30, -27, -24, -21, -19, -16, -13, -10, -8, -5, -2, 1, 3, 6, 9, 11, 14, 16, 19, 21, 23, 25, 27, 29, 31, 33, 35, 36, 38, 39, 41, 42, 44, 45, 46, 48, 49, 50, 51, 51, 51, 52, 52, 52, 52, 53, 53, 53, 53, 53, 54, 54, 54, 54, 54, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 54, 54, 54, 53, 53, 53, 52, 52, 51, 51, 50, 49, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 36, 35, 33, 32, 30, 29, 27, 25, 24, 22, 20, 18, 16, 14, 12, 9, 7, 5, 2, -1, -3, -6, -9, -11, -14, -17, -20, -23, -26, -29, -32, -35, -37, -40, -43, -45, -48, -50, -52, -55, -57, -59, -61, -64, -66, -68, -70
}; // shape 1 points for motor 1
const int shape2_m1[] = {-41, -42, -42, -43, -44, -45, -46, -47, -48, -49, -50, -51, -52, -53, -54, -55, -56, -58, -59, -60, -62, -63, -65, -66, -68, -70, -71, -73, -74, -76, -78, -79, -81, -82, -84, -86, -87, -89, -90, -92, -93, -95, -97, -98, -100, -101, -103, -104, -106, -107, -109, -111, -112, -114, -115, -117, -118, -120, -121, -123, -125, -126, -128, -129, -131, -132, -134, -135, -137, -139, -140, -142, -143, -145, -146, -148, -150, -151, -153, -154, -156, -157, -159, -161, -162, -164, -165, -167, -169, -170, -172, -173, -175, -177, -178, -180, -182, -183, -185, -187, -188, -190, -192, -193, -195, -197, -198, -200, -202, -203, -205, -207, -208, -210, -211, -213, -214, -216, -217, -218, -220, -221, -222, -223, -224, -225, -226, -227, -228, -229, -230, -231, -232, -233, -233, -232, -231, -230, -229, -228, -227, -226, -225, -224, -223, -222, -220, -219, -218, -217, -215, -214, -213, -211, -209, -208, -206, -204, -203, -201, -199, -198, -196, -194, -192, -191, -189, -187, -186, -184, -182, -181, -179, -177, -176, -174, -172, -171, -169, -167, -166, -164, -163, -161, -159, -158, -156, -155, -153, -152, -150, -149, -147, -146, -144, -143, -141, -140, -138, -137, -135, -134, -132, -131, -130, -128, -127, -126, -124, -123, -121, -120, -119, -118, -116, -115, -114, -112, -111, -110, -109, -108, -106, -105, -104, -103, -102, -101, -99, -98, -97, -96, -95, -94, -93, -92, -91, -90, -89, -88, -87, -86, -85, -84, -83, -82, -82, -81, -80, -79, -79, -78, -78, -77, -77, -76, -76, -75, -75, -74, -74, -74, -73, -73, -72, -72, -72, -71, -71, -70, -69, -68, -67, -67, -66, -65, -64, -63, -62, -61, -60, -59, -59, -57, -56, -55, -54, -53, -52, -51, -50, -48, -47, -46, -45, -44, -43, -42, -41, -40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -29, -28, -27, -26, -25, -25, -24, -23, -23, -22, -21, -21, -20, -20, -19, -18, -18, -17, -17, -16, -16, -16, -15, -15, -14, -14, -14, -13, -13, -13, -13, -12, -12, -12, -12, -12, -12, -12, -11, -11, -11, -11, -11, -12, -12, -12, -12, -12, -12, -12, -13, -13, -13, -14, -14, -15, -15, -15, -16, -17, -17, -18, -18, -19, -20, -21, -21, -22, -23, -24, -25, -26, -27, -27, -28, -29, -30, -31, -32, -32, -33, -34, -35, -36, -36, -37, -38, -39, -40
}; // shape 2 points for motor 1
const int shape3_m1[] = {-194, -193, -193, -193, -192, -192, -192, -191, -191, -190, -190, -190, -189, -189, -188, -188, -187, -187, -186, -185, -185, -184, -183, -183, -182, -181, -181, -180, -179, -178, -177, -177, -176, -175, -174, -173, -172, -171, -170, -169, -168, -167, -166, -164, -163, -162, -161, -160, -158, -157, -156, -154, -153, -152, -150, -149, -147, -146, -144, -143, -141, -140, -138, -137, -135, -133, -132, -130, -128, -127, -125, -123, -121, -120, -118, -116, -114, -112, -110, -109, -107, -105, -103, -101, -99, -97, -95, -93, -91, -89, -87, -85, -83, -81, -79, -77, -75, -73, -71, -69, -67, -65, -63, -61, -59, -57, -55, -53, -51, -49, -47, -45, -43, -41, -39, -37, -35, -33, -31, -29, -27, -25, -23, -21, -20, -18, -16, -14, -12, -10, -8, -7, -5, -3, -1, 0, 2, 4, 5, 7, 9, 10, 12, 13, 15, 17, 18, 20, 21, 22, 24, 25, 26, 28, 29, 30, 31, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 43, 44, 45, 45, 46, 47, 47, 48, 48, 48, 49, 49, 49, 50, 50, 50, 50, 50, 50, 50, 50, 49, 49, 49, 49, 48, 48, 47, 47, 46, 45, 44, 44, 43, 42, 41, 40, 39, 38, 36, 35, 34, 32, 31, 29, 28, 26, 25, 23, 21, 19, 17, 15, 14, 12, 9, 7, 5, 3, 1, -1, -4, -6, -8, -11, -13, -16, -18, -21, -23, -26, -28, -31, -33, -36, -39, -41, -44, -46, -49, -51, -54, -57, -59, -62, -64, -67, -69, -72, -74, -77, -79, -82, -84, -86, -89, -91, -93, -96, -98, -100, -102, -104, -106, -108, -110, -112, -114, -116, -118, -120, -122, -124, -126, -127, -129, -131, -132, -134, -136, -137, -139, -140, -142, -143, -145, -146, -147, -149, -150, -151, -153, -154, -155, -156, -157, -158, -160, -161, -162, -163, -164, -165, -166, -167, -168, -169, -169, -170, -171, -172, -173, -174, -174, -175, -176, -177, -177, -178, -179, -179, -180, -181, -181, -182, -183, -183, -184, -184, -185, -185, -186, -186, -187, -187, -188, -188, -189, -189, -189, -190, -190, -191, -191, -191, -192, -192, -192, -192, -193, -193, -193, -194, -194, -194, -194, -194, -195, -195, -195, -195, -195, -195, -195, -196, -196, -196, -196, -196, -196, -196, -196, -196, -196, -196, -196, -196, -196, -196, -196, -196, -196, -195, -195, -195, -195, -195, -195, -194, -194, -194, -194
}; // shape 3 points for motor 1
int point_index = 0;            // which setpoint motor is currently targeting
const float time_per_point[] = {7, 5.25, 4};    // ms to spend on each encoder point
float e_sum = 0; // Integral sum
float ki = 90; // Integral control gain
const float i_max = 70; // Integral control signal limiter. Prevents extreme buildup and overshoot.
float kd = 0;     // Derivative control gain
int e_prev = 0;      // Arithmetic for Derivative error
float d_term = 0;    // filtered de/dt (counts/s)
float kv = 0;
int pos_prev = 0;

// ---------- Motor 2 variables (independent of motor 1) ----------
volatile int posi_2 = 0;
float ref_2 = 0;
float kp_2 = 15;
int pos_2 = 0;
int e_2 = 0;
int u_amplitude_2 = 0;
float u_f_2 = 0;
int u_max_amplitude_2 = 0;
int u_sign_2 = 0;
long time_start_2 = 0;
const int shape1_m2[] = {850, 848, 846, 845, 843, 841, 839, 837, 835, 833, 831, 829, 827, 824, 822, 819, 816, 813, 810, 807, 804, 801, 797, 794, 790, 786, 783, 779, 775, 771, 768, 764, 760, 756, 752, 748, 745, 741, 737, 733, 729, 725, 721, 717, 713, 709, 705, 701, 697, 693, 689, 685, 681, 677, 673, 669, 665, 660, 656, 652, 648, 644, 639, 635, 631, 626, 622, 618, 613, 609, 604, 600, 595, 591, 587, 582, 577, 573, 569, 564, 560, 556, 552, 548, 544, 540, 537, 533, 530, 527, 524, 521, 518, 516, 513, 510, 508, 505, 502, 500, 497, 496, 495, 494, 493, 492, 491, 490, 489, 488, 486, 485, 484, 482, 481, 479, 478, 476, 474, 472, 470, 467, 465, 463, 460, 458, 455, 452, 450, 447, 444, 441, 438, 435, 433, 429, 426, 423, 420, 417, 414, 410, 407, 403, 400, 396, 393, 389, 386, 382, 378, 374, 370, 366, 362, 358, 354, 349, 345, 341, 336, 331, 327, 322, 317, 312, 307, 302, 296, 291, 285, 280, 274, 268, 262, 256, 249, 243, 236, 230, 223, 216, 210, 203, 196, 190, 183, 177, 171, 164, 158, 151, 145, 138, 131, 124, 117, 110, 101, 92, 83, 93, 102, 110, 118, 125, 132, 139, 146, 152, 159, 165, 172, 178, 185, 191, 198, 204, 211, 218, 225, 231, 238, 245, 251, 258, 264, 270, 276, 282, 288, 293, 299, 304, 310, 315, 320, 325, 330, 334, 339, 344, 348, 353, 357, 361, 365, 370, 374, 378, 382, 386, 389, 393, 397, 401, 404, 408, 411, 415, 418, 421, 425, 428, 431, 434, 437, 440, 443, 446, 449, 452, 455, 458, 461, 463, 466, 468, 471, 473, 476, 478, 480, 482, 484, 485, 487, 489, 490, 492, 493, 494, 495, 497, 498, 499, 500, 501, 502, 503, 504, 507, 509, 512, 514, 517, 519, 522, 525, 528, 530, 533, 536, 540, 543, 546, 550, 554, 558, 562, 566, 570, 574, 579, 583, 587, 592, 596, 601, 605, 610, 614, 618, 623, 627, 631, 635, 640, 644, 648, 652, 656, 661, 665, 669, 673, 677, 681, 685, 689, 693, 697, 701, 705, 709, 713, 717, 721, 725, 728, 732, 736, 740, 744, 748, 751, 755, 759, 763, 766, 770, 774, 777, 781, 785, 788, 792, 795, 799, 802, 806, 809, 812, 815, 817, 820, 823, 825, 827, 830, 832, 834, 836, 838, 840, 841, 843, 845, 847, 848
}; // shape 1 points for motor 2
const int shape2_m2[] = {831, 830, 829, 827, 826, 825, 824, 822, 821, 819, 818, 816, 815, 813, 811, 810, 808, 806, 804, 802, 799, 797, 795, 792, 790, 787, 784, 782, 779, 776, 774, 771, 768, 766, 763, 760, 757, 754, 752, 749, 746, 743, 740, 738, 735, 732, 729, 726, 723, 720, 718, 715, 712, 709, 706, 703, 700, 697, 694, 691, 688, 685, 682, 679, 676, 673, 670, 667, 664, 661, 657, 654, 651, 648, 645, 642, 639, 635, 632, 629, 626, 622, 619, 616, 613, 609, 606, 603, 599, 596, 593, 589, 586, 582, 579, 575, 572, 568, 565, 561, 558, 554, 551, 547, 543, 540, 536, 533, 529, 525, 522, 518, 515, 511, 508, 504, 501, 498, 495, 493, 490, 487, 485, 482, 480, 478, 475, 473, 471, 469, 467, 465, 463, 461, 460, 460, 461, 461, 461, 462, 462, 462, 462, 463, 463, 463, 464, 464, 464, 465, 465, 465, 465, 466, 466, 466, 467, 467, 467, 467, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 467, 467, 467, 467, 466, 466, 466, 465, 465, 464, 464, 464, 463, 463, 462, 462, 461, 460, 460, 459, 459, 458, 457, 456, 456, 455, 454, 453, 452, 452, 451, 450, 449, 448, 447, 446, 445, 444, 443, 442, 441, 439, 438, 437, 436, 435, 433, 432, 431, 429, 428, 427, 425, 424, 422, 421, 419, 418, 416, 415, 413, 411, 410, 408, 406, 405, 403, 401, 400, 398, 396, 395, 393, 392, 390, 389, 388, 386, 385, 384, 383, 381, 380, 379, 378, 377, 376, 375, 374, 374, 376, 379, 382, 384, 387, 390, 393, 395, 398, 401, 404, 407, 411, 414, 417, 421, 425, 429, 433, 437, 441, 445, 450, 454, 458, 463, 467, 472, 476, 480, 485, 489, 493, 498, 502, 506, 510, 514, 518, 522, 527, 531, 535, 539, 543, 547, 551, 554, 558, 562, 566, 570, 574, 578, 581, 585, 589, 593, 597, 600, 604, 608, 611, 615, 619, 622, 626, 630, 633, 637, 641, 644, 648, 651, 655, 659, 662, 666, 669, 673, 676, 680, 683, 687, 690, 694, 697, 701, 704, 708, 711, 715, 718, 722, 725, 729, 732, 736, 739, 743, 746, 750, 753, 757, 760, 763, 767, 770, 774, 777, 780, 784, 787, 790, 793, 795, 798, 801, 803, 806, 808, 810, 812, 814, 816, 818, 820, 822, 824, 826, 827, 829
}; // shape 2 points for motor 2
const int shape3_m2[] = {373, 370, 368, 365, 363, 360, 358, 355, 353, 351, 348, 346, 344, 342, 340, 337, 335, 333, 331, 330, 328, 326, 324, 323, 321, 319, 318, 316, 315, 314, 312, 311, 310, 309, 308, 307, 306, 305, 304, 304, 303, 302, 302, 301, 301, 301, 300, 300, 300, 300, 300, 300, 300, 301, 301, 301, 302, 302, 303, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 315, 316, 318, 319, 321, 322, 324, 326, 328, 329, 331, 333, 335, 337, 339, 341, 344, 346, 348, 350, 353, 355, 358, 360, 363, 365, 368, 370, 373, 376, 378, 381, 384, 387, 389, 392, 395, 398, 401, 404, 407, 410, 413, 416, 419, 422, 425, 428, 431, 435, 438, 441, 444, 447, 451, 454, 457, 460, 464, 467, 470, 474, 477, 480, 484, 487, 490, 494, 497, 501, 504, 507, 511, 514, 518, 521, 525, 528, 531, 535, 538, 542, 545, 549, 552, 555, 559, 562, 566, 569, 573, 576, 579, 583, 586, 590, 593, 596, 600, 603, 606, 610, 613, 616, 620, 623, 626, 630, 633, 636, 639, 643, 646, 649, 652, 655, 658, 661, 665, 668, 671, 674, 677, 680, 683, 686, 688, 691, 694, 697, 700, 702, 705, 708, 711, 713, 716, 718, 721, 723, 726, 728, 730, 733, 735, 737, 739, 741, 743, 745, 747, 749, 751, 753, 755, 756, 758, 760, 761, 762, 764, 765, 766, 768, 769, 770, 771, 772, 772, 773, 774, 775, 775, 776, 776, 776, 777, 777, 777, 777, 777, 777, 777, 776, 776, 776, 775, 775, 774, 773, 773, 772, 771, 770, 769, 768, 766, 765, 764, 763, 761, 760, 758, 756, 755, 753, 751, 749, 748, 746, 744, 742, 739, 737, 735, 733, 731, 728, 726, 723, 721, 718, 716, 713, 711, 708, 705, 703, 700, 697, 694, 692, 689, 686, 683, 680, 677, 674, 671, 668, 665, 662, 659, 655, 652, 649, 646, 643, 640, 636, 633, 630, 627, 623, 620, 617, 613, 610, 607, 603, 600, 597, 593, 590, 586, 583, 580, 576, 573, 569, 566, 563, 559, 556, 552, 549, 545, 542, 539, 535, 532, 528, 525, 521, 518, 515, 511, 508, 504, 501, 498, 494, 491, 487, 484, 481, 477, 474, 471, 467, 464, 461, 457, 454, 451, 448, 444, 441, 438, 435, 432, 429, 425, 422, 419, 416, 413, 410, 407, 404, 401, 398, 395, 392, 390, 387, 384, 381, 378, 376
}; // shape 2 points for motor 2
int point_index_2 = 0;            // which setpoint we're currently targeting
float e_sum_2 = 0; //Integral sum
float ki_2 = 90; //Integral control gain
const float i_max_2 = 70;
float kd_2 = 0;
int e_prev_2 = 0;
float d_term_2 = 0;
float kv_2 = 0;
int pos_prev_2 = 0;

// ---------- Shared ----------
int print_interval = 1000;  // how often values are sent to the serial monitor
int interval_count = 0; // arithmetic for print interval
const unsigned long Ts_us = 500;  // derivative sample period (2 ms)
const float d_alpha = 0.4;         // derivative filter: smaller = smoother but more lag
unsigned long t_prev_us = 0;

// ---------- Shape selection ----------
const int* const shapes_m1[] = { shape1_m1, shape2_m1, shape3_m1 };
const int* const shapes_m2[] = { shape1_m2, shape2_m2, shape3_m2 };
const int shape_len[] = { sizeof(shape1_m1) / sizeof(int),
                          sizeof(shape2_m1) / sizeof(int),
                          sizeof(shape3_m1) / sizeof(int) };
const int NUM_SHAPES = 3; // Change depending on number of different shape arrays to draw
int shape = 0;  // Target shape of run
// compile-time check that each shape's two motor arrays are the same length
static_assert(sizeof(shape1_m1) == sizeof(shape1_m2), "shape 1: M1/M2 lengths differ");
static_assert(sizeof(shape2_m1) == sizeof(shape2_m2), "shape 2: M1/M2 lengths differ");
static_assert(sizeof(shape3_m1) == sizeof(shape3_m2), "shape 2: M1/M2 lengths differ");
// point_index at which the pen drops, per shape.
// 0 = lower as soon as homed, X = lower when past encoder point X
const int pen_drop_index[] = { 0, 0, 100 };   // shape 1, 2 and 3


// ---------- Homing / state machine ----------
enum RigState { ST_IDLE, ST_HOMING, ST_HOMED, ST_RUNNING, ST_PAUSED }; // available states
RigState state = ST_IDLE;                   // starts idle (motors off)

// ---------- Final trajectory settling ----------
const int FINAL_TOLERANCE = 3;
const unsigned long FINAL_SETTLE_MS = 200;
bool trajectory_complete = false;
bool final_settle_timing = false;
unsigned long final_settle_since = 0;

const int U_MAX_RUN  = 256;                 // Amplitude limit while drawing shape
const int U_MAX_HOME = 80;                  // Amplitude limit while homing
const int HOME_TOLERANCE = 5;               // Tolerance in encoder counts for state > ST_HOMED
const unsigned long HOME_SETTLE_MS = 7000;  // Settling time for how long motors must be within HOME_TOLERANCE
unsigned long home_in_band_since = 0;
const unsigned long debounce_ms = 30;       // Button press debounce prevents multiple inputs from one button press

// ---------- Derivative Adapative Gain -----------
const float KD_STRAIGHT = 0.3, KD_CORNER = 1.5, KD_CIRCLE = 1.5;   // starting guesses
const int LEAD = 12, LAG = 30;                    // points
const int corners_s1[] = {100, 200, 300, 400}; // Corner Points shape 1 (square)
const int corners_s2[] = {133, 267, 400}; // Corner Points shape 2 (triangle)
const int* const corners[] = { corners_s1, corners_s2 };
const int N_CORNERS_S1 = sizeof(corners_s1) / sizeof(int);
const int N_CORNERS_S2 = sizeof(corners_s2) / sizeof(int);
const int N_CORNERS[] = { N_CORNERS_S1, N_CORNERS_S2 };

float kdSchedule(int idx) {
  float w = 0;
  for (int k = 0; k < N_CORNERS[shape]; k++) {
    int d = idx - corners[shape][k];                  // negative = approaching
    float x = (d < 0) ? -d / (float)LEAD : d / (float)LAG;
    w = max(w, 1.0f - x);                         // triangular, clipped at 0
  }
  return KD_STRAIGHT + w * (KD_CORNER - KD_STRAIGHT);
}

void setup() { // Starts serial connection and initialises connected hardware
  Serial.begin(230400);  // baud rate for communication between USB & raspberry pi pico

  // Servo
  penSetup();

  // button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Motor 1
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  pinMode(PWM_pin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Motor 2
  pinMode(ENCA_2, INPUT);
  pinMode(ENCB_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_2), readEncoder2, RISING);
  pinMode(PWM_pin_2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  time_start = millis();
  time_start_2 = millis();
}

// ============== MAIN CONTROL LOOP ================================================================

void loop() {
  
  // ---- flags for pen control ----
  if (state == ST_HOMED)        pen_down_cmd = (pen_drop_index[shape] == 0);          // drop if no running start
  else if (state == ST_RUNNING) pen_down_cmd = (point_index >= pen_drop_index[shape]); // drop once past the running start
  else if (state == ST_IDLE)    pen_down_cmd = false;
  penLoop();
  printState(); // Print state of main and pen loop every 1s

  // ---- Adaptive Derivative Gain -------
  if (shape <= 1) {
  kd = kdSchedule(point_index);
  kd_2 = kdSchedule(point_index);
  } else {
  kd = KD_CIRCLE;
  kd_2 = KD_CIRCLE;
  }

  // ===================== BUTTON / STATE MACHINE =====================
  if (buttonPressed()) {
    switch (state) {
      case ST_IDLE:                       // BTN Press 1 Starts Homing
        point_index   = 0;
        point_index_2 = 0;
        setLimits(U_MAX_HOME);  // Sets max amplitude to homing limit
        resetController();
        home_in_band_since = millis();
        state = ST_HOMING;
        Serial.println("HOMING");
        break;

      case ST_HOMING:                     // BTN Press during homing aborts to IDLE
        state = ST_IDLE;
        break;                            

      case ST_HOMED:                      // BTN Press 2 draws the shape
        setLimits(U_MAX_RUN);             // Sets max amplitude to running limit
        time_start   = millis();
        time_start_2 = millis();
        trajectory_complete = false;
        final_settle_timing = false;
        final_settle_since = 0;
        state = ST_RUNNING;
        Serial.println("RUNNING");
        break;

      case ST_RUNNING:                    // BTN Allows pause during running
        final_settle_timing = false;
        state = ST_PAUSED;
        Serial.println("PAUSED");
        break;

      case ST_PAUSED:                     // BTN Resumes from the current point
        time_start   = millis();
        time_start_2 = millis();
        final_settle_timing = false;
        resetController();
        state = ST_RUNNING;
        Serial.println("RUNNING");
        break;
    }
  }

  if (state == ST_IDLE || state == ST_PAUSED) {
    setMotor(0, 0, PWM_pin,   IN1, IN2);   // motors off during idle
    setMotor(0, 0, PWM_pin_2, IN3, IN4);
    return;                                // skip control and printing during idle
  }

  // ===================== DERIVATIVE / INTEGRAL TIMING =====================
  bool d_update = false;
  float dt = 0;
  unsigned long now_us = micros();
  if (now_us - t_prev_us >= Ts_us) {  // if time to take a sample, take a sample..
    dt = (now_us - t_prev_us) / 1e6;
    t_prev_us = now_us;
    d_update = true;
  }

  // ===================== MOTOR 1 =====================
  // Advance the reference while drawing, but hold the final point for settling.
  if (state == ST_RUNNING && !trajectory_complete &&
      (millis() - time_start) > time_per_point[shape]) {
    if (point_index < shape_len[shape] - 1) {
      point_index++;
    }
    time_start = millis();
  }
  ref = shapes_m1[shape][point_index]; // sets reference point

  ATOMIC() { // disables interrupts on read. This is a precaution to ensure that printed values to Serial Monitor are equal to the calculated values at this point in time.
    pos = posi;
  }

  e = ref - pos; // calcs error

// state: int pos_prev, pos_prev_2;
  if (d_update) {
    float d_raw = -(pos - pos_prev) / dt;   // = d(error)/dt with ref held constant
    d_term = d_alpha * d_raw + (1 - d_alpha) * d_term;
    pos_prev = pos;

    e_sum += e * dt;
    if (ki > 0) e_sum = constrain(e_sum, -i_max / ki, i_max / ki);
    if (state == ST_RUNNING && (abs(kp * e + kd * d_term) < 90)) e_sum = 0;
  }

  int nxt = min(point_index + 1, shape_len[shape] - 1);
  float ref_vel = (shapes_m1[shape][nxt] - shapes_m1[shape][point_index]) / (time_per_point[shape] / 1000.0f); // counts/s
  u_f = (kp*e + ki*e_sum + kd*d_term + kv*ref_vel);

  // Homing/homed: inside the tolerance band, stop driving and clear the integral
  if (state == ST_HOMED && abs(e) <= HOME_TOLERANCE) {
    u_f = 0;
    e_sum = 0;
  }

  // sets direction of motor drive
  u_sign = 0;
  if (u_f < 0) u_sign = -1;
  if (u_f > 0) u_sign = 1;

  // ensures drive PWM amplitude is within max limits
  u_amplitude = abs(u_f);
  if (u_amplitude > u_max_amplitude) {
    u_amplitude = u_max_amplitude;
  }

  setMotor(u_sign, u_amplitude, PWM_pin, IN1, IN2); // Carries out motor 1 function with these variables

  // ===================== MOTOR 2 =====================
  // Advance the reference while drawing, but do not wrap to the start.
  if (state == ST_RUNNING && !trajectory_complete &&
      (millis() - time_start_2) > time_per_point[shape]) {
    if (point_index_2 < shape_len[shape] - 1) {
      point_index_2++;
    }
    time_start_2 = millis();
  }

  // Start final settling only once both motors have reached their last reference.
  if (state == ST_RUNNING &&
      point_index >= shape_len[shape] - 1 &&
      point_index_2 >= shape_len[shape] - 1) {
    trajectory_complete = true;
  }
  ref_2 = shapes_m2[shape][point_index_2];

  ATOMIC() {
    pos_2 = posi_2;
  }

  e_2 = ref_2 - pos_2;

  if (d_update) {
    float d_raw_2 = -(pos_2 - pos_prev_2) / dt;   // = d(error)/dt with ref held constant
    d_term_2 = d_alpha * d_raw_2 + (1 - d_alpha) * d_term_2;
    pos_prev_2 = pos_2;

    e_sum_2 += e_2 * dt;
    if (ki_2 > 0) e_sum_2 = constrain(e_sum_2, -i_max_2 / ki_2, i_max_2 / ki_2);
    if (state == ST_RUNNING && (abs(kp_2 * e_2 + kd_2 * d_term_2) < 90)) e_sum_2 = 0;
  }

  int nxt_2 = min(point_index_2 + 1, shape_len[shape] - 1);
  float ref_vel_2 = (shapes_m2[shape][nxt_2] - shapes_m2[shape][point_index_2]) / (time_per_point[shape] / 1000.0f); // counts/s
  u_f_2 = (kp_2 * e_2 + ki_2 * e_sum_2 + kd_2 * d_term_2 + kv_2 * ref_vel_2);

  if (state == ST_HOMED && abs(e_2) <= HOME_TOLERANCE) {
    u_f_2 = 0;
    e_sum_2 = 0;
  }

  u_sign_2 = 0;
  if (u_f_2 < 0) u_sign_2 = -1;
  if (u_f_2 > 0) u_sign_2 = 1;

  u_amplitude_2 = abs(u_f_2);
  if (u_amplitude_2 > u_max_amplitude_2) {
    u_amplitude_2 = u_max_amplitude_2;
  }

  setMotor(u_sign_2, u_amplitude_2, PWM_pin_2, IN3, IN4);

  // ===================== FINAL TRAJECTORY SETTLING =====================
  if (state == ST_RUNNING && trajectory_complete) {
    bool in_band = (abs(e) <= FINAL_TOLERANCE &&
                    abs(e_2) <= FINAL_TOLERANCE);

    if (in_band) {
      if (!final_settle_timing) {
        final_settle_timing = true;
        final_settle_since = millis();
      }
      else if (millis() - final_settle_since >= FINAL_SETTLE_MS) {
        Serial.println("FINAL POSITION REACHED");

        // Stop both motors and finish this shape.
        setMotor(0, 0, PWM_pin, IN1, IN2);
        setMotor(0, 0, PWM_pin_2, IN3, IN4);
        state = ST_IDLE;

        point_index = 0;
        point_index_2 = 0;
        shape = (shape + 1) % NUM_SHAPES;

        trajectory_complete = false;
        final_settle_timing = false;
        final_settle_since = 0;
      }
    }
    else {
      // Both motors must remain inside the tolerance band continuously.
      final_settle_timing = false;
    }
  }

  // ===================== HOMING SETTLED? CHECK =====================
  if (state == ST_HOMING) {
    bool in_band = abs(e) <= HOME_TOLERANCE && abs(e_2) <= HOME_TOLERANCE;
    if (!in_band) {
      home_in_band_since = millis();                    // if left the band restart counter
    } else if (millis() - home_in_band_since > HOME_SETTLE_MS) {
      state = ST_HOMED;
      Serial.println("HOMED - press button to draw!!!");
    }
  }

  // ===================== PRINTING =====================
  interval_count = interval_count + 1;
  if (interval_count >= print_interval) { // if print interval met then print..
    interval_count = 0;

    // PWM contribution of each term (before the max amplitude clamp)
    float p_pwm   = kp * e;
    float i_pwm   = ki * e_sum;
    float d_pwm   = kd * d_term;
    float p_pwm_2 = kp_2 * e_2;
    float i_pwm_2 = ki_2 * e_sum_2;
    float d_pwm_2 = kd_2 * d_term_2;

    printField("t", (int)millis());

    Serial.print("| M1 ");
    printField("ref_1", ref);
    printField("pos_1", pos);
    printField("e_1", e);
    printField("P_1", p_pwm);
    printField("I_1", i_pwm);
    printField("D_1", d_pwm);
    printField("u_1", u_sign * u_amplitude);

    Serial.print("| M2 ");
    printField("ref_2", ref_2);
    printField("pos_2", pos_2);
    printField("e_2", e_2);
    printField("P_2", p_pwm_2);
    printField("I_2", i_pwm_2);
    printField("D_2", d_pwm_2);
    printField("u_2", u_sign_2 * u_amplitude_2);

    Serial.println();
  }
}

// Cap the control amplitude of both motors
void setLimits(int limit) {
  u_max_amplitude   = limit;
  u_max_amplitude_2 = limit;
}

// Clears controller memory
void resetController() {
  t_prev_us = micros();
  e_sum = 0;    e_sum_2 = 0;
  d_term = 0;   d_term_2 = 0;

  pos_prev = pos;
  pos_prev_2 = pos_2;
}

// Send PWM signal to motor driver (shared by both motors)
void setMotor(int dir, float pwmVal, int pwm_pin, int in1, int in2) {
  analogWrite(pwm_pin, pwmVal);  // 0 - 255
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

// Debounced BTN input
bool buttonPressed() {
  static bool stable = HIGH;              // last accepted state
  static bool last_reading = HIGH;
  static unsigned long last_change = 0;

  bool reading = digitalRead(BUTTON_PIN);
  if (reading != last_reading) {          // contact moved: restart the timer
    last_change = millis();
    last_reading = reading;
  }
  if ((millis() - last_change) > debounce_ms && reading != stable) {
    stable = reading;
    if (stable == LOW) return true;       // accepted press (pull-up: LOW = pressed)
  }
  return false;
}

// Pen setup..
void penSetup() {
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);           // no pulses = servo idle and silent
}

// Pen loop: reads only pen_down_cmd. Called once per main loop pass.
void penLoop() {
  if (pen_moving) {
    // let the timed move finish, then stop sending pulses
    if ((long)(millis() - pen_move_end) >= 0) {
      pen.detach();
      pinMode(SERVO_PIN, OUTPUT);
      digitalWrite(SERVO_PIN, LOW);
      pen_moving = false;
    }
    return;
  }

  // If not moving: start a move if the pen isn't where it should be.
  bool want_down = pen_down_cmd;
  if (want_down != pen_is_down) {
    pen.attach(SERVO_PIN, 700, 2300);     // limits stop the library clamping the pulse
    pen.writeMicroseconds(want_down ? PEN_DOWN_US : PEN_UP_US);
    pen_move_end = millis() + (want_down ? PEN_DOWN_MS : PEN_UP_MS);
    pen_is_down = want_down;              // taken as its position once the move completes
    pen_moving = true;
  }
}

// Interrupt service routine, motor 1
void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}

// Interrupt service routine, motor 2
void readEncoder2() {
  int b = digitalRead(ENCB_2);
  if (b > 0) {
    posi_2++;
  } else {
    posi_2--;
  }
}

// Print "label=value " (int and float versions)
void printField(const char* label, int value) {
  Serial.print(label);
  Serial.print("=");
  Serial.print(value);
  Serial.print(" ");
}

void printField(const char* label, float value) {
  Serial.print(label);
  Serial.print("=");
  Serial.print(value);
  Serial.print(" ");
}

// Prints the rig state and pen state once a second, independent of the control loop
void printState() {
  static unsigned long last_print = 0;
  if (millis() - last_print < 1000) return;
  last_print = millis();

  Serial.print("state=");
  switch (state) {
    case ST_IDLE:    Serial.print("IDLE");    break;
    case ST_HOMING:  Serial.print("HOMING");  break;
    case ST_HOMED:   Serial.print("HOMED");   break;
    case ST_RUNNING: Serial.print("RUNNING"); break;
    case ST_PAUSED:  Serial.print("PAUSED");  break;
  }

  Serial.print(" shape=");
  Serial.print(shape + 1);

  Serial.print(" pen=");
  if (pen_moving) {
    Serial.println(pen_is_down ? "LOWERING" : "RAISING");
  } else {
    Serial.println(pen_is_down ? "DOWN" : "UP");
  }
}