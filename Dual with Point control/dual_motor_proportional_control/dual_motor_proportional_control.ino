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
int u_max_amplitude = 150; // control signal amplitude (must be < 256)
int u_sign = 0;            // required sign (Drive Direction) of control signal
long time_start = 0;
const int shape1_m1[] = {460, 456, 452, 449, 447, 446, 444, 443, 441, 440, 438, 437, 436, 434, 433, 431, 430, 428, 427, 425, 424, 422, 421, 419, 418, 416, 415, 413, 412, 410, 409, 407, 406, 404, 403, 401, 400, 398, 397, 395, 394, 392, 391, 389, 388, 386, 385, 383, 382, 380, 379, 377, 375, 374, 372, 371, 369, 368, 366, 365, 363, 361, 360, 358, 357, 355, 353, 352, 350, 349, 347, 345, 344, 342, 341, 339, 337, 336, 334, 332, 331, 329, 327, 326, 324, 322, 320, 319, 317, 315, 313, 312, 310, 308, 306, 305, 303, 301, 299, 297, 295, 294, 293, 293, 295, 296, 297, 298, 300, 301, 302, 303, 304, 306, 307, 308, 309, 310, 311, 312, 313, 314, 315, 316, 317, 318, 319, 320, 321, 321, 322, 323, 324, 325, 325, 326, 327, 327, 328, 329, 329, 330, 331, 331, 332, 332, 333, 333, 333, 334, 334, 335, 335, 335, 336, 336, 336, 336, 336, 337, 337, 337, 337, 337, 337, 337, 337, 337, 337, 336, 336, 336, 336, 335, 335, 335, 334, 334, 333, 333, 332, 331, 331, 330, 329, 328, 327, 326, 325, 323, 322, 321, 319, 318, 316, 314, 312, 309, 307, 304, 300, 298, 299, 302, 307, 313, 318, 322, 327, 331, 335, 339, 343, 347, 350, 354, 357, 361, 364, 367, 370, 373, 377, 380, 383, 386, 389, 392, 394, 397, 400, 403, 406, 408, 411, 414, 417, 419, 422, 425, 427, 430, 432, 435, 438, 440, 443, 445, 448, 450, 453, 455, 458, 460, 462, 465, 467, 470, 472, 475, 477, 479, 482, 484, 486, 489, 491, 494, 496, 498, 501, 503, 505, 508, 510, 512, 514, 517, 519, 521, 524, 526, 528, 531, 533, 535, 537, 540, 542, 544, 547, 549, 551, 553, 556, 558, 560, 562, 565, 567, 569, 572, 574, 575, 576, 576, 576, 577, 577, 577, 578, 578, 578, 578, 578, 579, 579, 579, 579, 579, 579, 579, 579, 579, 579, 579, 578, 578, 578, 578, 578, 577, 577, 577, 576, 576, 575, 575, 574, 574, 573, 573, 572, 571, 571, 570, 569, 568, 568, 567, 566, 565, 564, 563, 562, 561, 560, 558, 557, 556, 555, 553, 552, 550, 549, 547, 546, 544, 543, 541, 539, 537, 536, 534, 532, 530, 528, 526, 523, 521, 519, 517, 514, 512, 509, 507, 504, 501, 499, 496, 493, 490, 487, 484, 481, 477, 474, 471, 467, 464
}; // shape 1 points for motor 1
const int shape2_m1[] = {490, 489, 486, 483, 481, 478, 476, 475, 473, 472, 470, 469, 467, 466, 464, 463, 462, 460, 459, 457, 456, 455, 453, 452, 450, 449, 448, 446, 445, 444, 442, 441, 440, 438, 437, 435, 434, 433, 431, 430, 429, 427, 426, 425, 423, 422, 421, 419, 418, 417, 415, 414, 413, 411, 410, 409, 407, 406, 405, 403, 402, 401, 399, 398, 397, 395, 394, 393, 391, 390, 389, 387, 386, 385, 383, 382, 381, 379, 378, 377, 375, 374, 373, 371, 370, 369, 367, 366, 365, 363, 362, 360, 359, 358, 356, 355, 354, 352, 351, 349, 348, 347, 345, 344, 342, 341, 340, 338, 337, 335, 334, 332, 331, 330, 328, 327, 325, 324, 322, 321, 319, 318, 316, 315, 313, 312, 310, 309, 307, 306, 304, 303, 301, 300, 298, 297, 296, 296, 297, 298, 300, 301, 303, 304, 306, 307, 309, 310, 312, 314, 315, 317, 318, 320, 321, 323, 324, 326, 327, 329, 330, 331, 333, 334, 336, 337, 339, 340, 342, 343, 345, 346, 347, 349, 350, 352, 353, 354, 356, 357, 359, 360, 361, 363, 364, 365, 367, 368, 369, 371, 372, 373, 375, 376, 377, 379, 380, 381, 382, 384, 385, 386, 387, 389, 390, 391, 392, 393, 395, 396, 397, 398, 399, 400, 402, 403, 404, 405, 406, 407, 408, 409, 411, 412, 413, 414, 415, 416, 417, 418, 419, 420, 421, 422, 423, 424, 425, 426, 427, 428, 429, 429, 430, 431, 432, 433, 434, 435, 436, 436, 437, 438, 439, 440, 440, 441, 442, 443, 443, 444, 445, 446, 446, 447, 448, 448, 449, 450, 451, 452, 454, 456, 458, 459, 461, 462, 463, 464, 466, 467, 468, 469, 470, 471, 472, 473, 474, 475, 476, 477, 478, 479, 480, 481, 482, 483, 484, 485, 486, 487, 487, 488, 489, 490, 491, 491, 492, 493, 494, 494, 495, 496, 497, 497, 498, 498, 499, 500, 500, 501, 501, 502, 503, 503, 504, 504, 505, 505, 506, 506, 506, 507, 507, 508, 508, 508, 509, 509, 509, 510, 510, 510, 511, 511, 511, 511, 511, 512, 512, 512, 512, 512, 512, 512, 512, 513, 513, 513, 513, 513, 512, 512, 512, 512, 512, 512, 512, 512, 511, 511, 511, 511, 510, 510, 510, 509, 509, 508, 508, 508, 507, 507, 506, 505, 505, 504, 503, 503, 502, 501, 500, 500, 499, 498, 497, 496, 495, 494, 493, 491
}; // shape 2 points for motor 1
const int shape3_m1[] = {330, 331, 331, 331, 332, 332, 332, 333, 333, 334, 334, 334, 335, 335, 336, 336, 337, 337, 338, 339, 339, 340, 341, 341, 342, 343, 343, 344, 345, 346, 347, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 360, 361, 362, 363, 364, 366, 367, 368, 370, 371, 372, 374, 375, 377, 378, 380, 381, 383, 384, 386, 387, 389, 391, 392, 394, 396, 397, 399, 401, 403, 404, 406, 408, 410, 412, 414, 415, 417, 419, 421, 423, 425, 427, 429, 431, 433, 435, 437, 439, 441, 443, 445, 447, 449, 451, 453, 455, 457, 459, 461, 463, 465, 467, 469, 471, 473, 475, 477, 479, 481, 483, 485, 487, 489, 491, 493, 495, 497, 499, 501, 503, 504, 506, 508, 510, 512, 514, 516, 517, 519, 521, 523, 524, 526, 528, 529, 531, 533, 534, 536, 537, 539, 541, 542, 544, 545, 546, 548, 549, 550, 552, 553, 554, 555, 557, 558, 559, 560, 561, 562, 563, 564, 565, 566, 567, 567, 568, 569, 569, 570, 571, 571, 572, 572, 572, 573, 573, 573, 574, 574, 574, 574, 574, 574, 574, 574, 573, 573, 573, 573, 572, 572, 571, 571, 570, 569, 568, 568, 567, 566, 565, 564, 563, 562, 560, 559, 558, 556, 555, 553, 552, 550, 549, 547, 545, 543, 541, 539, 538, 536, 533, 531, 529, 527, 525, 523, 520, 518, 516, 513, 511, 508, 506, 503, 501, 498, 496, 493, 491, 488, 485, 483, 480, 478, 475, 473, 470, 467, 465, 462, 460, 457, 455, 452, 450, 447, 445, 442, 440, 438, 435, 433, 431, 428, 426, 424, 422, 420, 418, 416, 414, 412, 410, 408, 406, 404, 402, 400, 398, 397, 395, 393, 392, 390, 388, 387, 385, 384, 382, 381, 379, 378, 377, 375, 374, 373, 371, 370, 369, 368, 367, 366, 364, 363, 362, 361, 360, 359, 358, 357, 356, 355, 355, 354, 353, 352, 351, 350, 350, 349, 348, 347, 347, 346, 345, 345, 344, 343, 343, 342, 341, 341, 340, 340, 339, 339, 338, 338, 337, 337, 336, 336, 335, 335, 335, 334, 334, 333, 333, 333, 332, 332, 332, 332, 331, 331, 331, 330, 330, 330, 330, 330, 329, 329, 329, 329, 329, 329, 329, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 328, 329, 329, 329, 329, 329, 329, 330, 330, 330, 330
}; // shape 3 points for motor 1
int point_index = 0;            // which setpoint motor is currently targeting
float time_per_point = 12.5;    // ms to spend on each encoder point
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
int u_max_amplitude_2 = 150;
int u_sign_2 = 0;
long time_start_2 = 0;
const int shape1_m2[] = {843, 846, 846, 845, 842, 839, 836, 833, 830, 827, 824, 821, 818, 815, 812, 808, 805, 802, 799, 796, 793, 790, 787, 784, 780, 777, 774, 771, 768, 765, 761, 758, 755, 752, 748, 745, 742, 739, 735, 732, 729, 726, 722, 719, 716, 712, 709, 706, 702, 699, 695, 692, 689, 685, 682, 678, 675, 671, 668, 664, 661, 657, 654, 650, 647, 643, 640, 636, 632, 629, 625, 621, 618, 614, 610, 607, 603, 599, 595, 592, 588, 584, 580, 576, 572, 568, 565, 561, 557, 553, 549, 545, 541, 536, 532, 528, 524, 520, 516, 511, 507, 503, 498, 495, 493, 491, 489, 488, 486, 484, 482, 480, 478, 477, 475, 473, 471, 469, 466, 464, 462, 460, 458, 456, 454, 451, 449, 447, 444, 442, 439, 437, 435, 432, 429, 427, 424, 422, 419, 416, 413, 411, 408, 405, 402, 399, 396, 393, 390, 387, 384, 381, 377, 374, 371, 367, 364, 361, 357, 353, 350, 346, 342, 339, 335, 331, 327, 323, 319, 315, 311, 306, 302, 297, 293, 288, 284, 279, 274, 269, 264, 258, 253, 248, 242, 236, 230, 224, 218, 211, 205, 198, 190, 183, 175, 167, 158, 149, 139, 128, 116, 107, 104, 110, 121, 132, 143, 153, 162, 171, 179, 187, 194, 202, 208, 215, 222, 228, 234, 240, 246, 252, 257, 262, 268, 273, 278, 283, 288, 292, 297, 302, 306, 310, 315, 319, 323, 327, 331, 335, 339, 343, 347, 351, 354, 358, 362, 365, 369, 372, 376, 379, 382, 386, 389, 392, 395, 398, 401, 404, 407, 410, 413, 416, 419, 422, 424, 427, 430, 433, 435, 438, 440, 443, 445, 448, 450, 453, 455, 457, 460, 462, 464, 466, 469, 471, 473, 475, 477, 479, 481, 483, 485, 487, 489, 491, 493, 495, 496, 498, 500, 503, 506, 511, 515, 520, 524, 528, 532, 536, 540, 544, 548, 552, 556, 560, 564, 568, 572, 576, 579, 583, 587, 591, 595, 598, 602, 606, 609, 613, 617, 620, 624, 628, 631, 635, 638, 642, 645, 649, 653, 656, 660, 663, 666, 670, 673, 677, 680, 684, 687, 690, 694, 697, 700, 704, 707, 710, 714, 717, 720, 723, 727, 730, 733, 736, 740, 743, 746, 749, 752, 756, 759, 762, 765, 768, 771, 774, 777, 781, 784, 787, 790, 793, 796, 799, 802, 805, 808, 811, 814, 817, 820, 823, 826, 829, 832, 835, 838, 840
}; // shape 1 points for motor 2
const int shape2_m2[] = {815, 818, 821, 822, 823, 822, 821, 818, 816, 814, 812, 810, 808, 805, 803, 801, 799, 796, 794, 792, 790, 787, 785, 783, 780, 778, 776, 774, 771, 769, 767, 764, 762, 760, 757, 755, 752, 750, 748, 745, 743, 740, 738, 736, 733, 731, 728, 726, 723, 721, 718, 716, 713, 711, 708, 706, 703, 701, 698, 696, 693, 691, 688, 685, 683, 680, 678, 675, 672, 670, 667, 664, 662, 659, 657, 654, 651, 648, 646, 643, 640, 638, 635, 632, 629, 627, 624, 621, 618, 615, 613, 610, 607, 604, 601, 598, 595, 593, 590, 587, 584, 581, 578, 575, 572, 569, 566, 563, 560, 557, 554, 551, 548, 544, 541, 538, 535, 532, 529, 526, 522, 519, 516, 513, 509, 506, 503, 499, 496, 493, 489, 486, 483, 479, 475, 472, 468, 465, 463, 462, 463, 463, 463, 464, 464, 465, 465, 465, 466, 466, 466, 466, 467, 467, 467, 467, 467, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 468, 467, 467, 467, 467, 467, 466, 466, 466, 465, 465, 465, 464, 464, 464, 463, 463, 462, 462, 462, 461, 461, 460, 460, 459, 458, 458, 457, 457, 456, 455, 455, 454, 453, 453, 452, 451, 450, 449, 449, 448, 447, 446, 445, 444, 443, 443, 442, 441, 440, 439, 438, 437, 436, 435, 433, 432, 431, 430, 429, 428, 427, 425, 424, 423, 422, 420, 419, 418, 416, 415, 414, 412, 411, 409, 408, 406, 405, 403, 402, 400, 399, 397, 395, 394, 392, 391, 389, 387, 385, 384, 382, 381, 382, 384, 388, 392, 396, 401, 405, 409, 413, 418, 422, 426, 430, 434, 438, 442, 446, 450, 454, 458, 461, 465, 469, 473, 477, 480, 484, 488, 491, 495, 499, 502, 506, 509, 513, 517, 520, 524, 527, 531, 534, 537, 541, 544, 548, 551, 554, 558, 561, 564, 568, 571, 574, 578, 581, 584, 587, 591, 594, 597, 600, 603, 607, 610, 613, 616, 619, 622, 626, 629, 632, 635, 638, 641, 644, 647, 650, 653, 656, 660, 663, 666, 669, 672, 675, 678, 681, 684, 687, 690, 693, 696, 699, 702, 705, 708, 711, 714, 717, 720, 723, 726, 729, 732, 735, 738, 741, 744, 746, 749, 752, 755, 758, 761, 764, 767, 770, 773, 776, 779, 782, 785, 788, 791, 794, 797, 800, 803, 806, 809, 812
}; // shape 2 points for motor 2
const int shape3_m2[] = {373, 370, 368, 365, 363, 360, 358, 355, 353, 351, 348, 346, 344, 342, 340, 337, 335, 333, 331, 330, 328, 326, 324, 323, 321, 319, 318, 316, 315, 314, 312, 311, 310, 309, 308, 307, 306, 305, 304, 304, 303, 302, 302, 301, 301, 301, 300, 300, 300, 300, 300, 300, 300, 301, 301, 301, 302, 302, 303, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 315, 316, 318, 319, 321, 322, 324, 326, 328, 329, 331, 333, 335, 337, 339, 341, 344, 346, 348, 350, 353, 355, 358, 360, 363, 365, 368, 370, 373, 376, 378, 381, 384, 387, 389, 392, 395, 398, 401, 404, 407, 410, 413, 416, 419, 422, 425, 428, 431, 435, 438, 441, 444, 447, 451, 454, 457, 460, 464, 467, 470, 474, 477, 480, 484, 487, 490, 494, 497, 501, 504, 507, 511, 514, 518, 521, 525, 528, 531, 535, 538, 542, 545, 549, 552, 555, 559, 562, 566, 569, 573, 576, 579, 583, 586, 590, 593, 596, 600, 603, 606, 610, 613, 616, 620, 623, 626, 630, 633, 636, 639, 643, 646, 649, 652, 655, 658, 661, 665, 668, 671, 674, 677, 680, 683, 686, 688, 691, 694, 697, 700, 702, 705, 708, 711, 713, 716, 718, 721, 723, 726, 728, 730, 733, 735, 737, 739, 741, 743, 745, 747, 749, 751, 753, 755, 756, 758, 760, 761, 762, 764, 765, 766, 768, 769, 770, 771, 772, 772, 773, 774, 775, 775, 776, 776, 776, 777, 777, 777, 777, 777, 777, 777, 776, 776, 776, 775, 775, 774, 773, 773, 772, 771, 770, 769, 768, 766, 765, 764, 763, 761, 760, 758, 756, 755, 753, 751, 749, 748, 746, 744, 742, 739, 737, 735, 733, 731, 728, 726, 723, 721, 718, 716, 713, 711, 708, 705, 703, 700, 697, 694, 692, 689, 686, 683, 680, 677, 674, 671, 668, 665, 662, 659, 655, 652, 649, 646, 643, 640, 636, 633, 630, 627, 623, 620, 617, 613, 610, 607, 603, 600, 597, 593, 590, 586, 583, 580, 576, 573, 569, 566, 563, 559, 556, 552, 549, 545, 542, 539, 535, 532, 528, 525, 521, 518, 515, 511, 508, 504, 501, 498, 494, 491, 487, 484, 481, 477, 474, 471, 467, 464, 461, 457, 454, 451, 448, 444, 441, 438, 435, 432, 429, 425, 422, 419, 416, 413, 410, 407, 404, 401, 398, 395, 392, 390, 387, 384, 381, 378, 376
}; // shape 2 points for motor 2
int point_index_2 = 0;            // which setpoint we're currently targeting
float time_per_point_2 = 12.5;    // ms spent on each point
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
const unsigned long Ts_us = 2500;  // derivative sample period (2 ms)
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

const int U_MAX_RUN  = 150;                 // Amplitude limit while drawing shape
const int U_MAX_HOME = 80;                  // Amplitude limit while homing
const int HOME_TOLERANCE = 5;               // Tolerance in encoder counts for state > ST_HOMED
const unsigned long HOME_SETTLE_MS = 7000;  // Settling time for how long motors must be within HOME_TOLERANCE
unsigned long home_in_band_since = 0;
const unsigned long debounce_ms = 30;       // Button press debounce prevents multiple inputs from one button press

// ---------- Derivative Adapative Gain -----------
const float KD_STRAIGHT = 0.3, KD_CORNER = 1.5, KD_CIRCLE = 1.0;   // starting guesses
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
      (millis() - time_start) > time_per_point) {
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
  float ref_vel = (shapes_m1[shape][nxt] - shapes_m1[shape][point_index]) / (time_per_point / 1000.0f); // counts/s
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
      (millis() - time_start_2) > time_per_point_2) {
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
  float ref_vel_2 = (shapes_m2[shape][nxt_2] - shapes_m2[shape][point_index_2]) / (time_per_point_2 / 1000.0f); // counts/s
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