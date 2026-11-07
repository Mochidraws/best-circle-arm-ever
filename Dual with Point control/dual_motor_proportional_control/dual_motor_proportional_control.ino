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
float kp = 3;            // proportional gain
int pos = 0;               // output encoder position from the gearbox
int e = 0;                 // error between ref and pos
int u_amplitude = 0;       // amplitude of the PID calculated control signal
float u_f = 0;             // float version of u_amplitude
int u_max_amplitude = 150; // control signal amplitude (must be < 256)
int u_sign = 0;            // required sign (Drive Direction) of control signal
long time_start = 0;
const int shape1_m1[] = {468,467,467,466,466,465,465,464,464,463,463,462,462,461,460,460,459,459,458,458,457,457,456,456,455,455,454,454,453,453,452,452,451,451,450,450,449,449,448,448,447,446,446,445,445,444,444,443,443,442,442,441,441,440,440,439,439,438,438,437,437,436,436,435,434,434,433,433,432,432,431,431,430,430,429,429,428,428,427,427,426,425,425,424,424,423,423,422,422,421,421,420,420,419,419,418,417,417,416,416,415,415,414,414,413,413,412,412,411,411,410,409,409,408,408,407,407,406,406,405,405,404,403,403,402,402,401,401,400,400,399,399,398,397,397,396,396,395,395,394,394,393,392,392,391,391,390,390,389,389,388,387,387,386,386,385,385,384,384,383,382,382,381,381,380,380,379,378,378,377,377,376,376,375,375,374,373,373,372,372,371,371,370,369,369,368,368,367,366,366,365,365,364,364,363,362,362,361,361,360,360,359,358,358,357,357,356,355,355,354,354,353,352,352,351,351,350,349,349,348,348,347,346,346,345,345,344,343,343,342,342,341,340,340,339,339,338,337,337,336,336,335,334,334,333,332,332,331,331,330,329,330,331,332,332,333,334,334,335,336,337,337,338,339,339,340,341,341,342,343,344,344,345,346,346,347,348,348,349,350,350,351,351,352,353,353,354,355,355,356,357,357,358,358,359,360,360,361,362,362,363,363,364,364,365,366,366,367,367,368,369,369,370,370,371,371,372,372,373,374,374,375,375,376,376,377,377,378,378,379,379,380,380,381,381,382,382,383,383,384,384,385,385,386,386,387,387,388,388,388,389,389,390,390,391,391,392,392,392,393,393,394,394,395,395,395,396,396,397,397,397,398,398,398,399,399,400,400,400,401,401,401,402,402,402,403,403,403,404,404,404,405,405,405,406,406,406,407,407,407,407,408,408,408,409,409,409,409,410,410,410,410,411,411,411,411,412,412,412,412,413,413,413,413,413,414,414,414,414,414,415,415,415,415,415,415,416,416,416,416,416,416,417,417,417,417,417,417,417,418,418,418,418,418,418,418,418,418,419,419,419,419,419,419,419,419,419,419,419,419,419,419,419,419,420,420,420,420,420,420,420,420,420,420,420,420,420,420,420,420,420,420,419,419,419,419,419,419,419,419,419,420,421,422,423,424,425,426,427,428,429,430,431,432,433,434,435,436,437,438,439,440,441,442,443,443,444,445,446,447,448,449,450,451,452,453,454,455,456,457,458,459,460,461,462,463,463,464,465,466,467,468,469,470,471,472,473,474,475,476,477,478,478,479,480,481,482,483,484,485,486,487,488,489,490,491,492,492,493,494,495,496,497,498,499,500,501,502,503,504,504,505,506,507,508,509,510,511,512,513,514,515,516,517,517,518,519,520,521,522,523,524,525,526,527,528,529,529,530,531,532,533,534,535,536,537,538,539,540,541,541,542,543,544,545,546,547,548,549,550,551,552,553,553,554,555,556,557,558,559,560,561,562,563,564,565,566,566,567,568,569,570,571,572,573,574,575,576,577,578,579,580,580,581,582,583,584,585,586,587,588,589,590,591,592,593,594,595,595,596,597,598,599,600,601,602,603,604,605,606,607,608,609,610,611,611,612,613,614,615,616,617,618,619,620,621,622,623,624,625,626,627,628,628,629,630,631,632,633,634,635,636,637,638,639,640,641,642,643,644,645,646,647,648,648,649,650,651,652,653,654,654,654,655,655,655,655,655,655,655,655,655,655,655,655,655,655,655,656,656,656,656,656,656,656,656,656,656,656,656,656,656,656,656,656,656,656,656,655,655,655,655,655,655,655,655,655,655,655,655,655,655,654,654,654,654,654,654,654,654,653,653,653,653,653,653,653,652,652,652,652,652,651,651,651,651,651,650,650,650,650,649,649,649,649,648,648,648,648,647,647,647,646,646,646,645,645,645,644,644,644,643,643,642,642,642,641,641,640,640,640,639,639,638,638,637,637,636,636,635,635,634,634,633,633,632,632,631,631,630,630,629,628,628,627,627,626,625,625,624,623,623,622,621,621,620,619,618,618,617,616,615,615,614,613,612,611,611,610,609,608,607,606,605,604,603,603,602,601,600,599,598,597,596,595,594,593,591,590,589,588,587,586,585,584,582,581,580,579,578,576,575,574,572,571,570,569,567,566,564,563,562,560,559,557,556,554,553,551,550,548,546,545,543,542,540,538,536,535,533,531,529,528,526,524,522,520,518,516,514,512,510,508,506,504,502,500,498,496,493,491,489,487,484,482,480,477,475,473,470
}; // shape 1 points for motor 1
const int shape2_m1[] = {836,834,832,830,829,827,825,823,822,820,818,816,815,813,811,810,808,806,805,803,801,800,798,796,795,793,791,790,788,787,785,783,782,780,779,777,776,774,772,771,769,768,766,765,763,762,760,759,757,756,754,753,751,750,749,747,746,744,743,741,740,739,737,736,734,733,732,730,729,727,726,725,723,722,721,719,718,717,715,714,713,711,710,709,707,706,705,704,702,701,700,698,697,696,695,693,692,691,690,688,687,686,685,684,682,681,680,679,678,676,675,674,673,672,670,669,668,667,666,665,663,662,661,660,659,658,657,655,654,653,652,651,650,649,648,646,645,644,643,642,641,640,639,638,637,636,634,633,632,631,630,629,628,627,626,625,624,623,622,621,620,619,618,617,616,615,614,613,612,611,610,609,608,607,606,605,604,603,602,601,600,599,598,597,596,595,594,593,592,591,590,589,588,587,586,585,584,583,582,581,580,579,578,577,577,576,575,574,573,572,571,570,569,568,567,566,565,565,564,563,562,561,560,559,558,557,556,556,555,554,553,552,551,550,549,548,548,547,546,545,544,543,542,541,541,540,539,538,537,536,535,535,534,533,532,531,530,529,529,528,527,526,525,524,523,523,522,521,520,519,518,518,517,516,515,514,513,513,512,511,510,509,508,508,507,506,505,504,503,503,502,501,500,499,499,498,497,496,495,495,494,493,492,491,491,490,489,488,487,487,486,485,484,483,483,482,481,480,479,479,478,477,476,475,475,474,473,472,471,471,470,469,468,467,467,467,468,469,470,470,471,472,473,473,474,475,476,476,477,478,479,479,480,481,482,482,483,484,485,485,486,487,488,488,489,490,491,491,492,493,493,494,495,496,496,497,498,499,499,500,501,501,502,503,504,504,505,506,506,507,508,509,509,510,511,511,512,513,514,514,515,516,516,517,518,518,519,520,521,521,522,523,523,524,525,525,526,527,527,528,529,529,530,531,531,532,533,534,534,535,536,536,537,538,538,539,540,540,541,542,542,543,543,544,545,545,546,547,547,548,549,549,550,551,551,552,553,553,554,554,555,556,556,557,558,558,559,559,560,561,561,562,563,563,564,564,565,566,566,567,567,568,569,569,570,570,571,572,572,573,573,574,575,575,576,576,577,578,578,579,579,580,580,581,582,582,583,583,584,584,585,586,586,587,587,588,588,589,589,590,591,591,592,592,593,593,594,594,595,595,596,596,597,597,598,599,599,600,600,601,601,602,602,603,603,604,604,605,605,606,606,607,607,608,608,609,609,610,610,611,611,612,612,613,613,613,614,614,615,615,616,616,617,617,618,618,619,619,620,620,620,621,621,622,622,623,623,623,624,624,625,625,626,626,627,627,627,628,628,629,629,629,630,630,631,631,631,632,632,633,633,633,634,634,635,635,635,636,636,637,637,637,638,638,638,639,639,640,640,640,641,641,641,642,642,642,643,643,643,644,644,644,645,645,646,646,646,647,647,647,647,648,648,648,649,649,649,650,650,650,651,651,651,652,652,652,652,653,653,654,654,655,656,656,657,657,658,658,659,659,660,661,661,662,662,663,663,664,664,665,665,666,667,667,668,668,669,669,670,670,671,671,672,673,673,674,674,675,675,676,676,677,677,678,678,679,680,680,681,681,682,682,683,683,684,684,685,685,686,687,687,688,688,689,689,690,690,691,691,692,692,693,693,694,694,695,696,696,697,697,698,698,699,699,700,700,701,701,702,702,703,703,704,705,705,706,706,707,707,708,708,709,709,710,710,711,711,712,712,713,713,714,715,715,716,716,717,717,718,718,719,719,720,720,721,721,722,722,723,723,724,724,725,725,726,727,727,728,728,729,729,730,730,731,731,732,732,733,733,734,734,735,735,736,736,737,738,738,739,739,740,740,741,741,742,742,743,743,744,744,745,745,746,746,747,747,748,749,749,750,750,751,751,752,752,753,753,754,754,755,755,756,756,757,758,758,759,759,760,760,761,761,762,762,763,763,764,764,765,765,766,767,767,768,768,769,769,770,770,771,771,772,772,773,774,774,775,775,776,776,777,777,778,778,779,780,780,781,781,782,782,783,783,784,784,785,786,786,787,787,788,788,789,789,790,791,791,792,792,793,793,794,795,795,796,796,797,797,798,798,799,800,800,801,801,802,803,803,804,804,805,805,806,807,807,808,808,809,810,810,811,811,812,812,813,814,814,815,815,816,817,817,818,819,819,820,820,821,822,822,823,823,824,825,825,826,827,827,828,828,829,830,830,831,832,832,833,834,834,835
}; // shape 2 points for motor 1
int point_index = 0;            // which setpoint motor is currently targeting
float time_per_point = 5;    // ms to spend on each encoder point
float e_sum = 0; // Integral sum
float ki = 3; // Integral control gain
const float i_max = 30; // Integral control signal limiter. Prevents extreme buildup and overshoot.
float kd = 0.12;     // Derivative control gain
int e_prev = 0;      // Arithmetic for Derivative error
float d_term = 0;    // filtered de/dt (counts/s)

// ---------- Motor 2 variables (independent of motor 1) ----------
volatile int posi_2 = 0;
float ref_2 = 0;
float kp_2 = 3;
int pos_2 = 0;
int e_2 = 0;
int u_amplitude_2 = 0;
float u_f_2 = 0;
int u_max_amplitude_2 = 150;
int u_sign_2 = 0;
long time_start_2 = 0;
const int shape1_m2[] = {924,923,922,921,919,918,917,916,915,914,913,911,910,909,908,907,906,905,904,902,901,900,899,898,897,896,894,893,892,891,890,889,888,886,885,884,883,882,881,880,878,877,876,875,874,873,872,870,869,868,867,866,865,863,862,861,860,859,858,857,855,854,853,852,851,850,848,847,846,845,844,843,841,840,839,838,837,836,834,833,832,831,830,829,827,826,825,824,823,822,820,819,818,817,816,814,813,812,811,810,809,807,806,805,804,803,801,800,799,798,797,795,794,793,792,791,789,788,787,786,785,783,782,781,780,779,777,776,775,774,773,771,770,769,768,766,765,764,763,762,760,759,758,757,755,754,753,752,751,749,748,747,746,744,743,742,741,739,738,737,736,734,733,732,731,729,728,727,726,724,723,722,721,719,718,717,716,714,713,712,711,709,708,707,706,704,703,702,700,699,698,697,695,694,693,691,690,689,688,686,685,684,682,681,680,678,677,676,675,673,672,671,669,668,667,665,664,663,661,660,659,657,656,655,653,652,651,649,648,647,645,644,643,641,640,638,637,636,634,633,632,630,629,628,626,625,623,622,621,619,618,617,617,617,616,616,615,615,614,614,613,613,613,612,612,611,611,610,610,609,609,608,608,607,607,606,606,605,604,604,603,603,602,602,601,601,600,600,599,598,598,597,597,596,596,595,594,594,593,593,592,591,591,590,590,589,588,588,587,586,586,585,584,584,583,582,582,581,580,580,579,578,578,577,576,576,575,574,574,573,572,571,571,570,569,569,568,567,566,566,565,564,563,563,562,561,560,560,559,558,557,556,556,555,554,553,552,552,551,550,549,548,548,547,546,545,544,543,543,542,541,540,539,538,537,536,536,535,534,533,532,531,530,529,528,528,527,526,525,524,523,522,521,520,519,518,517,516,515,514,514,513,512,511,510,509,508,507,506,505,504,503,502,501,500,499,498,497,496,494,493,492,491,490,489,488,487,486,485,484,483,482,481,480,478,477,476,475,474,473,472,471,469,468,467,466,465,464,463,461,460,459,458,457,456,454,453,452,451,450,448,447,446,445,443,442,441,440,438,437,436,435,433,432,431,429,428,427,426,424,423,422,420,419,418,416,415,414,412,411,409,408,407,405,404,403,401,400,398,397,395,397,398,400,401,403,404,405,407,408,409,411,412,414,415,416,418,419,420,422,423,424,426,427,428,429,431,432,433,435,436,437,438,440,441,442,443,445,446,447,448,450,451,452,453,454,456,457,458,459,460,461,463,464,465,466,467,468,469,471,472,473,474,475,476,477,478,480,481,482,483,484,485,486,487,488,489,490,491,492,493,494,496,497,498,499,500,501,502,503,504,505,506,507,508,509,510,511,512,513,514,514,515,516,517,518,519,520,521,522,523,524,525,526,527,528,528,529,530,531,532,533,534,535,536,536,537,538,539,540,541,542,543,543,544,545,546,547,548,548,549,550,551,552,552,553,554,555,556,556,557,558,559,560,560,561,562,563,563,564,565,566,566,567,568,569,569,570,571,571,572,573,574,574,575,576,576,577,578,578,579,580,580,581,582,582,583,584,584,585,586,586,587,588,588,589,590,590,591,591,592,593,593,594,594,595,596,596,597,597,598,598,599,600,600,601,601,602,602,603,603,604,604,605,606,606,607,607,608,608,609,609,610,610,611,611,612,612,613,613,613,614,614,615,615,616,616,617,617,617,618,619,621,622,623,625,626,628,629,630,632,633,634,636,637,638,640,641,643,644,645,647,648,649,651,652,653,655,656,657,659,660,661,663,664,665,667,668,669,671,672,673,675,676,677,678,680,681,682,684,685,686,688,689,690,691,693,694,695,697,698,699,700,702,703,704,706,707,708,709,711,712,713,714,716,717,718,719,721,722,723,724,726,727,728,729,731,732,733,734,736,737,738,739,741,742,743,744,746,747,748,749,751,752,753,754,755,757,758,759,760,762,763,764,765,766,768,769,770,771,773,774,775,776,777,779,780,781,782,783,785,786,787,788,789,791,792,793,794,795,797,798,799,800,801,803,804,805,806,807,809,810,811,812,813,814,816,817,818,819,820,822,823,824,825,826,827,829,830,831,832,833,834,836,837,838,839,840,841,843,844,845,846,847,848,850,851,852,853,854,855,857,858,859,860,861,862,863,865,866,867,868,869,870,872,873,874,875,876,877,878,880,881,882,883,884,885,886,888,889,890,891,892,893,894,896,897,898,899,900,901,902,904,905,906,907,908,909,910,911,913,914,915,916,917,918,919,921,922,923
}; // shape 1 points for motor 2
const int shape2_m2[] = {886,886,885,884,884,883,883,882,881,881,880,879,879,878,878,877,876,876,875,874,874,873,873,872,871,871,870,869,869,868,867,867,866,865,865,864,864,863,862,862,861,860,860,859,858,858,857,856,856,855,854,854,853,852,852,851,850,849,849,848,847,847,846,845,845,844,843,843,842,841,841,840,839,838,838,837,836,836,835,834,834,833,832,831,831,830,829,829,828,827,827,826,825,824,824,823,822,822,821,820,819,819,818,817,816,816,815,814,814,813,812,811,811,810,809,808,808,807,806,806,805,804,803,803,802,801,800,800,799,798,797,797,796,795,794,794,793,792,791,791,790,789,788,788,787,786,785,785,784,783,782,782,781,780,779,779,778,777,776,776,775,774,773,772,772,771,770,769,769,768,767,766,765,765,764,763,762,762,761,760,759,758,758,757,756,755,755,754,753,752,751,751,750,749,748,747,747,746,745,744,743,743,742,741,740,740,739,738,737,736,735,735,734,733,732,731,731,730,729,728,727,727,726,725,724,723,723,722,721,720,719,718,718,717,716,715,714,714,713,712,711,710,709,709,708,707,706,705,704,704,703,702,701,700,699,699,698,697,696,695,694,694,693,692,691,690,689,688,688,687,686,685,684,683,683,682,681,680,679,678,677,677,676,675,674,673,672,671,670,670,669,668,667,666,665,664,664,663,662,661,660,659,658,657,657,656,655,654,653,652,651,650,649,649,648,647,646,645,644,643,642,641,641,640,639,638,637,636,635,634,633,632,632,631,630,630,630,630,631,631,631,631,631,631,632,632,632,632,632,632,633,633,633,633,633,633,634,634,634,634,634,634,635,635,635,635,635,635,635,636,636,636,636,636,636,636,636,637,637,637,637,637,637,637,637,637,638,638,638,638,638,638,638,638,638,638,639,639,639,639,639,639,639,639,639,639,639,639,639,639,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,639,639,639,639,639,639,639,639,639,639,639,639,639,639,638,638,638,638,638,638,638,638,638,638,637,637,637,637,637,637,637,637,637,636,636,636,636,636,636,636,636,635,635,635,635,635,635,635,634,634,634,634,634,634,633,633,633,633,633,633,633,632,632,632,632,632,631,631,631,631,631,631,630,630,630,630,630,629,629,629,629,629,628,628,628,628,627,627,627,627,627,626,626,626,626,625,625,625,625,624,624,624,624,623,623,623,623,622,622,622,622,621,621,621,621,620,620,620,620,619,619,619,618,618,618,618,617,617,617,616,616,616,616,615,615,615,614,614,614,613,613,613,612,612,612,611,611,611,611,610,610,610,609,609,609,608,608,608,607,607,606,606,606,605,605,605,604,604,604,603,603,603,602,602,601,601,601,600,600,600,599,599,598,598,598,597,597,596,596,596,595,595,594,594,595,596,597,598,599,600,601,602,603,604,605,606,607,608,609,610,611,612,613,614,615,616,617,618,618,619,620,621,622,623,624,625,626,627,628,629,630,631,632,633,634,635,636,637,638,639,639,640,641,642,643,644,645,646,647,648,649,650,651,652,653,653,654,655,656,657,658,659,660,661,662,663,664,665,666,666,667,668,669,670,671,672,673,674,675,676,676,677,678,679,680,681,682,683,684,685,686,686,687,688,689,690,691,692,693,694,695,695,696,697,698,699,700,701,702,703,703,704,705,706,707,708,709,710,711,711,712,713,714,715,716,717,718,719,719,720,721,722,723,724,725,726,726,727,728,729,730,731,732,733,733,734,735,736,737,738,739,739,740,741,742,743,744,745,746,746,747,748,749,750,751,752,752,753,754,755,756,757,758,758,759,760,761,762,763,764,764,765,766,767,768,769,770,770,771,772,773,774,775,775,776,777,778,779,780,781,781,782,783,784,785,786,786,787,788,789,790,791,792,792,793,794,795,796,797,797,798,799,800,801,802,802,803,804,805,806,807,807,808,809,810,811,812,812,813,814,815,816,817,818,818,819,820,821,822,822,823,824,825,826,827,827,828,829,830,831,832,832,833,834,835,836,837,837,838,839,840,841,842,842,843,844,845,846,847,847,848,849,850,851,852,852,853,854,855,856,856,857,858,859,860,861,861,862,863,864,865,866,866,867,868,869,870,871,871,872,873,874,875,875,876,877,878,879,880,880,881,882,883,884,885,885
}; // shape 2 points for motor 2
int point_index_2 = 0;            // which setpoint we're currently targeting
float time_per_point_2 = 5;    // ms spent on each point
float e_sum_2 = 0; //Integral sum
float ki_2 = 3; //Integral control gain
const float i_max_2 = 30;
float kd_2 = 0.12;
int e_prev_2 = 0;
float d_term_2 = 0;

// ---------- Shared ----------
int print_interval = 1000;  // how often values are sent to the serial monitor
int interval_count = 0; // arithmetic for print interval
const unsigned long Ts_us = 2000;  // derivative sample period (2 ms)
const float d_alpha = 0.2;         // derivative filter: smaller = smoother but more lag
unsigned long t_prev_us = 0;

// ---------- Shape selection ----------
const int* const shapes_m1[] = { shape1_m1, shape2_m1 };
const int* const shapes_m2[] = { shape1_m2, shape2_m2 };
const int shape_len[] = { sizeof(shape1_m1) / sizeof(int),
                          sizeof(shape2_m1) / sizeof(int) };
const int NUM_SHAPES = 2; // Change depending on number of different shape arrays to draw
int shape = 0;  // Target shape of run
// compile-time check that each shape's two motor arrays are the same length
static_assert(sizeof(shape1_m1) == sizeof(shape1_m2), "shape 1: M1/M2 lengths differ");
static_assert(sizeof(shape2_m1) == sizeof(shape2_m2), "shape 2: M1/M2 lengths differ");


// ---------- Homing / state machine ----------
enum RigState { ST_IDLE, ST_HOMING, ST_HOMED, ST_RUNNING, ST_PAUSED }; // available states
RigState state = ST_IDLE;                   // starts idle (motors off)

const int U_MAX_RUN  = 150;                 // Amplitude limit while drawing shape
const int U_MAX_HOME = 50;                  // Amplitude limit while homing
const int HOME_TOLERANCE = 5;               // Tolerance in encoder counts for state > ST_HOMED
const unsigned long HOME_SETTLE_MS = 5000;  // Settling time for how long motors must be within HOME_TOLERANCE
unsigned long home_in_band_since = 0;
const unsigned long debounce_ms = 30;       // Button press debounce prevents multiple inputs from one button press

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
  if (state == ST_HOMED)     pen_down_cmd = true;    // on homed: lower the pen
  else if (state == ST_IDLE) pen_down_cmd = false;   // on idle: lift the pen
  penLoop(); // Run pen control with state flags
  printState(); // Print state of main and pen loop every 1s

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
        state = ST_RUNNING;
        Serial.println("RUNNING");
        break;

      case ST_RUNNING:                    // BTN Allows pause during running
        state = ST_PAUSED;
        Serial.println("PAUSED");
        break;

      case ST_PAUSED:                     // BTN Resumes from the current point
        time_start   = millis();
        time_start_2 = millis();
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
  // Setpoint only advances encoder points while the shape drawing is running; homing/homed states hold encoder point 0
  if (state == ST_RUNNING && (millis() - time_start) > time_per_point) {
    point_index++; // adds 1 !!!
    time_start = millis();
    if (point_index >= shape_len[shape]) {
      state = ST_IDLE;     // stop after one pass; next press re-homes
      point_index   = 0;
      point_index_2 = 0;
      shape = (shape + 1) % NUM_SHAPES;   // next run draws the next shape
      return;              // the idle block switches the motors off next loop
    }
  }
  ref = shapes_m1[shape][point_index]; // sets reference point

  ATOMIC() { // disables interrupts on read. This is a precaution to ensure that printed values to Serial Monitor are equal to the calculated values at this point in time.
    pos = posi;
  }

  e = ref - pos; // calcs error

  if (d_update) {
    float d_raw = (e - e_prev) / dt;
    d_term = d_alpha * d_raw + (1 - d_alpha) * d_term;
    e_prev = e;

    e_sum += e * dt;
    if (ki > 0) e_sum = constrain(e_sum, -i_max / ki, i_max / ki);
  }

  u_f = (kp * e + ki * e_sum + kd * d_term); // calcs control signal amplitude from PID

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
  if (state == ST_RUNNING && (millis() - time_start_2) > time_per_point_2) {
    point_index_2 = (point_index_2 + 1) % shape_len[shape];   // wraps
    time_start_2 = millis();
  }
  ref_2 = shapes_m2[shape][point_index_2];

  ATOMIC() {
    pos_2 = posi_2;
  }

  e_2 = ref_2 - pos_2;

  if (d_update) {
    float d_raw_2 = (e_2 - e_prev_2) / dt;
    d_term_2 = d_alpha * d_raw_2 + (1 - d_alpha) * d_term_2;
    e_prev_2 = e_2;

    e_sum_2 += e_2 * dt;
    if (ki_2 > 0) e_sum_2 = constrain(e_sum_2, -i_max_2 / ki_2, i_max_2 / ki_2);
  }

  u_f_2 = (kp_2 * e_2 + ki_2 * e_sum_2 + kd_2 * d_term_2);

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

  ATOMIC() { pos = posi; pos_2 = posi_2; }
  e_prev   = shapes_m1[shape][point_index]     - pos;
  e_prev_2 = shapes_m2[shape][point_index_2] - pos_2;
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