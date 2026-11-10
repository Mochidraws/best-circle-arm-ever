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
float kp = 6;            // proportional gain
int pos = 0;               // output encoder position from the gearbox
int e = 0;                 // error between ref and pos
int u_amplitude = 0;       // amplitude of the PID calculated control signal
float u_f = 0;             // float version of u_amplitude
int u_max_amplitude = 150; // control signal amplitude (must be < 256)
int u_sign = 0;            // required sign (Drive Direction) of control signal
long time_start = 0;
const int shape1_m1[] = {-243,-243,-243,-244,-244,-244,-244,-245,-245,-245,-245,-245,-246,-246,-246,-246,-246,-247,-247,-247,-247,-247,-248,-248,-248,-248,-248,-248,-248,-248,-249,-249,-249,-249,-249,-249,-249,-249,-249,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-250,-249,-249,-249,-249,-249,-249,-249,-249,-249,-249,-249,-248,-248,-248,-248,-248,-248,-248,-248,-248,-247,-247,-247,-247,-247,-247,-247,-246,-246,-246,-246,-246,-246,-245,-245,-245,-245,-245,-245,-244,-244,-244,-244,-244,-243,-243,-243,-243,-243,-242,-242,-242,-242,-242,-241,-241,-241,-241,-240,-240,-240,-240,-239,-239,-239,-239,-239,-238,-238,-238,-238,-237,-237,-237,-236,-236,-236,-236,-235,-235,-235,-235,-234,-234,-234,-233,-233,-233,-233,-232,-232,-232,-231,-231,-231,-230,-230,-230,-229,-229,-229,-229,-228,-228,-228,-227,-227,-227,-226,-226,-226,-225,-225,-225,-224,-224,-223,-223,-223,-222,-222,-222,-221,-221,-221,-220,-220,-220,-219,-219,-218,-218,-218,-217,-217,-217,-216,-216,-215,-215,-215,-214,-214,-213,-213,-213,-212,-212,-211,-211,-211,-210,-210,-209,-209,-208,-208,-208,-207,-207,-206,-206,-205,-205,-205,-204,-204,-203,-203,-202,-202,-201,-200,-199,-198,-197,-196,-195,-194,-193,-192,-191,-190,-189,-188,-187,-186,-185,-184,-183,-182,-181,-180,-179,-178,-177,-176,-176,-175,-174,-173,-172,-171,-170,-169,-168,-167,-166,-165,-164,-163,-162,-161,-160,-159,-158,-157,-156,-155,-154,-153,-152,-151,-150,-149,-148,-147,-146,-145,-144,-143,-142,-141,-140,-139,-138,-137,-136,-135,-134,-133,-132,-131,-130,-129,-128,-127,-126,-125,-124,-123,-122,-121,-120,-119,-118,-117,-116,-115,-114,-113,-112,-111,-110,-109,-108,-107,-106,-105,-104,-103,-102,-101,-100,-99,-98,-97,-96,-95,-94,-93,-92,-91,-90,-89,-88,-87,-86,-85,-84,-83,-82,-81,-80,-79,-78,-77,-76,-75,-74,-73,-72,-71,-70,-69,-68,-67,-66,-65,-64,-63,-62,-61,-60,-59,-58,-57,-56,-55,-54,-52,-51,-50,-49,-48,-47,-46,-45,-44,-43,-42,-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,20,21,22,23,24,25,26,27,28,29,30,31,33,34,35,36,37,38,39,40,41,42,44,45,46,47,48,49,50,51,52,54,53,53,53,52,52,52,51,51,51,50,50,50,50,49,49,49,48,48,48,48,47,47,47,47,46,46,46,46,45,45,45,45,44,44,44,44,44,43,43,43,43,43,42,42,42,42,42,42,41,41,41,41,41,41,40,40,40,40,40,40,40,39,39,39,39,39,39,39,39,38,38,38,38,38,38,38,38,38,38,38,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,36,36,36,36,36,36,36,36,36,36,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,38,38,38,38,38,38,38,38,38,38,38,39,39,39,39,39,39,39,39,39,40,40,40,40,40,40,40,41,41,41,41,41,41,41,42,42,42,42,42,42,43,43,43,43,43,44,44,44,44,44,45,45,45,45,45,46,46,46,46,47,47,47,47,48,48,48,48,49,49,49,49,50,50,50,50,51,51,51,52,52,52,52,53,53,53,54,54,54,55,55,55,55,56,56,56,57,57,57,58,58,58,59,59,60,60,60,61,61,61,62,62,62,63,63,64,63,62,61,60,59,58,57,56,55,54,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,8,7,6,5,4,3,2,1,0,-1,-2,-3,-4,-5,-6,-7,-8,-9,-11,-12,-13,-14,-15,-16,-17,-18,-19,-20,-21,-22,-23,-25,-26,-27,-28,-29,-30,-31,-32,-33,-34,-36,-37,-38,-39,-40,-41,-42,-43,-44,-46,-47,-48,-49,-50,-51,-52,-54,-55,-56,-57,-58,-59,-60,-62,-63,-64,-65,-66,-67,-69,-70,-71,-72,-73,-74,-76,-77,-78,-79,-80,-82,-83,-84,-85,-86,-88,-89,-90,-91,-92,-94,-95,-96,-97,-99,-100,-101,-102,-104,-105,-106,-107,-109,-110,-111,-112,-114,-115,-116,-118,-119,-120,-121,-123,-124,-125,-127,-128,-129,-131,-132,-133,-135,-136,-137,-139,-140,-141,-143,-144,-145,-147,-148,-149,-151,-152,-153,-155,-156,-158,-159,-160,-162,-163,-165,-166,-168,-169,-170,-172,-173,-175,-176,-178,-179,-180,-182,-183,-185,-186,-188,-189,-191,-192,-194,-195,-197,-198,-200,-201,-203,-204,-206,-208,-209,-211,-212,-214,-215,-217,-218,-220,-222,-223,-225,-226,-228,-230,-231,-233,-235,-236,-238,-239,-241
}; // shape 1 points for motor 1
const int shape2_m1[] = {836,834,832,830,829,827,825,823,822,820,818,816,815,813,811,810,808,806,805,803,801,800,798,796,795,793,791,790,788,787,785,783,782,780,779,777,776,774,772,771,769,768,766,765,763,762,760,759,757,756,754,753,751,750,749,747,746,744,743,741,740,739,737,736,734,733,732,730,729,727,726,725,723,722,721,719,718,717,715,714,713,711,710,709,707,706,705,704,702,701,700,698,697,696,695,693,692,691,690,688,687,686,685,684,682,681,680,679,678,676,675,674,673,672,670,669,668,667,666,665,663,662,661,660,659,658,657,655,654,653,652,651,650,649,648,646,645,644,643,642,641,640,639,638,637,636,634,633,632,631,630,629,628,627,626,625,624,623,622,621,620,619,618,617,616,615,614,613,612,611,610,609,608,607,606,605,604,603,602,601,600,599,598,597,596,595,594,593,592,591,590,589,588,587,586,585,584,583,582,581,580,579,578,577,577,576,575,574,573,572,571,570,569,568,567,566,565,565,564,563,562,561,560,559,558,557,556,556,555,554,553,552,551,550,549,548,548,547,546,545,544,543,542,541,541,540,539,538,537,536,535,535,534,533,532,531,530,529,529,528,527,526,525,524,523,523,522,521,520,519,518,518,517,516,515,514,513,513,512,511,510,509,508,508,507,506,505,504,503,503,502,501,500,499,499,498,497,496,495,495,494,493,492,491,491,490,489,488,487,487,486,485,484,483,483,482,481,480,479,479,478,477,476,475,475,474,473,472,471,471,470,469,468,467,467,467,468,469,470,470,471,472,473,473,474,475,476,476,477,478,479,479,480,481,482,482,483,484,485,485,486,487,488,488,489,490,491,491,492,493,493,494,495,496,496,497,498,499,499,500,501,501,502,503,504,504,505,506,506,507,508,509,509,510,511,511,512,513,514,514,515,516,516,517,518,518,519,520,521,521,522,523,523,524,525,525,526,527,527,528,529,529,530,531,531,532,533,534,534,535,536,536,537,538,538,539,540,540,541,542,542,543,543,544,545,545,546,547,547,548,549,549,550,551,551,552,553,553,554,554,555,556,556,557,558,558,559,559,560,561,561,562,563,563,564,564,565,566,566,567,567,568,569,569,570,570,571,572,572,573,573,574,575,575,576,576,577,578,578,579,579,580,580,581,582,582,583,583,584,584,585,586,586,587,587,588,588,589,589,590,591,591,592,592,593,593,594,594,595,595,596,596,597,597,598,599,599,600,600,601,601,602,602,603,603,604,604,605,605,606,606,607,607,608,608,609,609,610,610,611,611,612,612,613,613,613,614,614,615,615,616,616,617,617,618,618,619,619,620,620,620,621,621,622,622,623,623,623,624,624,625,625,626,626,627,627,627,628,628,629,629,629,630,630,631,631,631,632,632,633,633,633,634,634,635,635,635,636,636,637,637,637,638,638,638,639,639,640,640,640,641,641,641,642,642,642,643,643,643,644,644,644,645,645,646,646,646,647,647,647,647,648,648,648,649,649,649,650,650,650,651,651,651,652,652,652,652,653,653,654,654,655,656,656,657,657,658,658,659,659,660,661,661,662,662,663,663,664,664,665,665,666,667,667,668,668,669,669,670,670,671,671,672,673,673,674,674,675,675,676,676,677,677,678,678,679,680,680,681,681,682,682,683,683,684,684,685,685,686,687,687,688,688,689,689,690,690,691,691,692,692,693,693,694,694,695,696,696,697,697,698,698,699,699,700,700,701,701,702,702,703,703,704,705,705,706,706,707,707,708,708,709,709,710,710,711,711,712,712,713,713,714,715,715,716,716,717,717,718,718,719,719,720,720,721,721,722,722,723,723,724,724,725,725,726,727,727,728,728,729,729,730,730,731,731,732,732,733,733,734,734,735,735,736,736,737,738,738,739,739,740,740,741,741,742,742,743,743,744,744,745,745,746,746,747,747,748,749,749,750,750,751,751,752,752,753,753,754,754,755,755,756,756,757,758,758,759,759,760,760,761,761,762,762,763,763,764,764,765,765,766,767,767,768,768,769,769,770,770,771,771,772,772,773,774,774,775,775,776,776,777,777,778,778,779,780,780,781,781,782,782,783,783,784,784,785,786,786,787,787,788,788,789,789,790,791,791,792,792,793,793,794,795,795,796,796,797,797,798,798,799,800,800,801,801,802,803,803,804,804,805,805,806,807,807,808,808,809,810,810,811,811,812,812,813,814,814,815,815,816,817,817,818,819,819,820,820,821,822,822,823,823,824,825,825,826,827,827,828,828,829,830,830,831,832,832,833,834,834,835
}; // shape 2 points for motor 1
int point_index = 0;            // which setpoint motor is currently targeting
float time_per_point = 5;    // ms to spend on each encoder point
float e_sum = 0; // Integral sum
float ki = 3; // Integral control gain
const float i_max = 40; // Integral control signal limiter. Prevents extreme buildup and overshoot.
float kd = 0.12;     // Derivative control gain
int e_prev = 0;      // Arithmetic for Derivative error
float d_term = 0;    // filtered de/dt (counts/s)

// ---------- Motor 2 variables (independent of motor 1) ----------
volatile int posi_2 = 0;
float ref_2 = 0;
float kp_2 = 6;
int pos_2 = 0;
int e_2 = 0;
int u_amplitude_2 = 0;
float u_f_2 = 0;
int u_max_amplitude_2 = 150;
int u_sign_2 = 0;
long time_start_2 = 0;
const int shape1_m2[] = {-822,-821,-820,-819,-819,-818,-817,-816,-816,-815,-814,-813,-813,-812,-811,-810,-809,-809,-808,-807,-806,-806,-805,-804,-803,-802,-802,-801,-800,-799,-798,-798,-797,-796,-795,-794,-793,-793,-792,-791,-790,-789,-788,-788,-787,-786,-785,-784,-783,-783,-782,-781,-780,-779,-778,-778,-777,-776,-775,-774,-773,-772,-772,-771,-770,-769,-768,-767,-766,-765,-765,-764,-763,-762,-761,-760,-759,-758,-757,-757,-756,-755,-754,-753,-752,-751,-750,-749,-748,-748,-747,-746,-745,-744,-743,-742,-741,-740,-739,-738,-738,-737,-736,-735,-734,-733,-732,-731,-730,-729,-728,-727,-726,-725,-724,-723,-723,-722,-721,-720,-719,-718,-717,-716,-715,-714,-713,-712,-711,-710,-709,-708,-707,-706,-705,-704,-703,-702,-701,-700,-699,-698,-697,-696,-695,-694,-693,-692,-691,-690,-689,-688,-687,-686,-685,-684,-683,-682,-681,-680,-679,-678,-677,-676,-675,-674,-673,-672,-671,-670,-669,-668,-667,-666,-665,-664,-663,-662,-661,-660,-659,-658,-657,-656,-655,-654,-653,-652,-650,-649,-648,-647,-646,-645,-644,-643,-642,-641,-640,-639,-638,-637,-636,-634,-633,-632,-631,-630,-629,-628,-627,-626,-625,-624,-622,-621,-620,-619,-618,-617,-616,-615,-614,-613,-611,-610,-609,-608,-607,-606,-605,-604,-602,-601,-600,-599,-598,-597,-596,-594,-593,-592,-591,-590,-589,-588,-586,-585,-584,-583,-582,-581,-581,-580,-580,-579,-579,-578,-578,-577,-577,-576,-576,-575,-575,-574,-574,-573,-573,-572,-572,-571,-571,-570,-570,-569,-568,-568,-567,-567,-566,-566,-565,-565,-564,-563,-563,-562,-562,-561,-561,-560,-559,-559,-558,-558,-557,-556,-556,-555,-555,-554,-553,-553,-552,-552,-551,-550,-550,-549,-548,-548,-547,-547,-546,-545,-545,-544,-543,-543,-542,-541,-541,-540,-539,-539,-538,-537,-537,-536,-535,-535,-534,-533,-532,-532,-531,-530,-530,-529,-528,-528,-527,-526,-525,-525,-524,-523,-522,-522,-521,-520,-519,-519,-518,-517,-516,-516,-515,-514,-513,-513,-512,-511,-510,-510,-509,-508,-507,-506,-506,-505,-504,-503,-502,-502,-501,-500,-499,-498,-497,-497,-496,-495,-494,-493,-492,-492,-491,-490,-489,-488,-487,-486,-486,-485,-484,-483,-482,-481,-480,-479,-479,-478,-477,-476,-475,-474,-473,-472,-471,-470,-470,-469,-468,-467,-466,-465,-464,-463,-462,-461,-460,-459,-458,-457,-456,-455,-454,-453,-452,-451,-450,-449,-448,-447,-446,-445,-444,-443,-442,-441,-440,-439,-438,-437,-436,-435,-434,-433,-432,-431,-430,-429,-428,-427,-426,-425,-424,-422,-421,-420,-419,-418,-417,-416,-415,-414,-413,-411,-410,-409,-408,-407,-406,-405,-403,-402,-401,-400,-399,-398,-396,-395,-394,-393,-392,-390,-389,-388,-387,-385,-384,-383,-382,-381,-379,-378,-377,-375,-374,-375,-377,-378,-379,-381,-382,-383,-384,-385,-387,-388,-389,-390,-392,-393,-394,-395,-396,-398,-399,-400,-401,-402,-403,-405,-406,-407,-408,-409,-410,-411,-413,-414,-415,-416,-417,-418,-419,-420,-421,-422,-424,-425,-426,-427,-428,-429,-430,-431,-432,-433,-434,-435,-436,-437,-438,-439,-440,-441,-442,-443,-444,-445,-446,-447,-448,-449,-450,-451,-452,-453,-454,-455,-456,-457,-458,-459,-460,-461,-462,-463,-464,-465,-466,-467,-468,-469,-470,-470,-471,-472,-473,-474,-475,-476,-477,-478,-479,-479,-480,-481,-482,-483,-484,-485,-486,-486,-487,-488,-489,-490,-491,-492,-492,-493,-494,-495,-496,-497,-497,-498,-499,-500,-501,-502,-502,-503,-504,-505,-506,-506,-507,-508,-509,-510,-510,-511,-512,-513,-513,-514,-515,-516,-516,-517,-518,-519,-519,-520,-521,-522,-522,-523,-524,-525,-525,-526,-527,-528,-528,-529,-530,-530,-531,-532,-532,-533,-534,-535,-535,-536,-537,-537,-538,-539,-539,-540,-541,-541,-542,-543,-543,-544,-545,-545,-546,-547,-547,-548,-548,-549,-550,-550,-551,-552,-552,-553,-553,-554,-555,-555,-556,-556,-557,-558,-558,-559,-559,-560,-561,-561,-562,-562,-563,-563,-564,-565,-565,-566,-566,-567,-567,-568,-568,-569,-570,-570,-571,-571,-572,-572,-573,-573,-574,-574,-575,-575,-576,-576,-577,-577,-578,-578,-579,-579,-580,-580,-581,-581,-582,-583,-584,-585,-586,-588,-589,-590,-591,-592,-593,-594,-596,-597,-598,-599,-600,-601,-602,-604,-605,-606,-607,-608,-609,-610,-611,-613,-614,-615,-616,-617,-618,-619,-620,-621,-622,-624,-625,-626,-627,-628,-629,-630,-631,-632,-633,-634,-636,-637,-638,-639,-640,-641,-642,-643,-644,-645,-646,-647,-648,-649,-650,-652,-653,-654,-655,-656,-657,-658,-659,-660,-661,-662,-663,-664,-665,-666,-667,-668,-669,-670,-671,-672,-673,-674,-675,-676,-677,-678,-679,-680,-681,-682,-683,-684,-685,-686,-687,-688,-689,-690,-691,-692,-693,-694,-695,-696,-697,-698,-699,-700,-701,-702,-703,-704,-705,-706,-707,-708,-709,-710,-711,-712,-713,-714,-715,-716,-717,-718,-719,-720,-721,-722,-723,-723,-724,-725,-726,-727,-728,-729,-730,-731,-732,-733,-734,-735,-736,-737,-738,-738,-739,-740,-741,-742,-743,-744,-745,-746,-747,-748,-748,-749,-750,-751,-752,-753,-754,-755,-756,-757,-757,-758,-759,-760,-761,-762,-763,-764,-765,-765,-766,-767,-768,-769,-770,-771,-772,-772,-773,-774,-775,-776,-777,-778,-778,-779,-780,-781,-782,-783,-783,-784,-785,-786,-787,-788,-788,-789,-790,-791,-792,-793,-793,-794,-795,-796,-797,-798,-798,-799,-800,-801,-802,-802,-803,-804,-805,-806,-806,-807,-808,-809,-809,-810,-811,-812,-813,-813,-814,-815,-816,-816,-817,-818,-819,-819,-820,-821
}; // shape 1 points for motor 2
const int shape2_m2[] = {886,886,885,884,884,883,883,882,881,881,880,879,879,878,878,877,876,876,875,874,874,873,873,872,871,871,870,869,869,868,867,867,866,865,865,864,864,863,862,862,861,860,860,859,858,858,857,856,856,855,854,854,853,852,852,851,850,849,849,848,847,847,846,845,845,844,843,843,842,841,841,840,839,838,838,837,836,836,835,834,834,833,832,831,831,830,829,829,828,827,827,826,825,824,824,823,822,822,821,820,819,819,818,817,816,816,815,814,814,813,812,811,811,810,809,808,808,807,806,806,805,804,803,803,802,801,800,800,799,798,797,797,796,795,794,794,793,792,791,791,790,789,788,788,787,786,785,785,784,783,782,782,781,780,779,779,778,777,776,776,775,774,773,772,772,771,770,769,769,768,767,766,765,765,764,763,762,762,761,760,759,758,758,757,756,755,755,754,753,752,751,751,750,749,748,747,747,746,745,744,743,743,742,741,740,740,739,738,737,736,735,735,734,733,732,731,731,730,729,728,727,727,726,725,724,723,723,722,721,720,719,718,718,717,716,715,714,714,713,712,711,710,709,709,708,707,706,705,704,704,703,702,701,700,699,699,698,697,696,695,694,694,693,692,691,690,689,688,688,687,686,685,684,683,683,682,681,680,679,678,677,677,676,675,674,673,672,671,670,670,669,668,667,666,665,664,664,663,662,661,660,659,658,657,657,656,655,654,653,652,651,650,649,649,648,647,646,645,644,643,642,641,641,640,639,638,637,636,635,634,633,632,632,631,630,630,630,630,631,631,631,631,631,631,632,632,632,632,632,632,633,633,633,633,633,633,634,634,634,634,634,634,635,635,635,635,635,635,635,636,636,636,636,636,636,636,636,637,637,637,637,637,637,637,637,637,638,638,638,638,638,638,638,638,638,638,639,639,639,639,639,639,639,639,639,639,639,639,639,639,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,640,639,639,639,639,639,639,639,639,639,639,639,639,639,639,638,638,638,638,638,638,638,638,638,638,637,637,637,637,637,637,637,637,637,636,636,636,636,636,636,636,636,635,635,635,635,635,635,635,634,634,634,634,634,634,633,633,633,633,633,633,633,632,632,632,632,632,631,631,631,631,631,631,630,630,630,630,630,629,629,629,629,629,628,628,628,628,627,627,627,627,627,626,626,626,626,625,625,625,625,624,624,624,624,623,623,623,623,622,622,622,622,621,621,621,621,620,620,620,620,619,619,619,618,618,618,618,617,617,617,616,616,616,616,615,615,615,614,614,614,613,613,613,612,612,612,611,611,611,611,610,610,610,609,609,609,608,608,608,607,607,606,606,606,605,605,605,604,604,604,603,603,603,602,602,601,601,601,600,600,600,599,599,598,598,598,597,597,596,596,596,595,595,594,594,595,596,597,598,599,600,601,602,603,604,605,606,607,608,609,610,611,612,613,614,615,616,617,618,618,619,620,621,622,623,624,625,626,627,628,629,630,631,632,633,634,635,636,637,638,639,639,640,641,642,643,644,645,646,647,648,649,650,651,652,653,653,654,655,656,657,658,659,660,661,662,663,664,665,666,666,667,668,669,670,671,672,673,674,675,676,676,677,678,679,680,681,682,683,684,685,686,686,687,688,689,690,691,692,693,694,695,695,696,697,698,699,700,701,702,703,703,704,705,706,707,708,709,710,711,711,712,713,714,715,716,717,718,719,719,720,721,722,723,724,725,726,726,727,728,729,730,731,732,733,733,734,735,736,737,738,739,739,740,741,742,743,744,745,746,746,747,748,749,750,751,752,752,753,754,755,756,757,758,758,759,760,761,762,763,764,764,765,766,767,768,769,770,770,771,772,773,774,775,775,776,777,778,779,780,781,781,782,783,784,785,786,786,787,788,789,790,791,792,792,793,794,795,796,797,797,798,799,800,801,802,802,803,804,805,806,807,807,808,809,810,811,812,812,813,814,815,816,817,818,818,819,820,821,822,822,823,824,825,826,827,827,828,829,830,831,832,832,833,834,835,836,837,837,838,839,840,841,842,842,843,844,845,846,847,847,848,849,850,851,852,852,853,854,855,856,856,857,858,859,860,861,861,862,863,864,865,866,866,867,868,869,870,871,871,872,873,874,875,875,876,877,878,879,880,880,881,882,883,884,885,885
}; // shape 2 points for motor 2
int point_index_2 = 0;            // which setpoint we're currently targeting
float time_per_point_2 = 5;    // ms spent on each point
float e_sum_2 = 0; //Integral sum
float ki_2 = 3; //Integral control gain
const float i_max_2 = 40;
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
const int NUM_SHAPES = 1; // Change depending on number of different shape arrays to draw
int shape = 0;  // Target shape of run
// compile-time check that each shape's two motor arrays are the same length
static_assert(sizeof(shape1_m1) == sizeof(shape1_m2), "shape 1: M1/M2 lengths differ");
static_assert(sizeof(shape2_m1) == sizeof(shape2_m2), "shape 2: M1/M2 lengths differ");
// point_index at which the pen drops, per shape.
// 0 = lower as soon as homed, X = lower when past encoder point X
const int pen_drop_index[] = { 100, 0, 0 };   // shape 1, 2 and 3


// ---------- Homing / state machine ----------
enum RigState { ST_IDLE, ST_HOMING, ST_HOMED, ST_RUNNING, ST_PAUSED }; // available states
RigState state = ST_IDLE;                   // starts idle (motors off)

const int U_MAX_RUN  = 150;                 // Amplitude limit while drawing shape
const int U_MAX_HOME = 70;                  // Amplitude limit while homing
const int HOME_TOLERANCE = 5;               // Tolerance in encoder counts for state > ST_HOMED
const unsigned long HOME_SETTLE_MS = 7000;  // Settling time for how long motors must be within HOME_TOLERANCE
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
  if (state == ST_HOMED)        pen_down_cmd = (pen_drop_index[shape] == 0);          // drop if no running start
  else if (state == ST_RUNNING) pen_down_cmd = (point_index >= pen_drop_index[shape]); // drop once past the running start
  else if (state == ST_IDLE)    pen_down_cmd = false;
  penLoop();
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