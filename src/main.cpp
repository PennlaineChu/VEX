#include "vex.h"
#include "arm.h"
#include "note.h"
#include <cmath>
#include <algorithm>
extern vex::color selectedTeamColor;
using namespace vex;
competition Competition;


Drive chassis(
    // Specify your drive setup below. There are seven options:
    // ZERO_TRACKER_NO_ODOM, ZERO_TRACKER_ODOM, TANK_ONE_ENCODER, TANK_ONE_ROTATION, TANK_TWO_ENCODER, TANK_TWO_ROTATION, HOLONOMIC_TWO_ENCODER, and HOLONOMIC_TWO_ROTATION
    // For example, if you are not using odometry, put ZERO_TRACKER_NO_ODOM below:
    ZERO_TRACKER_ODOM,
    // Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
    // You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".
    // Left Motors:
    motor_group(L1, L2, L3),
    // Right Motors:
    motor_group(R1, R2, R3),
    // Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
    PORT12,
    // Input your wheel diameter. (4" omnis are actually closer to 4.125"):
    3.75,
    // 外部齒比，必須以小數形式表示，輸入齒數/輸出齒數格式。
    // 若您的馬達有 84 齒的齒輪，輪子有 60 齒的齒輪，此值將為 1.4。
    // 若馬達直接驅動輪子，則此值為 1：
    0.66666,
    // Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
    // For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
    360,
    /*---------------------------------------------------------------------------*/
    /*                                  PAUSE!                                   */
    /*                                                                           */
    /*  The rest of the drive constructor is for robots using POSITION TRACKING. */
    /*  If you are not using position tracking, leave the rest of the values as  */
    /*  they are.                                                                */
    /*---------------------------------------------------------------------------*/
    // If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.
    // FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
    // LF:      //RF:
    PORT12, -PORT14,
    // LB:      //RB:
    PORT13, -PORT20,
    // If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
    // If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
    // If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
    5,
    // Input the Forward Tracker diameter (reverse it to make the direction switch):
    -3.25,
    // Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
    // For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
    // This distance is in inches:
    5.2,
    // Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
    1,
    // Sideways tracker diameter (reverse to make the direction switch):
    -2.75,
    // Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
    5.5
);
// ---- helpers ----
static inline double clampd(double v, double lo, double hi){ return v<lo?lo:(v>hi?hi:v); }
static inline double sgn(double x){ return (x>=0)?1.0:-1.0; }
static inline double wrap180(double a){ while(a>180)a-=360; while(a<-180)a+=360; return a; }
constexpr double PI = 3.14159265358979323846;

// 依你目前的輪徑/齒比（Drive 設 3.75, 0.66666）
constexpr double WHEEL_DIAM_IN  = 3.75;
constexpr double EXT_GEAR_RATIO = 0.66666;
constexpr double WHEEL_CIRC_IN  = PI * WHEEL_DIAM_IN;

static double get_avg_inches(){
  double l_deg = std::fabs(L1.position(vex::deg));
  double r_deg = std::fabs(R1.position(vex::deg));
  double l_turn = (l_deg/360.0) * EXT_GEAR_RATIO;
  double r_turn = (r_deg/360.0) * EXT_GEAR_RATIO;
  return 0.5*(l_turn + r_turn) * WHEEL_CIRC_IN;
}

static void set_drive_volt(double leftV, double rightV){
  L1.spin(vex::fwd, leftV, vex::volt); L2.spin(vex::fwd, leftV, vex::volt); L3.spin(vex::fwd, leftV, vex::volt);
  R1.spin(vex::fwd, rightV, vex::volt); R2.spin(vex::fwd, rightV, vex::volt); R3.spin(vex::fwd, rightV, vex::volt);
}

// 平滑版：cos 加減速 + IMU 保角（強化：加入 heading I 項，較快加速）
void cos_move_distance_smooth(double distance_in, double angle_deg, double turn_maxV, double drive_maxV){
  double D = std::fabs(distance_in);
  if (D <= 0.0) return;

  L1.resetPosition(); R1.resetPosition();
  L1.setStopping(vex::brake); L2.setStopping(vex::brake); L3.setStopping(vex::brake);
  R1.setStopping(vex::brake); R2.setStopping(vex::brake); R3.setStopping(vex::brake);

  const double dir   = sgn(distance_in);
  const double Vmax  = clampd(drive_maxV, 0.0, 12.0);
  const double MIN_V = std::min(3.5, Vmax);

  // 速度輪廓參數
  // acc: 加速段長度, dec: 減速段長度, cruise: 等速段長度
  // acc 與 dec 比例可調整，但建議 acc 較短，dec 較長（較不會超過目標距離）
  // acc + dec 不得大於 D，否則會無法達成
  double acc = D * 0.22, dec = D * 0.41;
  if (acc + dec > D) { double s = D / (acc + dec); acc *= s; dec *= s; }
  const double cruise = std::max(0.0, D - acc - dec);

 
  const int    dt_ms = 10;
  const double kH    = 0.17;
  const double kHi   = 0.003;
  const double i_cap = turn_maxV * 0.5;
  const double settle_in = 0.30;

  double h_i = 0.0;
  double last_s = 0.0;
  int    settle_timer = 0;

 
  const double trimV = 0.0;

  while (true){
    double s = get_avg_inches();
    if (s >= D) break;

    
    double v;
    if (s < acc){
      double x = s / std::max(1e-6, acc);
      v = Vmax * 0.5 * (1.0 - std::cos(PI * x));
    } else if (s < acc + cruise){
      v = Vmax;
    } else {
      double x = (s - (D - dec)) / std::max(1e-6, dec);
      v = Vmax * 0.5 * (1.0 + std::cos(PI * x));
    }
    v = std::max(v, MIN_V) * dir;

 
    double h_err = wrap180(angle_deg - Inertial.heading());
    h_i += h_err * (dt_ms / 1000.0) * kHi;
    h_i  = clampd(h_i, -i_cap, i_cap);

    double turnV = clampd(kH * h_err + h_i, -turn_maxV, turn_maxV);


    double leftV  = clampd(v + turnV - trimV, -Vmax, Vmax);
    double rightV = clampd(v - turnV + trimV, -Vmax, Vmax);
    set_drive_volt(leftV, rightV);

    double ds = std::fabs(s - last_s);
    if ((D - s) <= settle_in && ds < 0.02) {
      settle_timer += dt_ms;
      if (settle_timer >= 150) break;
    } else {
      settle_timer = 0;
    }
    last_s = s;

    vex::wait(dt_ms, vex::msec);
  }

  set_drive_volt(0, 0);
  L1.stop(); L2.stop(); L3.stop();
  R1.stop(); R2.stop(); R3.stop();
}

int current_auton_selection = 0;
bool auto_started = false;
int air = 0;
int temp = 0;
int option = 0;
int hookcnt = 0;
bool airspace = false;
bool ran_auton = false; // 是否已經跑auto模式

void cylinderSwitch()
{
  intakeCylander = !intakeCylander;
}
void intakecylanderon()
{
  airspace = !airspace;
  intakeCylander = airspace;
}
void intakecylanderoff()
{
  intakeCylander = false;
}
void hookSwitch()
{
  if (hookcnt < 999)
    hookCylinder = !hookCylinder;
  hookcnt++;
}
void alignerSwitch()
{
  aligner = !aligner;
}
void hookOn()  { hookCylinder = true;  }
void hookOff() { hookCylinder = false; }
void alignerON()  { aligner = true;  }
void alignerOFF() { aligner = false; }

void hang() // 預留吊掛
{
  pushCylinder = !pushCylinder;
  intake.stop(brake);
}

void pre_auton(void)
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();
  Inertial.calibrate();
  while (Inertial.isCalibrating())
  {
    // 等待校準完成
    whitelight = 0;
  };
  Controller1.Screen.print("ok");
  redlight = 1;
  whitelight = 1;

  // 定義顏色
  vex::color red = vex::color::red;
  vex::color blue = vex::color::blue;
  vex::color white = vex::color::white;

  // 畫面刷新標記
  int previous_selection = -1;

  // 設定畫布區域
  const int usable_height = 240;            // 螢幕的有效高度
  const int row_height = usable_height / 2; // 每行的高度
  const int col_width = 480 / 5;            // 每列的寬度

  // 修改紅色區域的高度，讓紅色區域略多
  const int red_height = row_height + 6;  // 調整紅色區域的高度
  const int blue_height = row_height - 6; // 調整藍色區域的高度

  while (!auto_started)
  {
    // 每次只更新選擇的格子
    if (current_auton_selection != previous_selection)
    {
      previous_selection = current_auton_selection;

      for (int i = 0; i < 10; i++)
      {
        int x = (i % 5) * col_width;      // 計算格子的 x 座標
        int y = (i < 5) ? 0 : red_height; // 計算格子的 y 座標，前5個為紅色區域，後5個為藍色區域
        vex::color fillColor = (current_auton_selection == i) ? white : (i < 5 ? red : blue);

        // 畫格子背景
        Brain.Screen.setFillColor(fillColor);
        Brain.Screen.drawRectangle(x, y, col_width, (i < 5 ? red_height : blue_height));

        // 畫文字
        Brain.Screen.setFont(vex::fontType::mono20);
        Brain.Screen.setPenColor((current_auton_selection == i) ? red : white);

        if (i == 0)
          Brain.Screen.printAt(x + 10, y + 60, "RW_right ");
        if (i == 1)
          Brain.Screen.printAt(x + 10, y + 60, "R3_right ");
        if (i == 2)
          Brain.Screen.printAt(x + 10, y + 60, "RW_left");
        if (i == 3)
          Brain.Screen.printAt(x + 10, y + 60, "R5_left");
        if (i == 4)
          Brain.Screen.printAt(x + 10, y + 60, "R_SOLO");
        if (i == 5)
          Brain.Screen.printAt(x + 10, y + 60, "BW_left");
        if (i == 6)
          Brain.Screen.printAt(x + 10, y + 60, "B3_left");
        if (i == 7)
          Brain.Screen.printAt(x + 10, y + 60, "BW_right");
        if (i == 8)
          Brain.Screen.printAt(x + 10, y + 60, "B_SOLO");
        if (i == 9)
          Brain.Screen.printAt(x + 10, y + 60, "B_17022A");
      }
    }

    // 檢查觸控範圍
    if (Brain.Screen.pressing())
    {
      int touchX = Brain.Screen.xPosition();
      int touchY = Brain.Screen.yPosition();

      // 判斷觸控的是哪個格子
      int col = touchX / col_width;        // 每列寬度為 col_width
      int row = (touchY - 0) / row_height; // 計算行數，從紅色區域開始

      // 更新選擇的格子
      if (row >= 0 && row < 2)
      { // 確保觸控在有效範圍內
        current_auton_selection = col + row * 5;
      }

      wait(0.3, sec); // 防止重複觸發
    }

    wait(20, msec); // 確保程式執行穩定
  }
}

void autonomous(void)
{
  auto_started = true;
  ran_auton = true;
  // 根據選擇的自動任務來決定隊伍顏色
  if (current_auton_selection >= 0 && current_auton_selection <= 4)
  {
    selectedTeamColor = vex::color::red; // 紅隊
  }
  else if (current_auton_selection >= 5 && current_auton_selection <= 9)
  {
    selectedTeamColor = vex::color::blue; // 藍隊
  }
  else
  {
    selectedTeamColor = vex::color::black; // 預設為黑隊
  }
  switch (current_auton_selection)
  {

  case 0:
    RW_right();
    break;
  case 1:
    R3_right();
    break;
  case 2:
    RW_left();
    break;
  case 3:
    R5_left();
    break;
  case 4:
    skills();
    break;
  case 5:
    BW_left();
    break;
  case 6:
    B3_left();
    break;
  case 7:
    BW_right();
    break;
  case 8:
    B5_right();
    break;
  case 9:
    B_17022A();
    break;
  }
  
}

int momogoTask()
{
  while (true)
  {
    Optical_go.setLightPower(100, percent);
    if (airspace == 1)
    {
      Vision1.setLedColor(255, 0, 0);
      Vision2.setLedColor(255, 0, 0);
    }
    else if (Optical_go.isNearObject())
    {
      Vision1.setLedColor(255, 255, 255);
      Vision2.setLedColor(255, 255, 255);
      wait(0.1, sec);
      Vision1.setLedColor(0, 0, 0);
      Vision2.setLedColor(0, 0, 0);
      wait(0.1, sec);
    }
    else
    {
      Vision1.setLedColor(0, 255, 0);
      Vision2.setLedColor(0, 255, 0);
    }
  }
}

int intakeControlTask()
{
  intake.setMaxTorque(100, percent);

  while (true)
  {
    // 依優先權：L1 > L2 > R1 > R2
    if (Controller1.ButtonL1.pressing())
    {
      // L1：只動 intakedown 反轉
      intake.stop(coast);
      intakedown.spin(reverse, 12, volt);
    }
    else if (Controller1.ButtonL2.pressing())
    {
      // L2：只動 intakedown 正轉
      intake.stop(coast);
      intakedown.spin(forward, 12, volt);
    }
    else if (Controller1.ButtonR1.pressing())
    {
      // 原本功能保留：R1
      intake.spin(forward, 12, volt);
      intakedown.spin(reverse, 12, volt);
    }
    else if (Controller1.ButtonR2.pressing())
    {
      // 原本功能保留：R2
      intake.spin(forward, 12, volt);
      intakedown.spin(reverse, 12, volt);  
    }
    else
    {
      // 停止
      intake.stop(coast);
      intakedown.stop(coast);
    }

    wait(20, msec);
  }
  return 0;
}
void usercontrol(void)
{
  if (!ran_auton)
  {
    hang1.resetPosition();  
  }
  else
  {

    hang1.resetPosition();
  }
  
  task notetask(autonoteTask, 0);
  //---------------------------------------------------
  task momogo(momogoTask, 0);
  //-----------------------------------------------------
  task intake(intakeControlTask, 0);
  //-----------------------------------------------------
  task hangTask(hangControlTask, 0);
  //-----------------------------------------------------
  Controller1.ButtonY.pressed(hookSwitch);
  //-----------------------------------------------------
  Controller1.ButtonRight.pressed(intakecylanderon);
  //-----------------------------------------------------
  Controller1.ButtonB.pressed(hang); // 吊掛
  //-----------------------------------------------------
  Controller1.ButtonDown.pressed(alignerSwitch);
  //-----------------------------------------------------
  Controller1.ButtonR1.pressed(hookOn);
  Controller1.ButtonR1.pressed(alignerON);
  Controller1.ButtonR1.released(hookOff);
  Controller1.ButtonR1.released(alignerOFF);
  while (1)
  {
    if (Controller1.ButtonLeft.pressing())
    {
      selectedTeamColor = vex::color::red; // 紅隊
      Optical.setLightPower(100, percent); // 開啟燈光
    }
    else if (Controller1.ButtonRight.pressing())
    {
      selectedTeamColor = vex::color::blue; // 藍隊
      Optical.setLightPower(100, percent);  // 開啟燈光
    }
    else if (Controller1.ButtonUp.pressing())
    {
      selectedTeamColor = vex::color::black; // 無隊伍（禁用）
      Optical.setLightPower(0, percent);     // 關閉燈光
    }
    chassis.control_tank(100); // 底盤控制
  }
  wait(20, msec);
}
int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);
  }
}
