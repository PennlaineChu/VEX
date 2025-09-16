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
static const double PI = 3.14159265358979323846;

// 依你目前的輪徑/齒比（Drive 設 3.75, 0.66666）
static const double WHEEL_DIAM_IN  = 3.25;
static const double EXT_GEAR_RATIO = 0.75;
static const double WHEEL_CIRC_IN  = PI * WHEEL_DIAM_IN;

static double deg_to_inches(double posDeg){
    double rotations = posDeg / 360.0;
    double distanceInches = rotations * (WHEEL_CIRC_IN * EXT_GEAR_RATIO);
    return distanceInches;
}

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

    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("current travel: %.2f in", s);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("target travel: %.2f in", D);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("H: %.2f", Inertial.heading());

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

int current_auton_selection = 6;
bool auto_started = false;
int air = 0;
int temp = 0;
int option = 0;
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
void shooterSwitch()
{
    shooter = !shooter;

}
void alignerSwitch()
{
  aligner = !aligner;
}
void shooterOn()  
  {
   shooter = true;  
  }
void shooterOff() {
   shooter = false; 
  }
void alignerON()  
  {
   aligner = true;  
  }
void alignerOFF() 
  {
   aligner = false; 
  }

void hang() // 預留吊掛
{
  pushCylinder = !pushCylinder;
  intake.stop(brake);
}

struct Rect { int x, y, w, h; };

static inline void fillRect(const Rect& r, vex::color fill, vex::color pen = vex::transparent) {
  Brain.Screen.setFillColor(fill);
  Brain.Screen.setPenColor(pen);
  Brain.Screen.drawRectangle(r.x, r.y, r.w, r.h);
}
static inline void strokeRect(const Rect& r, vex::color pen) {
  Brain.Screen.setFillColor(vex::transparent);
  Brain.Screen.setPenColor(pen);
  Brain.Screen.drawRectangle(r.x, r.y, r.w, r.h, vex::transparent);
}
static inline void drawCenteredText(const Rect& r, const std::string& s, vex::color pen=vex::white) {
  Brain.Screen.setPenColor(pen);
  int tw = Brain.Screen.getStringWidth(s.c_str());
  int th = Brain.Screen.getStringHeight("A");
  int px = r.x + (r.w - tw) / 2;
  int py = r.y + (r.h + th) / 2; // y 是 baseline
  Brain.Screen.printAt(px, py, false, s.c_str());
}

// 水平條（-100~100）＋內文字置中
static inline void drawAxisBarLabeled(const char* name, int val, const Rect& labelBox, const Rect& barBox) {
  // 左側標籤（置中）
  fillRect(labelBox, vex::color(30,30,30));
  strokeRect(labelBox, vex::color(80,80,80));
  drawCenteredText(labelBox, name, vex::white);

  // 條底
  strokeRect(barBox, vex::color(90,90,90));
  fillRect({barBox.x+1, barBox.y+1, barBox.w-2, barBox.h-2}, vex::color(20,20,20));

  // 0 中線
  int zeroX = barBox.x + barBox.w/2;
  Brain.Screen.setPenColor(vex::color(90,90,90));
  Brain.Screen.drawLine(zeroX, barBox.y+1, zeroX, barBox.y + barBox.h - 2);

  // 值條
  int v = val; if(v>100) v=100; if(v<-100) v=-100;
  int half = (barBox.w-2)/2;
  int pix = (v * half) / 100;
  if (pix != 0) {
    int bx = (pix>0) ? zeroX : (zeroX+pix);
    int bw = (pix>0) ? pix : -pix;
    fillRect({bx, barBox.y+2, bw, barBox.h-4}, (pix>0)?vex::color(0,140,220):vex::color(220,140,0));
  }

  // 內文字置中（顯示數值）
  char buf[16];
  snprintf(buf, sizeof(buf), "%4d", val);
  drawCenteredText(barBox, buf, vex::color(230,230,230));
}

// 按鍵方塊
static inline void drawButtonBox(const char* name, bool pressed, const Rect& r) {
  vex::color fill = pressed ? vex::color(0,110,0) : vex::color(40,40,40);
  vex::color pen  = pressed ? vex::color(0,220,0) : vex::color(90,90,90);
  fillRect(r, fill); strokeRect(r, pen); drawCenteredText(r, name, vex::white);
}

// 小型指南針（顯示 heading 0~360）
static inline void drawCompass(const Rect& r, double heading_deg) {
  fillRect(r, vex::color(30,30,30)); strokeRect(r, vex::color(100,100,100));
  // 圓
  int cx = r.x + r.w/2, cy = r.y + r.h/2;
  int rad = (r.w<r.h ? r.w : r.h)/2 - 4;
  Brain.Screen.setPenColor(vex::color(160,160,160));
  Brain.Screen.drawCircle(cx, cy, rad);
  // N 標記
  Brain.Screen.printAt(cx-4, r.y+12, "N");
  // 指針（0 度朝上）
  double radian = (heading_deg - 90.0) * 3.1415926535 / 180.0;
  int tipX = cx + int(rad * std::cos(radian));
  int tipY = cy + int(rad * std::sin(radian));
  Brain.Screen.setPenColor(vex::color(255,80,80));
  Brain.Screen.drawLine(cx, cy, tipX, tipY);
  // 數值
  char buf[32]; snprintf(buf, sizeof(buf), "%3.0f°", heading_deg);
  drawCenteredText({r.x, r.y + r.h - 16, r.w, 16}, buf, vex::white);
}

// 小型 ON/OFF 方塊（氣動）
static inline void drawPneuBox(const char* name, bool on, const Rect& r) {
  vex::color fill = on ? vex::color(0,110,0) : vex::color(40,40,40);
  vex::color pen  = on ? vex::color(0,220,0) : vex::color(90,90,90);
  fillRect(r, fill); strokeRect(r, pen); drawCenteredText(r, name, vex::white);
}

// ========= 分頁列（右側顯示 Auton，不擋內容） =========
enum DashTab { TAB_INPUTS=0, TAB_MOTORS=1 };
static inline DashTab drawTabs(DashTab current, const char* autonText) {
  Rect bar = {0,0,480,22}; // 矮一點，留內容空間
  fillRect(bar, vex::color(35,35,35)); strokeRect(bar, vex::color(90,90,90));

  Rect t1 = {8,  2, 92, 18};
  Rect t2 = {t1.x + t1.w + 4, 2, 92, 18};

  auto tabColor = [&](DashTab t){ return (t==current)?vex::color(60,100,180):vex::color(55,55,55); };
  fillRect(t1, tabColor(TAB_INPUTS));  strokeRect(t1, vex::color(120,120,120));
  fillRect(t2, tabColor(TAB_MOTORS));  strokeRect(t2, vex::color(120,120,120));
  drawCenteredText(t1, "Inputs", vex::white);
  drawCenteredText(t2, "Motors", vex::white);

  // 右側顯示 Auton
  Brain.Screen.setFont(vex::fontType::mono12);
  Brain.Screen.setPenColor(vex::color(220,220,220));
  Brain.Screen.printAt(216, 16, false, "Auton: %s", autonText);

  DashTab out = current;
  if (Brain.Screen.pressing()) {
    int x = Brain.Screen.xPosition(), y = Brain.Screen.yPosition();
    if (y>=t1.y && y<=t1.y+t1.h) {
      if (x>=t1.x && x<=t1.x+t1.w) out = TAB_INPUTS;
      else if (x>=t2.x && x<=t2.x+t2.w) out = TAB_MOTORS;
    }
  }
  return out;
}

// ========= 你的馬達清單 =========
extern motor L1,L2,L3,R1,R2,R3,intake,intakedown,hang1;
static motor* kMotors[] = { &L1,&R1,&L2,&R2,&L3,&L3,&intake,&intakedown,&hang1 };
static const char* kMotorNames[] = { "L1","R1","L2","R2","L3","R3","INTK","IDWN","HANG" };
static const int kMotorCount = sizeof(kMotors)/sizeof(kMotors[0]);

// ========= Dashboard（雙分頁：Inputs / Motors） =========
// ========= Dashboard（雙分頁：Inputs / Motors） =========
static inline void show_status_page(int selectedAuton) {
  while (Brain.Screen.pressing()) wait(10, msec);

  const char* labels[10] = {
    "R_right","R_left","R_right_F","R_left_F","R_solo",
    "B_right","B_left","B_right_F","B_left_F","B_solo"
  };

  DashTab tab = TAB_INPUTS;
  DashTab prevTab = TAB_INPUTS;
  Brain.Screen.setFont(vex::fontType::mono12);

  // 版面參數
  const int  MARGIN  = 8;
  const int  TAB_H   = 22;
  const Rect content = { MARGIN, TAB_H + MARGIN, 480 - 2*MARGIN, 240 - (TAB_H + 2*MARGIN) };

  // 各分頁初始化旗標（只在剛切入該分頁時做一次）
  bool inputsInit = false;
  bool motorsInit = false;

  // 先鋪一次全局底色
  fillRect({0,0,480,240}, vex::color(18,18,18));

  while (true) {
    // 先畫分頁列（不清整頁，避免把 LOGO 擦掉）
    tab = drawTabs(tab, labels[selectedAuton]);

    // 分頁切換：只清「內容區」並重置 init
    if (tab != prevTab) {
      fillRect(content, vex::color(18,18,18));
      strokeRect(content, vex::color(60,60,60));
      if (tab == TAB_INPUTS)  inputsInit = false;
      if (tab == TAB_MOTORS)  motorsInit = false;
      prevTab = tab;
    }

    if (tab == TAB_INPUTS) {
      // ===== Inputs 頁 =====
      // 第一次進來：畫 LOGO（之後不再從 SD 讀，避免閃爍）
      if (!inputsInit) {
        const int barH = 20;
        const int gapY = 8;
        const int axTop = content.y;

        // A4 條下面顯示 LOGO.bmp（240x240）
        const int imgX = 35;                                        // 你要的 X
        const int imgY = axTop + 3*(barH+gapY) + barH + 10;         // A4 底下 10px

        if (Brain.SDcard.isInserted() && Brain.SDcard.exists("LOGO.bmp")) {
          Brain.Screen.drawImageFromFile("LOGO.bmp", imgX, imgY);
        } else {
          Brain.Screen.setPenColor(vex::color(200,200,200));
          Brain.Screen.printAt(imgX, imgY + 20, false, "LOGO.bmp not found");
        }
        inputsInit = true;
      }

      // —— 每幀只更新動態元件（不清整頁、也不重畫 LOGO） ——
      const int labelW = 36;
      const int barW   = (int)(content.w * 0.55);
      const int barH   = 20;
      const int gapY   = 8;
      const int axLeft = content.x;
      const int axTop  = content.y;

      int a1 = Controller1.Axis1.position();
      int a2 = Controller1.Axis2.position();
      int a3 = Controller1.Axis3.position();
      int a4 = Controller1.Axis4.position();

      drawAxisBarLabeled("A1", a1, {axLeft,             axTop + 0*(barH+gapY), labelW, barH},
                                {axLeft+labelW+6,       axTop + 0*(barH+gapY), barW,   barH});
      drawAxisBarLabeled("A2", a2, {axLeft,             axTop + 1*(barH+gapY), labelW, barH},
                                {axLeft+labelW+6,       axTop + 1*(barH+gapY), barW,   barH});
      drawAxisBarLabeled("A3", a3, {axLeft,             axTop + 2*(barH+gapY), labelW, barH},
                                {axLeft+labelW+6,       axTop + 2*(barH+gapY), barW,   barH});
      drawAxisBarLabeled("A4", a4, {axLeft,             axTop + 3*(barH+gapY), labelW, barH},
                                {axLeft+labelW+6,       axTop + 3*(barH+gapY), barW,   barH});

      // 右上小指南針
      Rect compass = { content.x + content.w - 90, axTop, 80, 80 };
      drawCompass(compass, Inertial.heading(degrees));

      // 右側：按鍵 3×4
      int gridX = content.x + content.w - (3*70 + 2*6);
      int gridY = compass.y + compass.h + 6;
      int bw    = 70, bh = 20, sp = 6;

      drawButtonBox("A",   Controller1.ButtonA.pressing(),   {gridX + 0*(bw+sp), gridY + 0*(bh+sp), bw, bh});
      drawButtonBox("B",   Controller1.ButtonB.pressing(),   {gridX + 1*(bw+sp), gridY + 0*(bh+sp), bw, bh});
      drawButtonBox("X",   Controller1.ButtonX.pressing(),   {gridX + 2*(bw+sp), gridY + 0*(bh+sp), bw, bh});

      drawButtonBox("Y",   Controller1.ButtonY.pressing(),   {gridX + 0*(bw+sp), gridY + 1*(bh+sp), bw, bh});
      drawButtonBox("L1",  Controller1.ButtonL1.pressing(),  {gridX + 1*(bw+sp), gridY + 1*(bh+sp), bw, bh});
      drawButtonBox("L2",  Controller1.ButtonL2.pressing(),  {gridX + 2*(bw+sp), gridY + 1*(bh+sp), bw, bh});

      drawButtonBox("R1",  Controller1.ButtonR1.pressing(),  {gridX + 0*(bw+sp), gridY + 2*(bh+sp), bw, bh});
      drawButtonBox("R2",  Controller1.ButtonR2.pressing(),  {gridX + 1*(bw+sp), gridY + 2*(bh+sp), bw, bh});
      drawButtonBox("Up",  Controller1.ButtonUp.pressing(),  {gridX + 2*(bw+sp), gridY + 2*(bh+sp), bw, bh});

      drawButtonBox("Down", Controller1.ButtonDown.pressing(), {gridX + 0*(bw+sp), gridY + 3*(bh+sp), bw, bh});
      drawButtonBox("Left", Controller1.ButtonLeft.pressing(), {gridX + 1*(bw+sp), gridY + 3*(bh+sp), bw, bh});
      drawButtonBox("Right",Controller1.ButtonRight.pressing(),{gridX + 2*(bw+sp), gridY + 3*(bh+sp), bw, bh});
    
      // 底部：氣動狀態
      int pneuY = content.y + content.h - 20;
      int pneuW = 68, pneuH = 18, pneuSP = 6;
      int pneuX = content.x;

      drawPneuBox("no status", redlight.value(),       {pneuX + 0*(pneuW+pneuSP), pneuY, pneuW, pneuH});
      drawPneuBox("no status", whitelight.value(),     {pneuX + 1*(pneuW+pneuSP), pneuY, pneuW, pneuH});
      drawPneuBox("INTK",      intakeCylander.value(), {pneuX + 2*(pneuW+pneuSP), pneuY, pneuW, pneuH});
      drawPneuBox("PUSH",      pushCylinder.value(),   {pneuX + 3*(pneuW+pneuSP), pneuY, pneuW, pneuH});
      drawPneuBox("SHOT",      shooter.value(),        {pneuX + 4*(pneuW+pneuSP), pneuY, pneuW, pneuH});
      drawPneuBox("ALIGN",     aligner.value(),        {pneuX + 5*(pneuW+pneuSP), pneuY, pneuW, pneuH});
    }
    else {
      // ===== Motors 頁（放大字體 + 逐行清除避免陰影） =====
      if (!motorsInit) {
        // 清內容區一次、畫框與標題
        fillRect(content, vex::color(18,18,18));
        strokeRect({content.x, content.y, content.w, content.h - 2}, vex::color(90,90,90));
        Brain.Screen.setPenColor(vex::color(200,200,200));
        Brain.Screen.setFont(vex::fontType::mono20);    // 放大字體
        // 標題往下擺，避免貼到 tabs
        Brain.Screen.printAt(content.x, content.y + 4, "Motor position (deg)");
        motorsInit = true;
      }

      // 版面配置（兩欄）
      const int colW = (content.w - 16) / 2;
      const int c1x  = content.x + 8;
      const int c2x  = content.x + 8 + colW;

      // 行高比字高大些，整體下移
      const int rowH = 26;                 // 搭配 mono20
      int rowTop = content.y + 28;         // ★ 往下移，避免貼到標題與 tabs

      // 顏色
      vex::color bg = vex::color(18,18,18);
      vex::color fg = vex::white;

      // 逐行更新：先清該行，再用不透明文字印上，避免陰影
      for (int i = 0; i < kMotorCount; ++i) {
        int colX = (i % 2 == 0) ? c1x : c2x;
        int rowY = rowTop + (i / 2) * rowH;

        // 清這一行的小區塊（不影響其它區域）
        Rect rline = { colX, rowY - 18, colW - 8, rowH };
        fillRect(rline, bg);

        const char* name = (i < (int)(sizeof(kMotorNames)/sizeof(kMotorNames[0]))) ? kMotorNames[i] : "M?";
        double posDeg = 0.0;
        if (kMotors[i]) {
            posDeg = deg_to_inches(kMotors[i]->position(degrees));
        }
        Brain.Screen.setPenColor(fg);
        Brain.Screen.setFillColor(bg);
        Brain.Screen.printAt(colX, rowY, /*bOpaque=*/true, "%-6s %8.1f", name, posDeg);
      }
    }

    // 觸控返回（點 tabs 區域只切換，不返回）
    if (Brain.Screen.pressing()) {
      int ty = Brain.Screen.yPosition();
      if (!(ty >= 0 && ty <= TAB_H)) {
        while (Brain.Screen.pressing()) wait(10, msec);
        break;   // 回到 pre_auton 的選單頁
      }
    }

    wait(100, msec);
  }
}

// ======================== 第一頁（選擇auto） =========================
void pre_auton(void)
{
  
  vexcodeInit();
  default_constants();

/*// ===== SD 卡測試 =====
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(vex::fontType::mono12);
  Brain.Screen.setPenColor(vex::white);

  if (Brain.SDcard.isInserted()) {
    Brain.Screen.printAt(10, 20, false, "SD card detected.");

    if (Brain.SDcard.exists("field.bmp")) {
      Brain.Screen.printAt(10, 40, false, "Loading field.bmp ...");
      bool ok = Brain.Screen.drawImageFromFile("field.bmp", 0, 0);
      if (ok) {
        Brain.Screen.printAt(10, 60, false, "Image loaded OK.");
      } else {
        Brain.Screen.printAt(10, 60, false, "Image load failed.");
      }
    } else {
      Brain.Screen.printAt(10, 40, false, "field.bmp not found.");
    }
  } else {
    Brain.Screen.printAt(10, 20, false, "No SD card detected.");
  }
  wait(10,sec);
  */// =======================================================================
  Inertial.calibrate();

  while (Inertial.isCalibrating()) {
    whitelight = 0;
  }
  Controller1.Screen.print("ok");
  redlight = 1;
  whitelight = 1;

  vex::color red   = vex::color::red;
  vex::color blue  = vex::color::blue;
  vex::color white = vex::color::white;

  int previous_selection = -1;

  const int cols      = 5;
  const int screen_w  = 480;
  const int screen_h  = 240;
  const int col_width = screen_w / cols;

  const int red_height  = screen_h / 2 + 6;
  const int blue_height = screen_h - red_height;

  const char* labels[10] = {
    "R_right","R_left","R_right_F","R_left_F","R_solo",
    "B_right","B_left","B_right_F","B_left_F","B_solo"
  };

  Brain.Screen.setFont(vex::fontType::mono20);

  while (!auto_started)
  {
    if (current_auton_selection != previous_selection)
    {
      previous_selection = current_auton_selection;

      for (int i = 0; i < 10; i++)
      {
        int col = i % cols;
        bool top = (i < cols);

        int x = col * col_width;
        int y = top ? 0 : red_height;
        int h = top ? red_height : blue_height;

        vex::color fillColor = (current_auton_selection == i) ? white : (top ? red : blue);
        vex::color textColor = (current_auton_selection == i) ? red   : white;

        Brain.Screen.setFillColor(fillColor);
        Brain.Screen.drawRectangle(x, y, col_width, h);

        Rect r { x, y, col_width, h };
        drawCenteredText(r, labels[i], textColor);
        
      }
    }

    if (Brain.Screen.pressing())
    {
      int touchX = Brain.Screen.xPosition();
      int touchY = Brain.Screen.yPosition();

      int col = touchX / col_width;
      int row = (touchY < red_height) ? 0 : (touchY < red_height + blue_height ? 1 : -1);

      if (col >= 0 && col < cols && (row == 0 || row == 1)) {
        current_auton_selection = col + row * cols;
        

        // 進入第二頁（Dashboard）
        
        show_status_page(current_auton_selection);
      }

      wait(0.3, sec);
    }

    wait(20, msec);
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
    R_right();
    break;
  case 1:
    R_left();
    break;
  case 2:
    R_right_final();
    break;
  case 3:
    R_left_final();
    break;
  case 4:
    R_solo();
    break;
  case 5:
    B_right();
    break;
  case 6:
    B_left();
    break;
  case 7:
    B_right_final();
    break;
  case 8:
    B_left_final();
    break;
  case 9:
    B_solo();
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
      intakedown.spin(forward, 12, volt);
    }
    else if (Controller1.ButtonL2.pressing())
    {
      // L2：只動 intakedown 正轉
      intake.stop(coast);
      intakedown.spin(reverse, 12, volt);
    }
    else if (Controller1.ButtonR1.pressing())
    {
      // 原本功能保留：R1
      intake.spin(forward, 12, volt);
      intakedown.spin(forward, 12, volt);
    }
    else if (Controller1.ButtonR2.pressing())
    {
      // 原本功能保留：R2
      intake.spin(forward, 12, volt);
      intakedown.spin(forward, 12, volt);  
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
    // 若未跑過 auto
  }
  else
  {
    // 若已跑過 auto
  }
  
  task notetask(autonoteTask, 0);
  //---------------------------------------------------
  task momogo(momogoTask, 0);
  //-----------------------------------------------------
  task intake(intakeControlTask, 0);
  //-----------------------------------------------------
  task hangTask(hangControlTask, 0);
  //-----------------------------------------------------
  Controller1.ButtonY.pressed(shooterSwitch);
  //-----------------------------------------------------
  Controller1.ButtonRight.pressed(intakecylanderon);
  //-----------------------------------------------------
  Controller1.ButtonB.pressed(hang); //屁股
  //-----------------------------------------------------
  Controller1.ButtonDown.pressed(alignerSwitch);
  //-----------------------------------------------------
  Controller1.ButtonR1.pressed(shooterOn);
  Controller1.ButtonR1.pressed(alignerON);
  Controller1.ButtonR1.released(shooterOff);
  Controller1.ButtonR1.released(alignerON);
  while (1)
  {
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
