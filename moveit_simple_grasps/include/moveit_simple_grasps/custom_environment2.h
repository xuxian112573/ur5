#include <moveit_visual_tools/moveit_visual_tools.h> // 显示掌握的简单工具

#ifndef BAXTER_PICK_PLACE__CUSTOM_ENVIRONMENT_
#define BAXTER_PICK_PLACE__CUSTOM_ENVIRONMENT_

namespace baxter_pick_place
{

// environment
static const std::string SUPPORT_SURFACE1_NAME = "monitor";
static const std::string SUPPORT_SURFACE2_NAME = "desk";
static const std::string SUPPORT_SURFACE3_NAME = "table";
static const std::string WALL1_NAME = "back_wall";
static const std::string WALL2_NAME = "right_wall";
static const std::string WALL3_NAME = "left_wall";

// 工作台尺寸
static const double TABLE_HEIGHT = 0.818;

static const double TABLE_WIDTH  = 0.87;
static const double TABLE_DEPTH  = 0.44;
static const double TABLE_X = 0.83;
static const double TABLE_Y = 0.15;

//对象尺寸
static const double OBJECT_SIZE = 0.04;

void createEnvironment(moveit_visual_tools::MoveItVisualToolsPtr visual_tools_)
{
  visual_tools_->cleanupCO(SUPPORT_SURFACE1_NAME);
  visual_tools_->cleanupCO(SUPPORT_SURFACE2_NAME);
  visual_tools_->cleanupCO(WALL1_NAME);
  visual_tools_->cleanupCO(WALL2_NAME);
  visual_tools_->cleanupCO(WALL3_NAME);

  // --------------------------------------------------------------------------------------------
  // Add objects to scene

  // Walls                            x,     y,     angle,  width, height, name
  visual_tools_->publishCollisionWall(-0.55, 0,     0,      2.2,   1.5,    WALL1_NAME);  // back wall
  visual_tools_->publishCollisionWall(0.05,  -1.1,  M_PI/2, 2.0,   1.5,    WALL2_NAME);  // baxter's right
  visual_tools_->publishCollisionWall(0.05,  1.1,   M_PI/2, 2.0,   1.5,    WALL3_NAME);  // baxter's left

  // Tables                            x,       y,       z, angle, width,       height,       depth,       name
  visual_tools_->publishCollisionTable(0.78,    -0.8,    0, 0,     0.4,         1.4,          0.47,        SUPPORT_SURFACE1_NAME); // computer monitor
  visual_tools_->publishCollisionTable(0.78,    -0.45,   0, 0,     0.4,         0.7,          0.47,        SUPPORT_SURFACE2_NAME); // my desk
  visual_tools_->publishCollisionTable(TABLE_X, TABLE_Y, 0, 0,     TABLE_WIDTH, TABLE_HEIGHT, TABLE_DEPTH, SUPPORT_SURFACE3_NAME); // andy table
}

double getTableHeight(double floor_offset)
{
  return TABLE_HEIGHT + floor_offset + OBJECT_SIZE / 2;
}

void getTableWidthRange(double &y_min, double &y_max)
{
  y_min = TABLE_Y - TABLE_WIDTH / 2;
  y_max = TABLE_Y + TABLE_WIDTH / 2;
}

void getTableDepthRange(double &x_min, double &x_max)
{
  x_min = TABLE_X - TABLE_DEPTH / 2;
  x_max = TABLE_X + TABLE_DEPTH / 2;
}

} // namespace

#endif
